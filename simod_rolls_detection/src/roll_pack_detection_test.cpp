// ROS
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <fstream>
#include <stdint.h>
#include <vector>
#include <cmath>
#include <memory>

#include <simod_rolls_detection/DetectPacksAction.h>

typedef ros::NodeHandle Node;
typedef sensor_msgs::Image ImageMsg;
typedef sensor_msgs::CameraInfo CameraInfoMsg;

class RollPackDetectionTest
{
  public:

  typedef actionlib::SimpleActionClient<simod_rolls_detection::DetectPacksAction> ActionClient;

  typedef geometry_msgs::TransformStamped TransformStampedMsg;

  typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > Affine3dVector;
  typedef std::vector<float> FloatVector;
  typedef uint64_t uint64;

  typedef std::vector<std::string> StringVector;

  template <typename T> static T SQR(const T & t) {return t * t; }

  struct CameraInfo
  {
    float fx, fy;
    float cx, cy;
  };

  RollPackDetectionTest(std::shared_ptr<Node> nodeptr): m_nodeptr(nodeptr), m_tf_listener(m_tf_buffer)
  {
    m_timer = m_nodeptr->createTimer(ros::Duration(0.0), [this](const ros::TimerEvent &){this->Run(); }, true);

    m_nodeptr->param<std::string>("rgb_filename", m_image_file_name, "");
    m_nodeptr->param<std::string>("depth_filename", m_depth_file_name, "");
    m_nodeptr->param<std::string>("camera_info_filename", m_camera_info_file_name, "");
    m_nodeptr->param<std::string>("camera_pose_filename", m_camera_pose_file_name, "");

    // no more than 10 reference images
    // add suffixes _0, _1... etc. except for first one
    for (int i = -1; i < 10; i++)
    {
      const std::string suffix = (i < 0) ? "" : ("_" + std::to_string(i));
      std::string image_filename;
      m_nodeptr->param<std::string>("reference_image_filename" + suffix, image_filename, "");
      std::string image_mask_filename;
      m_nodeptr->param<std::string>("reference_mask_filename" + suffix, image_mask_filename, "");
      std::string image_points_filename;
      m_nodeptr->param<std::string>("reference_description_filename" + suffix, image_points_filename, "");

      if (image_filename == "")
        continue;

      if (image_mask_filename == "")
      {
        ROS_ERROR("reference_image_mask_filename is empty, even if image filename is %s", image_filename.c_str());
        continue;
      }

      if (image_points_filename == "")
      {
        ROS_ERROR("reference_image_points_filename is empty, even if image points filename is %s", image_filename.c_str());
        continue;
      }

      m_reference_image_filename.push_back(image_filename);
      m_reference_mask_filename.push_back(image_mask_filename);
      m_reference_description_filename.push_back(image_points_filename);
    }

    m_nodeptr->param<bool>("flip_image", m_flip_image, true);

    m_nodeptr->param<bool>("use_real_camera", m_use_real_camera, false);

    m_nodeptr->param<std::string>("detect_packs_action", m_detect_packs_action, "/detect_packs_action");
    m_ac.reset(new ActionClient(*nodeptr, m_detect_packs_action, true));

    m_nodeptr->param<std::string>("rgb_image_topic", m_rgb_image_topic, "rgb_image_topic");
    m_nodeptr->param<std::string>("depth_image_topic", m_depth_image_topic, "depth_image_topic");
    m_nodeptr->param<std::string>("camera_info_topic", m_camera_info_topic, "camera_info_topic");
    m_rgb_image_pub = nodeptr->advertise<sensor_msgs::Image>(m_rgb_image_topic, 1);
    m_depth_image_pub = nodeptr->advertise<sensor_msgs::Image>(m_depth_image_topic, 1);
    m_camera_info_pub = nodeptr->advertise<sensor_msgs::CameraInfo>(m_camera_info_topic, 1);

    m_nodeptr->param<std::string>("world_frame_id", m_world_frame_id, "map");
    m_nodeptr->param<std::string>("camera_frame_id", m_camera_frame_id, "camera");

    m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();

    std::string marker_topic;
    m_nodeptr->param<std::string>("marker_topic", marker_topic, "/detect_pack_markers");
    m_markers_pub = nodeptr->advertise<visualization_msgs::MarkerArray>(marker_topic, 1);
  }

  void Load(cv::Mat & rgb_image, cv::Mat & depth_image, CameraInfo & camera_info, Eigen::Affine3f & camera_pose)
  {
    {
      ROS_INFO("loading camera_info file %s", m_camera_info_file_name.c_str());
      camera_info = LoadCameraInfo(m_camera_info_file_name);
    }

    rgb_image = cv::imread(m_image_file_name);

    if (!rgb_image.data)
    {
      ROS_FATAL("could not load rgb image: %s", m_image_file_name.c_str());
      std::exit(1);
    }

    depth_image = cv::imread(m_depth_file_name, cv::IMREAD_ANYDEPTH);

    if (!depth_image.data)
    {
      ROS_FATAL("could not load depth image: %s", m_depth_file_name.c_str());
      std::exit(2);
    }

    if (!MatrixFromFile(m_camera_pose_file_name, camera_pose))
    {
      ROS_FATAL("could not load camera_pose: %s", m_camera_pose_file_name.c_str());
      std::exit(3);
    }
  }

  CameraInfo LoadCameraInfo(const std::string filename)
  {
    CameraInfo result;

    std::ifstream ifile(filename);
    if (!ifile)
    {
      ROS_FATAL("could not find camera_info file: %s", filename.c_str());
      std::exit(1);
    }

    std::string line;
    while (std::getline(ifile, line))
    {
      std::istringstream istr(line);
      std::string field;
      istr >> field;
      if (field == "fx")
        istr >> result.fx;
      else if (field == "fy")
        istr >> result.fy;
      else if (field == "cx")
        istr >> result.cx;
      else if (field == "cy")
        istr >> result.cy;
      else
      {
        ROS_FATAL("invalid line in camera info file: %s", line.c_str());
        std::exit(1);
      }

      if (!istr)
      {
        ROS_FATAL("could not parse line in camera info file: %s", line.c_str());
        std::exit(1);
      }
    }

    return result;
  }

  bool MatrixFromFile(const std::string & filename, Eigen::Affine3f & matrix)
  {
    std::ifstream file(filename);

    for (uint64 i = 0; i < 3; ++i)
      for (uint64 j = 0; j < 4; ++j)
      {
        float v;
        file >> v;
        matrix.matrix()(i, j) = v;
      }

    if (!file)
      return false;
    return true;
  }

  void Run()
  {
    cv::Mat rgb_image, depth_image;
    Eigen::Affine3f camera_pose;

    ROS_INFO("roll_pack_detection_test: loading");

    CameraInfo camera_info;
    if (!m_use_real_camera)
      Load(rgb_image, depth_image, camera_info, camera_pose);

    if (!m_use_real_camera)
    {
      TransformStampedMsg t;
      tf::transformEigenToMsg(camera_pose.cast<double>(), t.transform);

      t.header.stamp = ros::Time::now();
      t.header.frame_id = m_world_frame_id;
      t.child_frame_id = m_camera_frame_id;

      ROS_INFO("roll_pack_detection_test: sending simulated camera pose to TF.");
      m_tf_broadcaster->sendTransform(t);
    }

    geometry_msgs::TransformStamped transformStamped;
    const double MAX_WAIT = 5.0; // wait at most 5 seconds for the transform
    try
    {
      ROS_INFO("roll_pack_detection_test: tf: waiting for camera pose between '%s' and '%s'",
               m_world_frame_id.c_str(), m_camera_frame_id.c_str());
      transformStamped = m_tf_buffer.lookupTransform(m_world_frame_id, m_camera_frame_id,
                                                     ros::Time(0), ros::Duration(MAX_WAIT));
      Eigen::Affine3d camera_pose_d;
      tf::transformMsgToEigen(transformStamped.transform, camera_pose_d);
      camera_pose = camera_pose_d.cast<float>();
      ROS_INFO("roll_pack_detection_test: tf: received camera pose.");
    }
    catch (tf2::TransformException &ex)
    {
      ROS_FATAL("roll_pack_detection_test: tf: could not find camera pose between '%s' and '%s' within %f seconds: %s",
                m_world_frame_id.c_str(), m_camera_frame_id.c_str(), double(MAX_WAIT), ex.what());
      std::exit(6);
    }

    simod_rolls_detection::DetectPacksGoal goal;
    for (uint64 i = 0; i < m_reference_image_filename.size(); i++)
    {
      simod_rolls_detection::ReferenceImage reference_image;
      reference_image.reference_image_filename = m_reference_image_filename[i];
      reference_image.reference_mask_filename = m_reference_mask_filename[i];
      reference_image.reference_description_filename = m_reference_description_filename[i];
      goal.reference_images.push_back(reference_image);
    }

    goal.flip_image = m_flip_image;

    tf::poseEigenToMsg(camera_pose.cast<double>(), goal.camera_pose);

    ROS_INFO("roll_pack_detection_test: waiting for server");
    m_ac->waitForServer();

    ROS_INFO("roll_pack_detection_test: sending goal");
    m_ac->sendGoal(goal);

    if (!m_use_real_camera)
    {
      ROS_INFO("roll_pack_detection_test: sleeping");
      ros::Duration(0.5).sleep();

      ROS_INFO("roll_pack_detection_test: publishing");
      PublishImage(rgb_image, "bgr8", m_rgb_image_pub);
      PublishImage(depth_image, "16UC1", m_depth_image_pub);

      sensor_msgs::CameraInfo camera_info_msg;
      camera_info_msg.K[0] = camera_info.fx;
      camera_info_msg.K[4] = camera_info.fy;
      camera_info_msg.K[2] = camera_info.cx;
      camera_info_msg.K[5] = camera_info.cy;
      m_camera_info_pub.publish(camera_info_msg);
    }

    ROS_INFO("roll_pack_detection_test: waiting for result.");
    m_ac->waitForResult();

    simod_rolls_detection::DetectPacksResult result = *(m_ac->getResult());

    ROS_INFO("roll_pack_detection_test: received: %d packs.", int(result.pack_poses.size()));

    Affine3dVector box_poses;
    FloatVector pack_edge_x_left;
    FloatVector pack_edge_x_right;
    FloatVector pack_heights;
    FloatVector pack_depths;
    for (uint64 i = 0; i < result.pack_poses.size(); i++)
    {
      ROS_INFO("roll_pack_detection_test: pack %d, type %d, is_upside_down: %s",
               int(i), int(result.pack_reference_id[i]), (result.is_upside_down[i] ? "YES" : "NO"));

      Eigen::Affine3d pose;
      tf::poseMsgToEigen(result.pack_poses[i], pose);
      box_poses.push_back(pose);

      pack_edge_x_left.push_back(result.pack_edge_x_left[i]);
      pack_edge_x_right.push_back(result.pack_edge_x_right[i]);
      pack_heights.push_back(result.pack_height[i]);
      pack_depths.push_back(result.pack_depth[i]);
    }

    // publishing to TF for visualization
    int increment = 0;
    for (uint64 i = 0; i < box_poses.size(); i++)
    {
      const Eigen::Affine3d p = box_poses[i];

      const Eigen::Affine3d p_left = p * Eigen::Translation3d(-Eigen::Vector3d::UnitX() * pack_edge_x_left[i]);
      const Eigen::Affine3d p_right = p * Eigen::Translation3d(Eigen::Vector3d::UnitX() * pack_edge_x_right[i]);

      TransformStampedMsg t;
      t.header.stamp = ros::Time::now();
      t.header.frame_id = m_world_frame_id;

      tf::transformEigenToMsg(p, t.transform);
      t.child_frame_id = "box_center_" + std::to_string(increment);
      m_tf_broadcaster->sendTransform(t);

      tf::transformEigenToMsg(p_left, t.transform);
      t.child_frame_id = "box_left_" + std::to_string(increment);
      m_tf_broadcaster->sendTransform(t);

      tf::transformEigenToMsg(p_right, t.transform);
      t.child_frame_id = "box_right_" + std::to_string(increment);
      m_tf_broadcaster->sendTransform(t);

      increment++;
    }

    visualization_msgs::MarkerArray markers;
    markers = PacksToVisualizationMarkers(box_poses,
                                          pack_edge_x_left,
                                          pack_edge_x_right,
                                          pack_heights,
                                          pack_depths,
                                          Eigen::Vector3f(1.0f, 0.0f, 0.0f), // red
                                          "detected_packs");
    m_markers_pub.publish(markers);

    ROS_INFO("roll_pack_detection_test: end.");
  }

  visualization_msgs::MarkerArray PacksToVisualizationMarkers(const Affine3dVector & box_poses,
                                                              const FloatVector & pack_edge_x_left,
                                                              const FloatVector & pack_edge_x_right,
                                                              const FloatVector & heights,
                                                              const FloatVector & depths,
                                                              const Eigen::Vector3f & base_color,
                                                              const std::string & ns)
  {
    visualization_msgs::MarkerArray result;

    uint64 marker_id = 0;
    visualization_msgs::Marker marker;
    marker.header.frame_id = m_world_frame_id;
    marker.action = marker.DELETEALL;
    marker.id = marker_id++;
    marker.ns = ns;
    result.markers.push_back(marker);

    for (uint64 i = 0; i < box_poses.size(); i++)
    {
      Eigen::Affine3d center_pose = box_poses[i];

      const Eigen::Vector3d center_shift = Eigen::Vector3d::UnitX() *
                                           (pack_edge_x_right[i] - pack_edge_x_left[i]) / 2.0;
      center_pose = center_pose * Eigen::Translation3d(center_shift);

      const Eigen::Matrix3d rot_mat = center_pose.linear();
      const Eigen::Quaterniond q(rot_mat);

      marker.header.frame_id = m_world_frame_id;
      marker.action = marker.ADD;
      marker.type = marker.CUBE;
      marker.id = marker_id++;
      marker.ns = ns;

      marker.scale.x = pack_edge_x_left[i] + pack_edge_x_right[i];
      marker.scale.y = depths[i];
      marker.scale.z = heights[i];

      marker.pose.position.x = center_pose.translation().x();
      marker.pose.position.y = center_pose.translation().y();
      marker.pose.position.z = center_pose.translation().z();
      marker.pose.orientation.x = q.x();
      marker.pose.orientation.y = q.y();
      marker.pose.orientation.z = q.z();
      marker.pose.orientation.w = q.w();

      marker.color.r = base_color.x();
      marker.color.g = base_color.y();
      marker.color.b = base_color.z();
      marker.color.a = 0.5;

      result.markers.push_back(marker);
    }

    return result;
  }

  void PublishImage(const cv::Mat & image, const std::string & encoding, ros::Publisher & pub)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->image = image;
    cv_ptr->encoding = encoding;
    ImageMsg img = *cv_ptr->toImageMsg();
    pub.publish(img);
  }

  private:
  std::shared_ptr<Node> m_nodeptr;

  ros::Publisher m_rgb_image_pub;
  ros::Publisher m_camera_info_pub;
  ros::Publisher m_depth_image_pub;
  std::string m_rgb_image_topic;
  std::string m_depth_image_topic;
  std::string m_camera_info_topic;

  StringVector m_reference_image_filename;
  StringVector m_reference_mask_filename;
  StringVector m_reference_description_filename;
  bool m_flip_image;

  bool m_use_real_camera;

  std::string m_world_frame_id;
  std::string m_camera_frame_id;

  ros::Publisher m_markers_pub;

  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;
  std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

  std::string m_detect_packs_action;

  std::shared_ptr<ActionClient> m_ac;

  ros::Timer m_timer;

  std::string m_image_file_name;
  std::string m_depth_file_name;
  std::string m_camera_info_file_name;
  std::string m_camera_pose_file_name;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roll_pack_detection_test");
  std::shared_ptr<Node> nodeptr(new Node("~"));
  ROS_INFO("roll_pack_detection_test started");

  RollPackDetectionTest pd(nodeptr);
  ros::spin();

	return 0;
}
