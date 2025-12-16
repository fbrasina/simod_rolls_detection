// ROS
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/server/simple_action_server.h>

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// STL
#include <fstream>
#include <stdint.h>
#include <vector>
#include <set>
#include <cmath>
#include <map>
#include <mutex>
#include <random>

#include <simod_rolls_detection/DetectPacksAction.h>

#include <roll_pack_detection.h>

typedef ros::NodeHandle Node;
typedef sensor_msgs::Image ImageMsg;
typedef sensor_msgs::CameraInfo CameraInfoMsg;

class PacksDetectionNode
{
  public:
  typedef actionlib::SimpleActionServer<simod_rolls_detection::DetectPacksAction> ActionServer;

  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> Vector3fVector;

  typedef std::vector<std::string> StringVector;

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointXYZRGBCloud;

  typedef PackDetection::ReferencePointsPtr ReferencePointsPtr;

  typedef uint64_t uint64;

  template <typename T> static T SQR(const T & t) {return t * t; }

  PacksDetectionNode(std::shared_ptr<Node> nodeptr): m_nodeptr(nodeptr)
  {
    m_log = [this](const uint level, const std::string & message) {this->Log(level, message); };

    m_nodeptr->param<std::string>("rgb_image_topic", m_rgb_image_topic, "rgb_image_topic");
    m_nodeptr->param<std::string>("depth_image_topic", m_depth_image_topic, "depth_image_topic");
    m_nodeptr->param<std::string>("camera_info_topic", m_camera_info_topic, "camera_info_topic");

    m_nodeptr->param<std::string>("world_frame_id", m_world_frame_id, "map");

    m_nodeptr->param<bool>("no_depth", m_no_depth, false);

    Uint64Param("random_seed", m_random_seed, int(std::random_device()()));
    srand(m_random_seed);
    m_log(1, "Random seed is " + std::to_string(m_random_seed));

    m_nodeptr->param<std::string>("detect_packs_action", m_detect_packs_action, "/detect_packs_action");
    m_as.reset(new ActionServer(*nodeptr, m_detect_packs_action,
                                boost::function<void(const simod_rolls_detection::DetectPacksGoalConstPtr &)>(
                                  [this](const simod_rolls_detection::DetectPacksGoalConstPtr & goal){this->Run(goal); }),
                                false));

    ROS_INFO("roll_pack_detection_node: subscribing to color topic %s", m_rgb_image_topic.c_str());
    ROS_INFO("roll_pack_detection_node: subscribing to camera_info topic %s", m_camera_info_topic.c_str());
    m_rgb_image_sub = m_nodeptr->subscribe<sensor_msgs::Image>(m_rgb_image_topic, 1,
                                                               boost::function<void(const sensor_msgs::Image &)>(
                                                              [this](const sensor_msgs::Image & msg){this->RgbImageCallback(msg); }));
    m_depth_image_sub = m_nodeptr->subscribe<sensor_msgs::Image>(m_depth_image_topic, 1,
                                                                 boost::function<void(const sensor_msgs::Image &)>(
                                                                 [this](const sensor_msgs::Image & msg){this->DepthImageCallback(msg); }));
    m_camera_info_sub = m_nodeptr->subscribe<sensor_msgs::CameraInfo>(m_camera_info_topic, 1,
                                                                      boost::function<void(const sensor_msgs::CameraInfo &)>(
                                                                      [this](const sensor_msgs::CameraInfo & msg){
                                                                        this->CameraInfoCallback(msg);
                                                                      }));

    m_nodeptr->param<int>("discard_first_camera_frames", m_discard_first_camera_frames, 0);

    {
      const StringVector publish_image_names = PackDetection::GetImagePublisherNames();
      for (const std::string & s : publish_image_names)
      {
        const std::string topic_name = s + "_image";
        m_image_publishers[s] = m_nodeptr->advertise<sensor_msgs::Image>(topic_name, 1);
      }
    }

    {
      const StringVector publish_cloud_names = PackDetection::GetCloudPublisherNames();
      for (const std::string & s : publish_cloud_names)
      {
        const std::string topic_name = s + "_cloud";
        m_cloud_publishers[s] = m_nodeptr->advertise<sensor_msgs::PointCloud2>(topic_name, 1);
      }
    }

    m_as->start();
  }

  void Uint64Param(const std::string name, uint64 & outp, const uint64 & defp)
  {
    int ioutp;
    m_nodeptr->param<int>(name, ioutp, int(defp));
    outp = ioutp;
  }

  void Run(const simod_rolls_detection::DetectPacksGoalConstPtr & goalptr)
  {
    const simod_rolls_detection::DetectPacksGoal & goal = *goalptr;

    cv::Mat rgb_image;
    cv::Mat depth_image;
    std::shared_ptr<PackDetection::Intrinsics> camera_info;
    ROS_INFO("roll_pack_detection_node: action start.");
    {
      ros::Rate wait_rate(100);
      std::unique_lock<std::mutex> lock(m_mutex);
      if (m_discard_first_camera_frames)
        ROS_INFO("roll_pack_detection_node: first %d camera frames will be discarded.",
                 int(m_discard_first_camera_frames));
      for (int i = 0; i < m_discard_first_camera_frames + 1; i++)
      {
        m_last_rgb_image = cv::Mat();
        m_last_depth_image = cv::Mat();
        m_last_camera_info.reset();
        while (m_last_rgb_image.empty() || (!m_no_depth && m_last_depth_image.empty()) || !m_last_camera_info)
        {
          lock.unlock();
          ROS_INFO_THROTTLE(2.0, "roll_pack_detection_node: waiting for images...");
          wait_rate.sleep();
          lock.lock();

          if (!ros::ok())
            return;
        }

        if (!ros::ok())
          return;
      }

      rgb_image = m_last_rgb_image;
      if (!m_no_depth)
        depth_image = m_last_depth_image;
      camera_info = m_last_camera_info;
    } // lock released here
    ROS_INFO("roll_pack_detection_node: received images.");

    PackDetection::ReferenceImageVector reference_images;
    for (const simod_rolls_detection::ReferenceImage & reference_image_msg : goal.reference_images)
    {
      const std::string reference_image_filename = reference_image_msg.reference_image_filename;
      const std::string reference_mask_filename = reference_image_msg.reference_mask_filename;
      const std::string reference_description_filename = reference_image_msg.reference_description_filename;

      cv::Mat reference_image;
      m_log(1, "roll_pack_detection_node: Loading reference image " + reference_image_filename);
      reference_image = cv::imread(reference_image_filename);
      if (!reference_image.data)
      {
        m_log(3, "Could not read image!");
        m_as->setAborted(simod_rolls_detection::DetectPacksResult());
        return;
      }

      cv::Mat reference_image_mask;
      if (!reference_mask_filename.empty())
      {
        m_log(1, "roll_pack_detection_node: Loading reference mask image " + reference_mask_filename);
        reference_image_mask = cv::imread(reference_mask_filename, cv::IMREAD_GRAYSCALE);
        if (!reference_image_mask.data)
        {
          m_log(3, "Could not read image!");
          m_as->setAborted(simod_rolls_detection::DetectPacksResult());
          return;
        }
      }

      m_log(1, "Loading reference description " + reference_description_filename);
      PackDetection::ReferencePointsPtr reference_points = PackDetection::LoadReferencePoints(reference_description_filename, m_log);
      if (!reference_points)
      {
        m_log(3, "Could not read reference points!");
        m_as->setAborted(simod_rolls_detection::DetectPacksResult());
        return;
      }

      PackDetection::ReferenceImage ri;
      ri.reference_image = reference_image;
      ri.reference_mask = reference_image_mask;
      ri.reference_points = *reference_points;
      reference_images.push_back(ri);
    }

    Eigen::Affine3d camera_pose;
    tf::poseMsgToEigen(goal.camera_pose, camera_pose);

    PackDetection::Config config;

    config.reverse_image = goal.flip_image;

    m_nodeptr->param<float>("max_valid_depth", config.max_valid_depth, config.max_valid_depth);
    Uint64Param("nfeatures_reference", config.nfeatures_reference, config.nfeatures_reference);
    Uint64Param("nfeatures_image", config.nfeatures_image, config.nfeatures_image);
    Uint64Param("ransac_iterations", config.ransac_iterations, config.ransac_iterations);
    Uint64Param("ransac_refine_iterations", config.ransac_refine_iterations, config.ransac_refine_iterations);
    m_nodeptr->param<float>("feature_ratio_thresh", config.feature_ratio_thresh, config.feature_ratio_thresh);
    Uint64Param("knn_K", config.knn_K, config.knn_K);
    m_nodeptr->param<float>("reproj_threshold_px", config.reproj_threshold_px, config.reproj_threshold_px);
    m_nodeptr->param<float>("reproj_refine_threshold_px", config.reproj_refine_threshold_px, config.reproj_refine_threshold_px);
    m_nodeptr->param<float>("reproj_closed_form_threshold_px", config.reproj_closed_form_threshold_px,
                            config.reproj_closed_form_threshold_px);
    m_nodeptr->param<float>("match_spread_score_max_diff", config.match_spread_score_max_diff, config.match_spread_score_max_diff);
    m_nodeptr->param<float>("translation_px_weight_x", config.translation_px_weight_x, config.translation_px_weight_x);
    m_nodeptr->param<float>("translation_px_weight_y", config.translation_px_weight_y, config.translation_px_weight_y);
    m_nodeptr->param<float>("max_error_for_huber_loss", config.max_error_for_huber_loss, config.max_error_for_huber_loss);
    m_nodeptr->param<float>("max_scaling", config.max_scaling, config.max_scaling);
    m_nodeptr->param<float>("max_non_uniform_scaling", config.max_non_uniform_scaling, config.max_non_uniform_scaling);
    m_nodeptr->param<float>("max_feature_diff_angle", config.max_feature_diff_angle, config.max_feature_diff_angle);
    m_nodeptr->param<float>("min_object_score", config.min_object_score, config.min_object_score);
    Uint64Param("sanity_max_objects", config.sanity_max_objects, config.sanity_max_objects);
    m_nodeptr->param<float>("pose_depth_weight", config.pose_depth_weight, config.pose_depth_weight);
    m_nodeptr->param<float>("pose_px_weight", config.pose_px_weight, config.pose_px_weight);

    PackDetection rpd(config,
                      m_log,
                      [this](const std::string & name, const cv::Mat & image, const std::string & encoding) {
                        this->PublishImage(image, encoding, name);
                      },
                      [this](const std::string & name, const PointXYZRGBCloud & cloud) {
                        this->PublishCloud(cloud, name);
                      },
                      rand()
                      );

    PackDetection::DetectedPackVector detected_packs =
      rpd.FindMultipleObjects(rgb_image, depth_image, reference_images,
                              *camera_info, camera_pose.cast<float>());

    ROS_INFO_STREAM("roll_pack_detection_node: found: " << detected_packs.size() << " packs.");

    simod_rolls_detection::DetectPacksResult result;

    for (const PackDetection::DetectedPack & detected_pack : detected_packs)
    {
      Eigen::Affine3d p = detected_pack.center_pose;

      geometry_msgs::Pose pose_msg;
      tf::poseEigenToMsg(p, pose_msg);
      result.pack_poses.push_back(pose_msg);

      result.pack_edge_x_left.push_back(detected_pack.detected_left_width);
      result.pack_edge_x_right.push_back(detected_pack.detected_right_width);

      result.pack_height.push_back(detected_pack.height);
      result.pack_depth.push_back(detected_pack.depth);

      result.pack_reference_id.push_back(detected_pack.reference_id);
      result.is_upside_down.push_back(detected_pack.is_upside_down);
    }

    ROS_INFO("roll_pack_detection_node: action end.");
    m_as->setSucceeded(result);
  }

  void RgbImageCallback(const sensor_msgs::Image & msg)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    try
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      m_last_rgb_image = cv_ptr->image;
    }
    catch (const cv_bridge::Exception & e)
    {
      ROS_ERROR("RgbImageCallback: Could not convert from '%s' to 'bgr8'.", msg.encoding.c_str());
      return;
    }
  }

  void DepthImageCallback(const sensor_msgs::Image & msg)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    try
    {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
      m_last_depth_image = cv_ptr->image;
    }
    catch (const cv_bridge::Exception & e)
    {
      ROS_ERROR("DepthImageCallback: Could not convert from '%s' to '16UC1'.", msg.encoding.c_str());
      return;
    }
  }

  void CameraInfoCallback(const sensor_msgs::CameraInfo & msg)
  {
    std::unique_lock<std::mutex> lock(m_mutex);

    const float fx = msg.K[0];
    const float fy = msg.K[4];
    const float cx = msg.K[2];
    const float cy = msg.K[5];

    std::shared_ptr<PackDetection::Intrinsics> ci(new PackDetection::Intrinsics);
    ci->fx = fx;
    ci->fy = fy;
    ci->cx = cx;
    ci->cy = cy;

    m_last_camera_info = ci;
  }

  void Log(const uint level, const std::string & message)
  {
    switch (level)
    {
    case 0:
      ROS_DEBUG("roll_pack_detection_node: %s", message.c_str());
      break;
    case 1:
      ROS_INFO("roll_pack_detection_node: %s", message.c_str());
      break;
    case 2:
      ROS_WARN("roll_pack_detection_node: %s", message.c_str());
      break;
    case 3:
      ROS_ERROR("roll_pack_detection_node: %s", message.c_str());
      break;
    case 4:
      ROS_FATAL("roll_pack_detection_node: %s", message.c_str());
      break;
    default:
      ROS_ERROR("roll_pack_detection_node: Invalid logger level %d, message was '%s'", int(level), message.c_str());
    }
  }

  void PublishImage(const cv::Mat &image, const std::string &encoding, const std::string & name)
  {
    if (m_image_publishers.find(name) == m_image_publishers.end())
    {
      ROS_WARN("roll_pack_detection_node: Could not find image publisher with name %s", name.c_str());
      return;
    }

    PublishImage(image, encoding, m_image_publishers[name]);
  }

  void PublishImage(const cv::Mat & image, const std::string & encoding, ros::Publisher & pub)
  {
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    cv_ptr->image = image;
    cv_ptr->encoding = encoding;
    ImageMsg img = *cv_ptr->toImageMsg();
    pub.publish(img);
  }

  void PublishCloud(const PointXYZRGBCloud & cloud, const std::string & name)
  {
    if (m_cloud_publishers.find(name) == m_cloud_publishers.end())
    {
      ROS_WARN("roll_pack_detection_node: Could not find cloud publisher with name %s", name.c_str());
      return;
    }

    PublishCloud(cloud, m_cloud_publishers[name]);
  }

  void PublishCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, ros::Publisher & pub)
  {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = m_world_frame_id;
    output.header.stamp = ros::Time::now();
    pub.publish(output);
  }

  private:
  std::shared_ptr<Node> m_nodeptr;

  ros::Subscriber m_rgb_image_sub;
  ros::Subscriber m_depth_image_sub;
  ros::Subscriber m_camera_info_sub;

  std::map<std::string, ros::Publisher> m_image_publishers;
  std::map<std::string, ros::Publisher> m_cloud_publishers;

  std::shared_ptr<ActionServer> m_as;

  std::string m_rgb_image_topic;
  std::string m_depth_image_topic;
  std::string m_camera_info_topic;
  std::string m_detect_packs_action;

  cv::Mat m_last_rgb_image;
  cv::Mat m_last_depth_image;
  std::shared_ptr<PackDetection::Intrinsics> m_last_camera_info;
  bool m_no_depth;

  std::string m_world_frame_id;

  PackDetection::LogFunction m_log;

  ros::Timer m_timer;
  std::mutex m_mutex;

  uint64 m_random_seed;

  int m_discard_first_camera_frames;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roll pack detection");
  std::shared_ptr<Node> nodeptr(new Node("~"));
  ROS_INFO("roll_pack_detection_node started");

  PacksDetectionNode pd(nodeptr);
  ros::spin();

  return 0;
}
