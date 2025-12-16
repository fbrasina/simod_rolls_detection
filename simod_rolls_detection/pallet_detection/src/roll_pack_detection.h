#ifndef ROLL_PACK_DETECTION_H
#define ROLL_PACK_DETECTION_H

// OpenCV
#include <opencv2/opencv.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// STL
#include <fstream>
#include <stdint.h>
#include <vector>
#include <set>
#include <cmath>
#include <map>
#include <random>
#include <memory>
#include <functional>

using std::placeholders::_1;
using std::placeholders::_2;

#include "roll_pack_detection_model.h"
#include "roll_pack_pose_model.h"

class PackDetection
{
  public:
  typedef std::vector<int> IntVector;
  typedef std::shared_ptr<IntVector> IntVectorPtr;
  typedef std::vector<float> FloatVector;
  typedef std::vector<double> DoubleVector;
  typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > Vector4fVector;
  typedef std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > Vector4dVector;
  typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dVector;
  typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > Vector2dVector;
  typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
  typedef std::vector<cv::Point2f> Point2fVector;
  typedef std::vector<cv::Mat> MatVector;
  typedef std::vector<bool> BoolVector;
  typedef std::vector<BoolVector> BoolVectorVector;
  typedef std::vector<FloatVector> FloatVectorVector;
  typedef std::vector<std::string> StringVector;

  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef int64_t int64;
  typedef uint8_t uint8;
  typedef uint16_t uint16;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::pair<uint64, uint64> Uint64Uint64Pair;
  typedef std::pair<Uint64Uint64Pair, Uint64Vector> Uint64VectorUint64PairPair;
  typedef std::map<Uint64Uint64Pair, Uint64Vector> Uint64VectorUint64PairMap;

  static constexpr int ESTIMATOR_COUNT = 7;

  typedef RollPackDetectionEstimator<ESTIMATOR_COUNT> MyRollPackDetectionEstimator;
  typedef typename MyRollPackDetectionEstimator::HomographyArray HomographyArray;
  typedef typename MyRollPackDetectionEstimator::MyRollPackDetectionModel RollPackDetectionModel;
  typedef std::shared_ptr<RollPackDetectionModel> RollPackDetectionModelPtr;

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointXYZRGBCloud;

  static constexpr float MY_NAN = std::numeric_limits<float>::quiet_NaN();

  template <typename T> static T SQR(const T & t) {return t * t; }

  typedef std::function<void(const int level, const std::string & message)> LogFunction;
  typedef std::function<void(const std::string & image_name, const cv::Mat & image, const std::string & encoding)> PublishImageFunction;
  typedef std::function<void(const std::string & cloud_name, const PointXYZRGBCloud & cloud)> PublishCloudFunction;

  static StringVector GetImagePublisherNames();
  static StringVector GetCloudPublisherNames();

  struct Config
  {
    float max_valid_depth = 3.0f;

    uint64 nfeatures_reference = 500;
    uint64 nfeatures_image = 5000;
    uint64 ransac_iterations = 200 * 1000; // 10 * 1000 * 1000;
    uint64 ransac_refine_iterations = 10;
    float feature_ratio_thresh = 0.85f;
    uint64 knn_K = 11;
    float reproj_threshold_px = 40;
    float reproj_refine_threshold_px = 5;
    float reproj_closed_form_threshold_px = 10;
    float match_spread_score_max_diff = 3.0f;
    float translation_px_weight_x = 0.01;
    float translation_px_weight_y = 0.03;
    float max_error_for_huber_loss = 3.0;
    float max_scaling = 3.0f;
    float max_non_uniform_scaling = 1.5f;
    float max_feature_diff_angle = M_PI / 4.0f;
    float min_object_score = 10.0f;
    uint64 sanity_max_objects = 20;

    float pose_depth_weight = 1.0f;
    float pose_px_weight = 1.0f;

    bool reverse_image = true;
  };

  struct DetectedPack
  {
    Eigen::Affine3d center_pose = Eigen::Affine3d::Identity();
    double height = 0.0f, width = 0.0f, depth = 0.0f;
    double detected_left_width = 1.0f;
    double detected_right_width = 1.0f;
    uint64 reference_id = uint64(-1);
    bool is_upside_down = false;
  };
  typedef std::vector<DetectedPack> DetectedPackVector;

  PackDetection(const Config & config,
                const LogFunction & log_function,
                const PublishImageFunction & publish_image_function,
                const PublishCloudFunction & cloud_function,
                const uint64 random_seed);

  struct Intrinsics
  {
    float fx, fy;
    float cx, cy;
  };

  struct RansacFindHomographyResult
  {
    cv::Mat homography;
    cv::Mat initial_homography;
    RollPackDetectionModel detection_model;
    float score = 0.0f;
    float detection_model_score = 0.0f;
    float detection_model_final_cost = std::numeric_limits<float>::quiet_NaN();
    Uint64Vector consensus;
    Uint64Vector detection_model_consensus;
    std::vector<cv::DMatch> consensus_matches;
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    Eigen::Vector3d detected_nominal_size = Eigen::Vector3d::Ones();
    double detected_left_width = 1.0f;
    double detected_right_width = 1.0f;
    bool valid = false;
    uint64 reference_id = uint64(-1);
    bool is_upside_down = false;
  };

  struct ReferencePoint
  {
    cv::Point2f reference;
    cv::Point2f real;

    ReferencePoint() {}

    ReferencePoint(const cv::Point2f & ref, const cv::Point2f & re)
    {
      reference = ref;
      real = re;
    }
  };

  struct ReferencePoints
  {
    std::map<std::string, ReferencePoint> pts;

    Point2fVector element_centers;

    float box_width, box_height, box_depth;
    float pixel_size_in_meters;
  };
  typedef std::shared_ptr<ReferencePoints> ReferencePointsPtr;

  struct ReferenceImage
  {
    cv::Mat reference_image;  // bgr8
    cv::Mat reference_mask;   // uint8, boolean
    ReferencePoints reference_points;
  };
  typedef std::vector<ReferenceImage> ReferenceImageVector;

  struct ReferenceImageFeatures
  {
    cv::Mat reference_grayscale;
    std::vector<cv::KeyPoint> keypoints_reference;
    cv::Mat descriptors_reference;
    ReferencePoints reference_points;

    std::vector<cv::DMatch> matches;
    FloatVector matches_score;
    BoolVectorVector matches_compatibility;
    Uint64VectorVector matches_comp_graph;
    FloatVectorVector matches_scale;
  };
  typedef std::vector<ReferenceImageFeatures> ReferenceImageFeaturesVector;

  static ReferencePointsPtr LoadReferencePoints(const std::string & filename, LogFunction & log);

  HomographyArray CVHomographyToArray(const cv::Mat & h);

  RollPackDetectionModelPtr EstimateModelMinSq(const double reference_size_x, const double reference_size_y,
                                               const Vector2dVector & reference_points,
                                               const Vector2dVector & observed_points,
                                               const double cost_x, const double cost_y);

  float FindConsensus(const double image_width, const double image_height,
                      const Vector2dVector & reference_points, const Vector2dVector & observed_points,
                      const FloatVector & cons_matches_score,
                      double reproj_threshold,
                      const RollPackDetectionModel model, Uint64Vector & consensus);

  float ComputeDeformationScore(const float image_width, const float image_height,
                                const RollPackDetectionModel model);

  RansacFindHomographyResult RansacFindDeformModel(const cv::Mat & reference_image, const cv::Mat & image,
                                                   const std::vector<cv::KeyPoint> & keypoint_reference,
                                                   const std::vector<cv::KeyPoint> & keypoint_image,
                                                   const std::vector<cv::DMatch> & matches,
                                                   const FloatVector & matches_score,
                                                   const RansacFindHomographyResult & initial_guess,
                                                   const float best_score_in);

  RansacFindHomographyResult RansacFindHomography(const cv::Mat & image,
                                                  const cv::Mat & image_mask,
                                                  const std::vector<cv::KeyPoint> & keypoint_image,
                                                  ReferenceImageFeaturesVector & reference_image_features_vector);

  void PrintModel(const RollPackDetectionModel & model);

  void PrintModel(const RansacFindHomographyResult & rfhr);

  PointXYZRGBCloud ImageToCloud(const cv::Mat & rgb_image, const cv::Mat & depth_image,
                                const Intrinsics & intrinsics, const Eigen::Affine3f & camera_pose);

  DetectedPackVector FindMultipleObjects(const cv::Mat & image, const cv::Mat & depth_image_in,
                                         const ReferenceImageVector & reference_images,
                                         const Intrinsics & intrinsics,
                                         const Eigen::Affine3f & camera_pose_in);

  RansacFindHomographyResult FindObjectPose(const cv::Mat & image_grayscale, const cv::Mat & depth_image,
                                            const cv::Mat & reference_image,
                                            const Eigen::Affine3f & camera_pose,
                                            const Intrinsics & intrinsics,
                                            const RansacFindHomographyResult & rfhr,
                                            const ReferencePoints & reference_points);

  RansacFindHomographyResult FindObject(const cv::Mat & image_grayscale, const cv::Mat & depth_image,
                                        const cv::Mat & image_mask,
                                        const std::vector<cv::KeyPoint> & keypoints_image,
                                        const cv::Mat & descriptors_image,
                                        const BoolVector & valid_keypoints_image,
                                        ReferenceImageFeaturesVector & reference_image_features_vector,
                                        const Intrinsics & intrinsics,
                                        const Eigen::Affine3f & camera_pose);

  float ComputeMatchSpreadScore(const float min_match_element, const float max_match_element) const;

  void ExtractFeaturesOrientationInvariant(int nfeatures,
                                           const cv::Mat & grayscale_image,
                                           const cv::Mat & image_mask,
                                           std::vector<cv::KeyPoint> & keypoints,
                                           cv::Mat & descriptors);

  private:
  std::string m_reference_image_points_filename;

  uint64 m_random_seed;
  std::shared_ptr<std::mt19937> m_random_generator;

  LogFunction m_log;
  PublishImageFunction m_publish_image;
  PublishCloudFunction m_publish_cloud;

  Config m_config;
};

#endif // ROLL_PACK_DETECTION_H
