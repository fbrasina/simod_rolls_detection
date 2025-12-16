// OpenCV
#include <opencv2/opencv.hpp>
#ifdef EXTRA_SIFT_FILES
  #include "../sift/sift.h"
#endif // EXTRA_SIFT_FILES

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
#include "roll_pack_detection.h"

inline cv::Mat GrayscaleToColor(const cv::Mat & img)
{
  cv::Mat result;
  cv::cvtColor(img, result, cv::COLOR_GRAY2RGB);
  return result;
}

inline cv::Mat ColorToGrayscale(const cv::Mat & img)
{
  cv::Mat result;
  cv::cvtColor(img, result, cv::COLOR_BGR2GRAY);
  return result;
}

PackDetection::PackDetection(const Config & config,
              const LogFunction & log_function,
              const PublishImageFunction & publish_image_function,
              const PublishCloudFunction & cloud_function,
              const uint64 random_seed)
{
  m_config = config;
  m_log = log_function;
  m_publish_image = publish_image_function;
  m_publish_cloud = cloud_function;

  std::string param_str;

  m_random_seed = random_seed;
  m_random_generator.reset(new std::mt19937(m_random_seed));
}

PackDetection::StringVector PackDetection::GetImagePublisherNames()
{
  StringVector result;
  result.push_back("image_mask");
  result.push_back("matches");
  return result;
}

PackDetection::StringVector PackDetection::GetCloudPublisherNames()
{
  StringVector result;
  result.push_back("input");
  return result;
}

PackDetection::ReferencePointsPtr PackDetection::LoadReferencePoints(const std::string & filename, LogFunction & log)
{
  std::ifstream ifile(filename);
  if (!ifile)
  {
    log(3, "Could not load file \"" + filename + "\"");
    return ReferencePointsPtr();
  }

  ReferencePointsPtr result(new ReferencePoints);

  std::string line;
  while (std::getline(ifile, line))
  {
    if (line.empty() || line[0] == '#')
      continue;
    std::istringstream linestr(line);
    std::string cmd;
    linestr >> cmd;

    if (cmd == "element_center")
    {
      cv::Point2f c;
      linestr >> c.x >> c.y;
      if (!linestr)
      {
        log(3, "Could not parse element center at line \"" + line + "\"");
        return ReferencePointsPtr();
      }
      result->element_centers.push_back(c);
    }
    else if (cmd == "box_size_whd")
    {
      linestr >> result->box_width >> result->box_height >> result->box_depth;
      if (!linestr)
      {
        log(3, "Could not parse box size at line \"" + line + "\"");
        return ReferencePointsPtr();
      }
    }
    else if (cmd == "pixel_size_in_meters")
    {
      linestr >> result->pixel_size_in_meters;
      if (!linestr)
      {
        log(3, "Could not parse pixel_size_in_meters at line \"" + line + "\"");
        return ReferencePointsPtr();
      }
    }
    else if (cmd == "p")
    {
      std::string type;
      cv::Point2f ci, cr;
      linestr >> type >> ci.x >> ci.y >> cr.x >> cr.y;
      if (!linestr)
      {
        log(3, "Could not parse point at line \"" + line + "\"");
        return ReferencePointsPtr();
      }
      result->pts[type] = ReferencePoint(ci, cr);
    }
    else
    {
      log(2, "Unknown command at line \"" + line + "\"");
    }
  }

  return result;
}

PackDetection::HomographyArray PackDetection::CVHomographyToArray(const cv::Mat & h)
{
  HomographyArray ih;
  for (uint64 y = 0; y < 3; y++)
  {
    for (uint64 x = 0; x < 3; x++)
    {
      if (x == 2 && y == 2)
        continue;

      ih[x + y * 3] = h.at<double>(y, x);
    }
  }
  return ih;
}

PackDetection::RollPackDetectionModelPtr PackDetection::EstimateModelMinSq(const double reference_size_x, const double reference_size_y,
                                             const Vector2dVector & reference_points,
                                             const Vector2dVector & observed_points,
                                             const double cost_x, const double cost_y)
{
  RollPackDetectionModelPtr result(new RollPackDetectionModel);

  IntVector element_n(reference_points.size());
  for (uint64 i = 0; i < reference_points.size(); i++)
    element_n[i] = result->PointToElementN(reference_size_x, reference_size_y,
                                           reference_points[i].x(), reference_points[i].y());

  const int COUNT = ESTIMATOR_COUNT;

  const int HOMOGRAPHY_SIZE = 8;
  const int row_length = HOMOGRAPHY_SIZE + (COUNT - 1) * 2;
  // -x -y -1 0 0 0 ux uy
  // 1 0 0 0 0 0 0 0 0 0 0 0      1 0 0 0 0 0 0 0 0 0 0 0

  Eigen::MatrixXd rows = Eigen::MatrixXd::Zero(reference_points.size() * 2 + (COUNT - 1) * 2, row_length);
  Eigen::VectorXd knowns = Eigen::VectorXd::Zero(reference_points.size() * 2 + (COUNT - 1) * 2);

  const double pt_weight = 1.0 / reference_points.size();

  for (uint64 i = 0; i < reference_points.size(); i++)
  {
    #define rowx (rows.row(i * 2 + 0))
    #define rowy (rows.row(i * 2 + 1))

    rowx[0] = -observed_points[i].x();
    rowx[1] = -observed_points[i].y();
    rowx[2] = -1.0f;
    rowx[6] = reference_points[i].x() * observed_points[i].x();
    rowx[7] = reference_points[i].x() * observed_points[i].y();

    for (int h = element_n[i]; h < 0; h++)
      rowx[HOMOGRAPHY_SIZE + h + ((COUNT - 1) / 2)] = 1.0f;
    for (int h = 1; h <= element_n[i]; h++)
      rowx[HOMOGRAPHY_SIZE + h + ((COUNT - 1) / 2) - 1] = 1.0f;

    rowy[3] = -observed_points[i].x();
    rowy[4] = -observed_points[i].y();
    rowy[5] = -1.0;
    rowy[6] = reference_points[i].y() * observed_points[i].x();
    rowy[7] = reference_points[i].y() * observed_points[i].y();

    for (int h = element_n[i]; h < 0; h++)
      rowy[HOMOGRAPHY_SIZE + h + (COUNT - 1) + ((COUNT - 1) / 2)] = 1.0f;
    for (int h = 1; h <= element_n[i]; h++)
      rowy[HOMOGRAPHY_SIZE + h + (COUNT - 1) + ((COUNT - 1) / 2) - 1] = 1.0f;

    rowx = rowx * pt_weight;
    knowns[i * 2 + 0] = -reference_points[i].x() * pt_weight;
    rowy = rowy * pt_weight;
    knowns[i * 2 + 1] = -reference_points[i].y() * pt_weight;

    #undef rowy
    #undef rowx
  }

  // costs
  for (uint64 i = 0; i < (COUNT - 1); i++)
  {
    rows.row(reference_points.size() * 2 + i * 2 + 0)[HOMOGRAPHY_SIZE + i] = cost_x / (COUNT - 1);
    // knowns[reference_points.size() * 2 + i * 2 + 0] = 0.0;
    rows.row(reference_points.size() * 2 + i * 2 + 1)[HOMOGRAPHY_SIZE + (COUNT - 1) + i] = cost_y / (COUNT - 1);
    // knowns[reference_points.size() * 2 + i * 2 + 1] = 0.0;
  }

  Eigen::VectorXd s = rows.colPivHouseholderQr().solve(knowns);

  Eigen::Matrix3f homography;
  for (uint64 i = 0; i < HOMOGRAPHY_SIZE; i++)
    homography(i / 3, i % 3) = s[i];
  homography(2, 2) = 1.0f;
  {
    Eigen::Matrix3f temp_h = homography;
    homography = temp_h.inverse();
    homography = homography / homography(2, 2);
  }

  for (uint64 i = 0; i < HOMOGRAPHY_SIZE; i++)
    result->homography[i] = homography(i / 3, i % 3);

  for (uint64 i = 0; i < (COUNT - 1); i++)
    result->translations_x[i] = s[HOMOGRAPHY_SIZE + i];
  for (uint64 i = 0; i < (COUNT - 1); i++)
    result->translations_y[i] = s[HOMOGRAPHY_SIZE + (COUNT - 1) + i];

  return result;
}

float PackDetection::ComputeDeformationScore(const float image_width, const float image_height,
                              const RollPackDetectionModel model)
{
  // if width is increased:
  const double max_acceptable_deformation_x =  (double(image_width) / ESTIMATOR_COUNT) / 2.0;
  // if width is decreased:
  const double min_acceptable_deformation_x =  (double(image_width) / ESTIMATOR_COUNT) / 4.0;
  // any change in y:
  const double max_acceptable_deformation_y =  double(image_height) / 6.0;

  float min_score = 1.0f;
  for (int i = 0; i < ESTIMATOR_COUNT - 1; i++)
  {
    float smx = 1.0f;
    // (i >= (ESTIMATOR_COUNT - 1) / 2): in the left side, negative increases width and positive decreases
    if ((i >= (ESTIMATOR_COUNT - 1) / 2) == (model.translations_x[i] > 0.0))
      smx = std::max(0.0, 1.0 - SQR(model.translations_x[i] / max_acceptable_deformation_x));
    if ((i >= (ESTIMATOR_COUNT - 1) / 2) == (model.translations_x[i] < 0.0))
      smx = std::max(0.0, 1.0 - SQR(model.translations_x[i] / min_acceptable_deformation_x));
    min_score = std::min(min_score, smx);

    float smy = 1.0f;
    smy = std::max(0.0, 1.0 - std::abs(model.translations_y[i] / max_acceptable_deformation_y));
    min_score = std::min(min_score, smy);
  }

  return min_score;
}

float PackDetection::FindConsensus(const double image_width, const double image_height,
                                   const Vector2dVector & reference_points, const Vector2dVector & observed_points,
                                   const FloatVector & cons_matches_score,
                                   double reproj_threshold,
                                   const RollPackDetectionModel model, Uint64Vector & consensus)
{
  float score = 0.0f;
  int min_match_element = 0, max_match_element = 0;
  consensus.clear();

  consensus.reserve(reference_points.size());
  for (uint64 i = 0; i < reference_points.size(); i++)
  {
    const int element_n = model.PointToElementN(image_width, image_height, reference_points[i].x(), reference_points[i].y());
    double nx, ny;
    model.ApplyToPoint(element_n, reference_points[i].x(), reference_points[i].y(), nx, ny);
    const Eigen::Vector2d pred_pt(nx, ny);

    if ((observed_points[i] - pred_pt).norm() < reproj_threshold)
    {
      score += cons_matches_score[i];

      if (element_n < min_match_element)
        min_match_element = element_n;
      if (element_n > max_match_element)
        max_match_element = element_n;

      consensus.push_back(i);
    }
  }
  score *= ComputeMatchSpreadScore(min_match_element, max_match_element);

  return score;
}

float PackDetection::ComputeMatchSpreadScore(const float min_match_element, const float max_match_element) const
{
  const float match_spread_score_max_diff = m_config.match_spread_score_max_diff;
  const float match_spread_score = float(max_match_element - min_match_element + 1 -
                                         (ESTIMATOR_COUNT - match_spread_score_max_diff)) / match_spread_score_max_diff;
  return match_spread_score;
}

PackDetection::RansacFindHomographyResult PackDetection::RansacFindDeformModel(const cv::Mat & reference_image, const cv::Mat & image,
                                                 const std::vector<cv::KeyPoint> & keypoint_reference,
                                                 const std::vector<cv::KeyPoint> & keypoint_image,
                                                 const std::vector<cv::DMatch> & matches,
                                                 const FloatVector & matches_score,
                                                 const RansacFindHomographyResult & initial_guess,
                                                 const float best_score_in)
{
  const uint64 ITERATIONS = m_config.ransac_refine_iterations;
  const double REPROJ_CLOSED_FORM_THRESHOLD = m_config.reproj_closed_form_threshold_px;
  const double REPROJ_REFINE_THRESHOLD = m_config.reproj_refine_threshold_px;
  const Uint64Vector & initial_consensus = initial_guess.consensus;
  const uint64 INITIAL_MATCHES = initial_consensus.size() * 9 / 10;

  RansacFindHomographyResult result = initial_guess;

  std::mt19937 & random_generator = *m_random_generator;

  Vector2dVector reference_points(initial_consensus.size());
  Vector2dVector observed_points(initial_consensus.size());
  FloatVector cons_matches_score(initial_consensus.size());
  for (uint64 i = 0; i < initial_consensus.size(); i++)
  {
    cv::Point2f rpt = keypoint_reference[matches[initial_consensus[i]].queryIdx].pt;
    reference_points[i] = Eigen::Vector2d(rpt.x, rpt.y);
    cv::Point2f opt = keypoint_image[matches[initial_consensus[i]].trainIdx].pt;
    observed_points[i] = Eigen::Vector2d(opt.x, opt.y);
    cons_matches_score[i] = matches_score[initial_consensus[i]];
  }

  const double max_error_for_huber_loss = m_config.max_error_for_huber_loss;
  const Eigen::Vector2d translation_px_weight(m_config.translation_px_weight_x, m_config.translation_px_weight_y);
  MyRollPackDetectionEstimator estimator(reference_image.cols, reference_image.rows,
                                         max_error_for_huber_loss, translation_px_weight);

  std::shared_ptr<RollPackDetectionModel> best_model_ptr;
  float best_score = best_score_in;
  Uint64Vector best_consensus;

  for (uint64 iter = 0; iter < ITERATIONS; iter++)
  {
    Uint64Vector consensus_shuffle(initial_consensus.size());
    for (uint64 i = 0; i < initial_consensus.size(); i++)
      consensus_shuffle[i] = i;
    for (uint64 i = 0; i < INITIAL_MATCHES; i++)
    {
      std::uniform_int_distribution<uint64> distrib(i, initial_consensus.size() - 1);
      const uint64 sel = distrib(random_generator);
      std::swap(consensus_shuffle[i], consensus_shuffle[sel]);
    }

    Vector2dVector initial_reference_points(INITIAL_MATCHES);
    Vector2dVector initial_observed_points(INITIAL_MATCHES);
    for (uint64 i = 0; i < INITIAL_MATCHES; i++)
    {
      const uint64 idx = consensus_shuffle[i];
      initial_reference_points[i] = reference_points[idx];
      initial_observed_points[i] = observed_points[idx];
    }

    RollPackDetectionModelPtr closed_model_ptr = EstimateModelMinSq(reference_image.cols, reference_image.rows,
                                                                    initial_reference_points, initial_observed_points,
                                                                    m_config.translation_px_weight_x, m_config.translation_px_weight_y);
    if (!closed_model_ptr)
    {
      continue;
    }

    float score_approx;
    {
      Uint64Vector consensus;
      score_approx = FindConsensus(reference_image.cols, reference_image.rows, reference_points,
                                   observed_points, cons_matches_score, REPROJ_CLOSED_FORM_THRESHOLD, *closed_model_ptr, consensus);
    }
    score_approx *= ComputeDeformationScore(reference_image.cols, reference_image.rows, *closed_model_ptr);

    if (score_approx <= best_score)
    {
      continue;
    }

    // const HomographyArray ih = CVHomographyToArray(result.initial_homography);

    // estimate model
    double model_final_cost;
    RollPackDetectionModelPtr model_ptr = estimator.Estimate(*closed_model_ptr, initial_reference_points,
                                                             initial_observed_points, model_final_cost);
    if (!model_ptr || std::isnan(model_final_cost))
    {
      continue;
    }

    // find consensus
    float score = 0.0f;
    Uint64Vector consensus;
    {
      score = FindConsensus(reference_image.cols, reference_image.rows, reference_points,
                            observed_points, cons_matches_score, REPROJ_REFINE_THRESHOLD, *model_ptr, consensus);
    }
    // check that deformation is acceptable
    score *= ComputeDeformationScore(reference_image.cols, reference_image.rows, *model_ptr);

    if (score > best_score)
    {
      best_model_ptr = model_ptr;
      best_consensus = consensus;
      best_score = score;
    }
  }

  // final model refinement
  if (best_model_ptr)
  {
    Vector2dVector d_reference_points(best_consensus.size());
    Vector2dVector d_observed_points(best_consensus.size());
    for (uint64 i = 0; i < best_consensus.size(); i++)
    {
      d_reference_points[i] = reference_points[best_consensus[i]];
      d_observed_points[i] = observed_points[best_consensus[i]];
    }

    // estimate refined model
    double model_final_cost;
    RollPackDetectionModelPtr model_ptr = estimator.Estimate(*best_model_ptr, d_reference_points,
                                                             d_observed_points, model_final_cost);
    if (!model_ptr || std::isnan(model_final_cost))
    {
      return result;
    }

    result.detection_model = *model_ptr;
    result.detection_model_score = best_score;
    result.detection_model_final_cost = model_final_cost;

    for (uint64 idx : best_consensus)
      result.detection_model_consensus.push_back(initial_consensus[idx]);
  }

  return result;
}

inline Eigen::Matrix3d OpenCVMat3x3ToEigen(const cv::Mat & m)
{
  Eigen::Matrix3d result;
  for (int y = 0; y < 3; y++)
    for (int x = 0; x < 3; x++)
      result(y, x) = m.at<double>(y, x);

  return result;
}

PackDetection::RansacFindHomographyResult PackDetection::RansacFindHomography(const cv::Mat & image,
                                                const cv::Mat & image_mask,
                                                const std::vector<cv::KeyPoint> & keypoint_image,
                                                ReferenceImageFeaturesVector & reference_image_features_vector)
{
  const uint64 ITERATIONS = m_config.ransac_iterations;
  const float REPROJ_THRESHOLD = m_config.reproj_threshold_px;
  const uint64 INITIAL_MATCHES = 4;
  const float MAX_SCALING = m_config.max_scaling;
  const float MAX_NON_UNIFORM_SCALING = m_config.max_non_uniform_scaling;

  Point2fVector points_image;
  for (uint64 i = 0; i < keypoint_image.size(); i++)
    points_image.push_back(keypoint_image[i].pt);

  Point2fVector initial_matches_obj(INITIAL_MATCHES);
  Point2fVector initial_matches_scene(INITIAL_MATCHES);

  std::mt19937 & random_generator = *m_random_generator;
  std::uniform_int_distribution<uint64> ref_distrib(0, reference_image_features_vector.size() - 1);

  uint64 succeeded_counter = 0;
  uint64 compatible_counter = 0;

  const double max_error_for_huber_loss = m_config.max_error_for_huber_loss;
  const Eigen::Vector2d translation_px_weight(m_config.translation_px_weight_x, m_config.translation_px_weight_y);

  m_log(1, "Running RANSAC for " + std::to_string(ITERATIONS) + "*" +
           std::to_string(reference_image_features_vector.size()) + " iterations.");

  RansacFindHomographyResult result;
  result.score = 0.0f;
  for (uint64 iter = 0; iter < ITERATIONS; iter++)
  {
    const uint64 reference_id = ref_distrib(random_generator);
    const std::vector<cv::DMatch> & matches = reference_image_features_vector[reference_id].matches;
    const FloatVector & matches_score = reference_image_features_vector[reference_id].matches_score;
    const BoolVectorVector & matches_compatibility = reference_image_features_vector[reference_id].matches_compatibility;
    const Uint64VectorVector & matches_comp_graph = reference_image_features_vector[reference_id].matches_comp_graph;
    const FloatVectorVector & matches_scale = reference_image_features_vector[reference_id].matches_scale;
    const std::vector<cv::KeyPoint> & keypoint_reference = reference_image_features_vector[reference_id].keypoints_reference;
    const cv::Mat & reference_grayscale = reference_image_features_vector[reference_id].reference_grayscale;

    MyRollPackDetectionEstimator estimator(reference_grayscale.cols, reference_grayscale.rows,
                                           max_error_for_huber_loss, translation_px_weight);

    // select INITIAL_MATCHES random matches
    Uint64Vector initial_matches;
    initial_matches.reserve(INITIAL_MATCHES);

    {
      Uint64Vector compatible_matches;
      for (uint64 i = 0; i < INITIAL_MATCHES; i++)
      {
        uint64 sel;

        if (i == 0)
        {
          std::uniform_int_distribution<uint64> distrib(0, matches.size() - 1);
          sel = distrib(random_generator);
        }
        else
        {
          if (compatible_matches.empty())
            break;

          std::uniform_int_distribution<uint64> distrib(0, compatible_matches.size() - 1);
          sel = compatible_matches[distrib(random_generator)];
        }

        if (i >= 2) // check compatibility with first two matches
        {
          const float s0 = matches_scale[initial_matches[0]][sel];
          const float s1 = matches_scale[initial_matches[0]][initial_matches[1]];
          const float s2 = matches_scale[initial_matches[1]][sel];
          if (s0 > s1 * m_config.max_non_uniform_scaling)
            break;
          if (s1 > s0 * m_config.max_non_uniform_scaling)
            break;
          if (s2 > s1 * m_config.max_non_uniform_scaling)
            break;
          if (s1 > s2 * m_config.max_non_uniform_scaling)
            break;
          if (s0 > s2 * m_config.max_non_uniform_scaling)
            break;
          if (s2 > s0 * m_config.max_non_uniform_scaling)
            break;
        }

        if (i == 0)
          compatible_matches = matches_comp_graph[sel];
        else
        {
          if (i + 1 < INITIAL_MATCHES) // update compatible_matches only if it is not the last iteration
          {
            Uint64Vector new_cmp(compatible_matches.size());
            Uint64Vector::iterator end_iter = std::set_intersection(compatible_matches.begin(),
                                                                    compatible_matches.end(),
                                                                    matches_comp_graph[sel].begin(),
                                                                    matches_comp_graph[sel].end(),
                                                                    new_cmp.begin());
            new_cmp.resize(end_iter - new_cmp.begin());
            compatible_matches = new_cmp;
          }
        }

        // insertion sort to keep ordered
        for (uint64 h = 0; h < i; h++)
        {
          if (sel < initial_matches[h])
          {
            std::swap(initial_matches[h], sel);
          }
        }

        initial_matches.push_back(sel);
      }

      if (initial_matches.size() < INITIAL_MATCHES)
        continue;
    }

    // for (uint64 i : initial_matches)
    //   std::cout << i << " ";
    // std::cout << std::endl;

    // check compatibility
    // {
    //   bool incomp = false;
    //   for (uint64 i = 0; i < INITIAL_MATCHES; i++)
    //     for (uint64 h = i + 1; h < INITIAL_MATCHES; h++)
    //       if (!matches_compatibility[initial_matches[i]][initial_matches[h]])
    //         incomp = true;
    //   if (incomp)
    //     continue;
    // }

    compatible_counter++;

    // find initial homography
    cv::Mat initial_h;
    {
      for (uint64 i = 0; i < INITIAL_MATCHES; i++)
      {
        initial_matches_obj[i] = keypoint_reference[matches[initial_matches[i]].queryIdx].pt;
        initial_matches_scene[i] = keypoint_image[matches[initial_matches[i]].trainIdx].pt;
      }

      initial_h = cv::getPerspectiveTransform(initial_matches_obj, initial_matches_scene);

      // check that reference borders are still within the image
      // objects partially in the image will be discarded here
      {
        const Eigen::Matrix3d he = OpenCVMat3x3ToEigen(initial_h);
        const Eigen::Vector3d pts_to_check[4] = {Eigen::Vector3d(0.0, 0.0, 1.0),
                                                 Eigen::Vector3d(reference_grayscale.cols, 0.0, 1.0),
                                                 Eigen::Vector3d(0.0, reference_grayscale.rows, 1.0),
                                                 Eigen::Vector3d(reference_grayscale.cols, reference_grayscale.rows, 1.0)};

        bool valid = true;
        for (uint64 i = 0; i < 4 && valid; i++)
        {
          Eigen::Vector3d p = he * pts_to_check[i];
          if (p.z() < 0.001)
            valid = false;
          p = p / p.z();

          if (p.x() < 0.0f || p.x() >= image.cols || p.y() < 0.0f || p.y() >= image.rows)
            valid = false;
        }
        if (!valid)
          continue;

        constexpr uint64 COUNT = ESTIMATOR_COUNT;
        Eigen::Vector3d internal_pts_to_check[COUNT];
        for (uint64 i = 0; i < COUNT; i++)
        {
          internal_pts_to_check[i] = Eigen::Vector3d((i * 0.5) * double(reference_grayscale.cols) / COUNT,
                                                     reference_grayscale.rows / 2.0, 1.0);
        }

        // check that all internal points are within image mask
        for (uint64 i = 0; i < COUNT && valid; i++)
        {
          Eigen::Vector3d p = he * internal_pts_to_check[i];
          if (p.z() < 0.001)
            valid = false;
          p = p / p.z();
          const int rx = int(std::round(p.x()));
          const int ry = int(std::round(p.y()));

          if (rx < 0.0f || rx >= image.cols || ry < 0.0f || ry >= image.rows)
            valid = false;
          if (!valid)
            continue;

          if (!image_mask.at<uint8>(ry, rx))
            valid = false;
        }
        if (!valid)
          continue;
      }

      // check no reflection or collapse occurred
      const double a = initial_h.at<double>(0, 0);
      const double b = initial_h.at<double>(0, 1);
      const double c = initial_h.at<double>(1, 0);
      const double d = initial_h.at<double>(1, 1);
      const double det = a * d - b * c;
      if (det < 0.01)
        continue;

      Eigen::Matrix2f m;
      m(0, 0) = a;
      m(0, 1) = b;
      m(1, 0) = c;
      m(1, 1) = d;
      Eigen::JacobiSVD<Eigen::Matrix2f> svd(m);
      Eigen::Vector2f singular_values = svd.singularValues();
      // constrain scaling in any direction
      if (std::abs(singular_values.x()) < 1.0f/MAX_SCALING || std::abs(singular_values.x()) > MAX_SCALING)
        continue;
      if (std::abs(singular_values.y()) < 1.0f/MAX_SCALING || std::abs(singular_values.y()) > MAX_SCALING)
        continue;
      if (std::abs(singular_values.y()) / std::abs(singular_values.x()) > MAX_NON_UNIFORM_SCALING)
        continue;
      if (std::abs(singular_values.x()) / std::abs(singular_values.y()) > MAX_NON_UNIFORM_SCALING)
        continue;
    }

    succeeded_counter++;

    // check consensus
    Uint64Vector consensus;
    float score = 0.0f;
    {
      Point2fVector points_image_reference;
      cv::perspectiveTransform(points_image, points_image_reference, initial_h.inv());

      int min_match_element = 0, max_match_element = 0;
      for (uint64 i = 0; i < matches.size(); i++)
      {
        const cv::Point2f ptri = points_image_reference[matches[i].trainIdx];
        const cv::Point2f ptr = keypoint_reference[matches[i].queryIdx].pt;
        if (cv::norm(ptri - ptr) < REPROJ_THRESHOLD)
        {
          bool incomp = false;
          for (const uint64 c : consensus)
            if (!matches_compatibility[c][i]) // check compatibility with prev consensus
              incomp = true;
          if (incomp)
            continue;

          consensus.push_back(i);
          score += matches_score[i];

          const int element_n = estimator.PointToElementN(ptr.x, ptr.y);
          if (element_n < min_match_element)
            min_match_element = element_n;
          if (element_n > max_match_element)
            max_match_element = element_n;
        }
      }

      score *= ComputeMatchSpreadScore(min_match_element, max_match_element);
    }

    if (score < m_config.min_object_score)
      continue;
    if (score < result.detection_model_score)
      continue; // score already lower than best detection

    RansacFindHomographyResult rhfr;
    rhfr.score = score;
    rhfr.consensus = consensus;
    rhfr.initial_homography = initial_h;
    rhfr.homography = result.initial_homography.clone();
    rhfr.reference_id = reference_id;

    // refine homography with new consensus
    {
      Point2fVector obj;
      Point2fVector scene;
      obj.reserve(rhfr.consensus.size());
      scene.reserve(rhfr.consensus.size());
      for (const uint64 i : rhfr.consensus)
      {
        obj.push_back(keypoint_reference[matches[i].queryIdx].pt);
        scene.push_back(keypoint_image[matches[i].trainIdx].pt);
      }

      if (rhfr.consensus.size() >= 4)
        rhfr.homography = cv::findHomography(obj, scene);
    }

    rhfr = RansacFindDeformModel(reference_grayscale, image,
                                 keypoint_reference, keypoint_image,
                                 matches, matches_score, rhfr, result.detection_model_score);

    if (rhfr.detection_model_score >= result.detection_model_score)
    {
      result = rhfr;
    }
  }

  m_log(1, "RANSAC end.");
  m_log(1, "Compatible " + std::to_string(compatible_counter) + " Succeeded " +
             std::to_string(succeeded_counter) + " failed " + std::to_string(ITERATIONS - succeeded_counter));

  return result;
}

void PackDetection::PrintModel(const RollPackDetectionModel & model)
{
  std::ostringstream model_str;
  for (uint64 y = 0; y < 3; y++)
  {
    for (uint64 x = 0; x < 3; x++)
    {
      if (x == 2 && y == 2)
      {
        model_str << "1.0" << "\t";
        continue;
      }

      model_str << model.homography[x + y * 3] << "\t";
    }
    model_str << "\n";
  }
  model_str << "Deformation x: ";
  for (uint64 x = 0; x < 6; x++)
  {
    if (x == 3)
      model_str << 0 << " ";

    model_str << model.translations_x[x] << " ";
  }
  model_str << "\nDeformation y: ";
  for (uint64 x = 0; x < 6; x++)
  {
    if (x == 3)
      model_str << 0 << " ";

    model_str << model.translations_y[x] << " ";
  }
  m_log(1, "Model:\n" + model_str.str());
}

void PackDetection::PrintModel(const RansacFindHomographyResult & rfhr)
{
  m_log(1, "Object type (Reference image id): " + std::to_string(rfhr.reference_id));
  m_log(1, "Is upside down: " + std::string(rfhr.is_upside_down ? "YES" : "NO"));

  {
    std::ostringstream hstr;
    hstr << rfhr.homography;
    m_log(1, "H:\n" + hstr.str());
  }

  PrintModel(rfhr.detection_model);
}

PackDetection::PointXYZRGBCloud PackDetection::ImageToCloud(const cv::Mat & rgb_image, const cv::Mat & depth_image,
                              const Intrinsics & intrinsics, const Eigen::Affine3f & camera_pose)
{
  const int width = rgb_image.cols;
  const int height = rgb_image.rows;

  PointXYZRGBCloud result;
  for (int y = 0; y < height; y++)
    for (int x = 0; x < width; x++)
    {
      pcl::PointXYZRGB pt;
      pt.r = rgb_image.at<cv::Vec3b>(y, x)[2];
      pt.g = rgb_image.at<cv::Vec3b>(y, x)[1];
      pt.b = rgb_image.at<cv::Vec3b>(y, x)[0];
      pt.z = depth_image.at<uint16>(y, x) / 1000.0f;
      pt.x = (x - intrinsics.cx + 0.5f) / intrinsics.fx * pt.z;
      pt.y = (y - intrinsics.cy + 0.5f) / intrinsics.fy * pt.z;

      result.push_back(pt);
    }

  pcl::transformPointCloud(result, result, camera_pose);

  return result;
}

void PackDetection::ExtractFeaturesOrientationInvariant(int nfeatures,
                                                        const cv::Mat & grayscale_image,
                                                        const cv::Mat & image_mask,
                                                        std::vector<cv::KeyPoint> & keypoints,
                                                        cv::Mat & descriptors)
{
  cv::Ptr<cv::SIFT> detector = cv::SIFT::create(nfeatures);
  detector->detectAndCompute(grayscale_image, image_mask, keypoints, descriptors);

  // rotate 180 degrees and repeat detection
  cv::Point2f pc(grayscale_image.cols/2.0f - 0.5f, grayscale_image.rows/2.0f - 0.5f);
  cv::Mat r = cv::getRotationMatrix2D(pc, 180, 1.0);
  cv::Mat rotated_image;
  cv::Mat rotated_mask;

  cv::warpAffine(grayscale_image, rotated_image, r, grayscale_image.size());
  cv::warpAffine(image_mask, rotated_mask, r, image_mask.size());

  std::vector<cv::KeyPoint> rotated_keypoints_reference;
  cv::Mat rotated_descriptors_reference;
  detector->detectAndCompute(rotated_image, rotated_mask,
                             rotated_keypoints_reference, rotated_descriptors_reference);

  // rotate keypoints back
  for (cv::KeyPoint & kp : rotated_keypoints_reference)
  {
    kp.pt.x = grayscale_image.cols - kp.pt.x - 0.5f;
    kp.pt.y = grayscale_image.rows - kp.pt.y - 0.5f;
    kp.angle = kp.angle + 180.0f;
    if (kp.angle > 360.0f)
      kp.angle -= 360.0f;
  }

  // initialize a flann KDTree for faster search
  cv::Mat rkp_mat(rotated_keypoints_reference.size(), 2, CV_32FC1);
  for (uint64 rkri = 0; rkri < rotated_keypoints_reference.size(); rkri++)
  {
    rkp_mat.at<float>(rkri, 0) = rotated_keypoints_reference[rkri].pt.x;
    rkp_mat.at<float>(rkri, 1) = rotated_keypoints_reference[rkri].pt.y;
  }

  cv::flann::KDTreeIndexParams index_params(1);
  cv::flann::Index kd_tree(rkp_mat, index_params);

  // average the descriptors
  IntVector indices;
  FloatVector dists;
  FloatVector kp_mat(2);
  for (uint64 kri = 0; kri < keypoints.size(); kri++)
  {
    const cv::KeyPoint & kp = keypoints[kri];

    kp_mat[0] = kp.pt.x;
    kp_mat[1] = kp.pt.y;

    kd_tree.knnSearch(kp_mat, indices, dists, 1);
    if (indices.empty())
      continue;

    const uint64 rkri = indices[0];
    const cv::KeyPoint & rkp = rotated_keypoints_reference[rkri];
    if (cv::norm(kp.pt - rkp.pt) <= 1.0f)
    {
      descriptors.row(kri) = (descriptors.row(kri) + rotated_descriptors_reference.row(rkri)) / 2.0f;
    }
  }
}

PackDetection::DetectedPackVector PackDetection::FindMultipleObjects(const cv::Mat & image, const cv::Mat & depth_image_in,
                                       const ReferenceImageVector & reference_images,
                                       const Intrinsics & intrinsics,
                                       const Eigen::Affine3f & camera_pose_in)
{
  m_log(1, "FindMultipleObjects started.");

  Eigen::Affine3f camera_pose = camera_pose_in;
  cv::Mat depth_image;
  if (!depth_image_in.empty())
    depth_image = depth_image_in.clone();
  else
  {
    depth_image = cv::Mat(image.rows, image.cols, CV_16UC1);
    depth_image.setTo(uint16(0)); // all invalid
  }

  cv::Mat image_grayscale = ColorToGrayscale(image);
  cv::Mat image_mask = image_grayscale.clone();
  image_mask.setTo(255);

  {
    for (int y = 0; y < depth_image.rows; y++)
      for (int x = 0; x < depth_image.cols; x++)
        if (depth_image.at<uint16>(y, x) && ((depth_image.at<uint16>(y, x) / 1000.0f) > m_config.max_valid_depth))
          image_mask.at<uint8>(y, x) = 0;

    const int dilation_size = 5;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                cv::Size(2 * dilation_size + 1, 2 * dilation_size+1));
    cv::dilate(image_mask, image_mask, element);
  }

  {
    const PointXYZRGBCloud cloud = ImageToCloud(image, depth_image, intrinsics, camera_pose);
    m_publish_cloud("input", cloud);
  }

  if (m_config.reverse_image)
  {
    cv::Mat rotated;
    cv::Point2f pc(image_grayscale.cols/2.0f, image_grayscale.rows/2.0f);
    cv::Mat r = cv::getRotationMatrix2D(pc, 180, 1.0);

    cv::warpAffine(image_grayscale, rotated, r, image_grayscale.size());
    image_grayscale = rotated.clone();

    cv::warpAffine(image_mask, rotated, r, image_mask.size());
    image_mask = rotated.clone();

    cv::warpAffine(depth_image, rotated, r, depth_image.size());
    depth_image = rotated.clone();

    camera_pose = camera_pose * Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ());
  }

  m_publish_image("image_mask", image_mask, "mono8");

  const int nfeatures_image = m_config.nfeatures_image;
  const int nfeatures_reference = m_config.nfeatures_reference;

  m_log(1, "Finding features.");

  std::vector<cv::KeyPoint> keypoints_image;
  cv::Mat descriptors_image;
  ExtractFeaturesOrientationInvariant(nfeatures_image, image_grayscale, image_mask,
                                      keypoints_image, descriptors_image);
  m_log(1, "Initial keypoints image size: " + std::to_string(keypoints_image.size()));

  ReferenceImageFeaturesVector reference_image_features_vector;
  for (const ReferenceImage & refimg : reference_images)
  {
    const cv::Mat & reference_image = refimg.reference_image;
    const cv::Mat & reference_mask = refimg.reference_mask;
    const ReferencePoints & reference_points = refimg.reference_points;

    cv::Mat reference_grayscale = ColorToGrayscale(reference_image);
    cv::Mat reference_mask_grayscale;
    if (reference_mask.data)
    {
      reference_mask_grayscale = reference_mask.clone();
    }
    else
    {
      reference_mask_grayscale = cv::Mat(reference_image.rows, reference_image.cols, CV_8UC1);
      reference_mask_grayscale.setTo(255);
    }

    std::vector<cv::KeyPoint> keypoints_reference;
    cv::Mat descriptors_reference;
    // feature extraction of OpenCV is not enough orientation invariant, apparently
    ExtractFeaturesOrientationInvariant(nfeatures_reference, reference_grayscale, reference_mask_grayscale,
                                        keypoints_reference, descriptors_reference);

    m_log(1, "Keypoints reference size: " + std::to_string(keypoints_reference.size()));

    ReferenceImageFeatures feats;
    feats.descriptors_reference = descriptors_reference;
    feats.keypoints_reference = keypoints_reference;
    feats.reference_grayscale = reference_grayscale;
    feats.reference_points = reference_points;
    reference_image_features_vector.push_back(feats);
  }

  BoolVector valid_keypoints_image(keypoints_image.size(), true);

  std::vector<RansacFindHomographyResult> rfhrs;

  while (rfhrs.size() < m_config.sanity_max_objects)
  {
    m_log(1, "Keypoints image size: " + std::to_string(keypoints_image.size()));
    m_log(1, "Searching object...");

    const RansacFindHomographyResult rfhr = FindObject(image_grayscale, depth_image, image_mask,
                                                       keypoints_image, descriptors_image,
                                                       valid_keypoints_image,
                                                       reference_image_features_vector,
                                                       intrinsics, camera_pose);

    if (!rfhr.valid)
    {
      m_log(1, "Object not found!");
      break;
    }

    if (rfhr.detection_model_score < m_config.min_object_score)
    {
      m_log(1, "Object found, but score too low!");
      break;
    }

    m_log(1, "Object found:");

    PrintModel(rfhr);

    rfhrs.push_back(rfhr);

    // remove keypoints used by this model
    for (uint64 i = 0; i < keypoints_image.size(); i++)
    {
      if (!valid_keypoints_image[i])
        continue;

      const uint64 reference_id = rfhr.reference_id;
      const ReferenceImageFeatures & feats = reference_image_features_vector[reference_id];

      const cv::Point2f pt = keypoints_image[i].pt;
      double nx, ny;
      rfhr.detection_model.ApplyToPointInv(feats.reference_grayscale.cols, feats.reference_grayscale.rows, pt.x, pt.y, nx, ny);
      if (!std::isnan(nx) && !std::isnan(ny) && nx >= 0.0 && ny >= 0.0 &&
          nx < feats.reference_grayscale.cols && ny < feats.reference_grayscale.rows)
        valid_keypoints_image[i] = false;
    }
  }
  m_log(1, "Found " + std::to_string(rfhrs.size()) + " objects!");

  DetectedPackVector result;
  for (const RansacFindHomographyResult & r : rfhrs)
  {
    DetectedPack pack;
    pack.center_pose = r.pose;
    pack.depth = r.detected_nominal_size.y();
    pack.height = r.detected_nominal_size.z();
    pack.width = r.detected_nominal_size.x();
    pack.detected_left_width = r.detected_left_width;
    pack.detected_right_width = r.detected_right_width;
    pack.reference_id = r.reference_id;
    pack.is_upside_down = r.is_upside_down;
    result.push_back(pack);
  }

  // ----------- VISUALIZATION -------

  cv::Mat img_matches;
  int max_reference_width = 0;
  {
    std::mt19937 & random_generator = *m_random_generator;
    std::uniform_int_distribution<uint8> d0255(0, 255);

    int total_reference_height = 0;
    IntVector offsets;
    for (uint64 i = 0; i < reference_image_features_vector.size(); i++)
    {
      const ReferenceImageFeatures & rif = reference_image_features_vector[i];
      max_reference_width = std::max(max_reference_width, rif.reference_grayscale.cols);
      offsets.push_back(total_reference_height);
      total_reference_height += rif.reference_grayscale.rows;
    }

    const int matches_image_height = std::max(total_reference_height, image_grayscale.rows);
    const int matches_image_width = max_reference_width + image_grayscale.cols;

    img_matches = cv::Mat(matches_image_height, matches_image_width, CV_8UC3);
    img_matches = cv::Scalar(0, 0, 0);
    GrayscaleToColor(image_grayscale).copyTo(
        img_matches(cv::Rect(max_reference_width, 0, image_grayscale.cols, image_grayscale.rows)));
    for (uint64 i = 0; i < reference_image_features_vector.size(); i++)
    {
      const ReferenceImageFeatures & rif = reference_image_features_vector[i];
      GrayscaleToColor(rif.reference_grayscale).copyTo(
          img_matches(cv::Rect(0, offsets[i], rif.reference_grayscale.cols, rif.reference_grayscale.rows)));
    }

    for (const RansacFindHomographyResult & r : rfhrs)
    {
      const std::vector<cv::DMatch> & consensus_matches = r.consensus_matches;
      const cv::Point2f image_offset(max_reference_width, 0);
      const cv::Point2f reference_offset(0, offsets[r.reference_id]);

      const ReferenceImageFeatures & rif = reference_image_features_vector[r.reference_id];
      for (const cv::DMatch & match : consensus_matches)
      {
        const cv::Scalar color(d0255(random_generator), d0255(random_generator), d0255(random_generator));
        const cv::Point ref_pt = rif.keypoints_reference[match.queryIdx].pt + reference_offset;
        const cv::Point img_pt = keypoints_image[match.trainIdx].pt + image_offset;
        cv::line(img_matches, ref_pt, img_pt, color, 1);
        cv::circle(img_matches, ref_pt, 4, color, 1, 8, 0);
        cv::circle(img_matches, img_pt, 4, color, 1, 8, 0);
      }
    }
  }

  for (const RansacFindHomographyResult & rfhr : rfhrs)
  {
    const ReferenceImageFeatures & rif = reference_image_features_vector[rfhr.reference_id];
    const cv::Mat & reference_grayscale = rif.reference_grayscale;

    //-- Get the corners from the image_1 ( the object to be "detected" )
    std::vector<cv::Point2f> obj_corners;
    obj_corners.push_back(cv::Point2f(0, 0));
    for (uint64 i = 0; i < ESTIMATOR_COUNT; i++)
    {
      obj_corners.push_back(cv::Point2f((float)reference_grayscale.cols / (ESTIMATOR_COUNT) * (i + 0.5), 0 ));
    }
    obj_corners.push_back(cv::Point2f((float)reference_grayscale.cols, 0));
    obj_corners.push_back(cv::Point2f((float)reference_grayscale.cols, (float)reference_grayscale.rows));
    for (uint64 i = 0; i < ESTIMATOR_COUNT; i++)
    {
      obj_corners.push_back(cv::Point2f((float)reference_grayscale.cols - (float)reference_grayscale.cols / (ESTIMATOR_COUNT) * (i + 0.5),
                                        (float)reference_grayscale.rows));
    }
    obj_corners.push_back(cv::Point2f(0, (float)reference_grayscale.rows));
    std::vector<cv::Point2f> scene_corners(obj_corners.size());
    std::vector<cv::Point2f> scene_centers(ESTIMATOR_COUNT);
    try
    {
      cv::perspectiveTransform(obj_corners, scene_corners, rfhr.homography);
    }
    catch (cv::Exception & e)
    {
      m_log(3, "Exception in cv::perspectiveTransform: " + std::string(e.what()));
      continue;
    }

    std::vector<cv::Point2f> obj_centers;
    for (uint64 i = 0; i < ESTIMATOR_COUNT; i++)
    {
      obj_centers.push_back(cv::Point2f((float)reference_grayscale.cols / ESTIMATOR_COUNT * (i + 0.5f),
                                        (float)reference_grayscale.rows / 2));
    }

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    for (uint64 i = 0; i < scene_corners.size(); i++)
    {
      const cv::Point2f t = cv::Point2f((float)max_reference_width, 0);
      cv::line(img_matches, scene_corners[i] + t,
               scene_corners[(i + 1) % scene_corners.size()] + t, cv::Scalar(0, 255, 0), 4);
    }

    const double max_error_for_huber_loss = m_config.max_error_for_huber_loss;
    const Eigen::Vector2d translation_px_weight(m_config.translation_px_weight_x, m_config.translation_px_weight_y);
    MyRollPackDetectionEstimator estimator(reference_grayscale.cols, reference_grayscale.rows,
                                           max_error_for_huber_loss, translation_px_weight);
    for (uint64 i = 0; i < obj_corners.size(); i++)
    {
      const cv::Point2f & pt = obj_corners[i];
      const int element_n = estimator.PointToElementN(pt.x, pt.y);
      double nx, ny;
      rfhr.detection_model.ApplyToPoint(element_n, pt.x, pt.y, nx, ny);
      scene_corners[i] = cv::Point2f(nx, ny);
    }

    for (uint64 i = 0; i < obj_centers.size(); i++)
    {
      const cv::Point2f & pt = obj_centers[i];
      const int element_n = estimator.PointToElementN(pt.x, pt.y);
      double nx, ny;
      rfhr.detection_model.ApplyToPoint(element_n, pt.x, pt.y, nx, ny);
      scene_centers[i] = cv::Point2f(nx, ny);
    }

    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
    for (uint64 i = 0; i < scene_corners.size(); i++)
    {
      const cv::Point2f t = cv::Point2f((float)max_reference_width, 0);
      cv::line(img_matches, scene_corners[i] + t,
               scene_corners[(i + 1) % scene_corners.size()] + t, cv::Scalar(255, 0, 0), 4);
    }
    for (uint64 i = 0; i < scene_centers.size(); i++)
    {
      const cv::Point2f t = cv::Point2f((float)max_reference_width, 0);
      cv::circle(img_matches, scene_centers[i] + t, 6, cv::Scalar(0, 0, 255), cv::FILLED, 8, 0);
    }
  }

  //-- Show detected matches
  m_publish_image("matches", img_matches, "bgr8");

  return result;
}

PackDetection::RansacFindHomographyResult PackDetection::FindObjectPose(const cv::Mat & image_grayscale, const cv::Mat & depth_image,
                                          const cv::Mat & reference_image,
                                          const Eigen::Affine3f & camera_pose,
                                          const Intrinsics & intrinsics,
                                          const RansacFindHomographyResult & rfhr,
                                          const ReferencePoints & reference_points)
{
  RansacFindHomographyResult result = rfhr;

  Vector3dVector observed_points;
  Vector3dVector adjusted_reference_points;

  for (const std::pair<const std::string, ReferencePoint> & rp : reference_points.pts)
  {
    const cv::Point2f opt = rp.second.reference;
    const int element_n = rfhr.detection_model.PointToElementN(reference_image.cols, reference_image.rows, opt.x, opt.y);
    double nx, ny;
    rfhr.detection_model.ApplyDeformToPoint(element_n, opt.x, opt.y, nx, ny);

    Eigen::Vector3d adjusted_point;
    adjusted_point.x() = rp.second.real.x + reference_points.pixel_size_in_meters * (nx - opt.x);
    adjusted_point.y() = -reference_points.box_depth / 2.0;
    adjusted_point.z() = rp.second.real.y + reference_points.pixel_size_in_meters * (ny - opt.y);
    adjusted_reference_points.push_back(adjusted_point);

    rfhr.detection_model.ApplyToPoint(element_n, opt.x, opt.y, nx, ny);

    float maybe_depth = MY_NAN;
    const int inx = int(std::round(nx));
    const int iny = int(std::round(ny));
    if (inx >= 0 && iny >= 0 && inx < depth_image.cols && iny < depth_image.rows)
      if (const uint16 d = depth_image.at<uint16>(ny, nx))
        maybe_depth = d / 1000.0f;

    Eigen::Vector3d observed_point;
    observed_point.x() = nx;
    observed_point.y() = ny;
    observed_point.z() = maybe_depth;
    observed_points.push_back(observed_point);
  }

  // probe to see if the template is rotated 180 degrees in the image (package upside down).
  // if so, rotate 180 degrees the reference points too
  {
    const double probe_x = reference_image.cols / 2;
    const double probe_y_low = reference_image.rows * 3 / 4;
    const double probe_y_up = reference_image.rows * 1 / 4;
    const int element_low = rfhr.detection_model.PointToElementN(reference_image.cols, reference_image.rows, probe_x, probe_y_low);
    const int element_up = rfhr.detection_model.PointToElementN(reference_image.cols, reference_image.rows, probe_x, probe_y_up);
    double lx, ly;
    double ux, uy;
    rfhr.detection_model.ApplyToPoint(element_low, probe_x, probe_y_low, lx, ly);
    rfhr.detection_model.ApplyToPoint(element_up, probe_x, probe_y_up, ux, uy);

    const Eigen::Vector2d image_up(ux - lx, uy - ly);
    const Eigen::Vector3d camera_up = camera_pose.linear().inverse().cast<double>() * Eigen::Vector3d::UnitZ();
    const bool is_rotated = image_up.dot(camera_up.head<2>()) < 0.0;

    if (is_rotated)
    {
      m_log(1, "Template seems reversed in the image, rotating reference points during pose estimation.");
      result.is_upside_down = true;

      for (Eigen::Vector3d & arp : adjusted_reference_points)
      {
        // rotate 180 degrees around y
        arp.x() *= -1.0;
        arp.z() *= -1.0;
      }
    }
  }

  RollPackPoseEstimator estimator(m_config.pose_px_weight,
                                  m_config.pose_depth_weight / reference_points.pixel_size_in_meters,
                                  camera_pose.cast<double>(),
                                  intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy);

  double final_cost;
  RollPackPoseEstimator::MyRollPackPoseModelPtr pose_model = estimator.Estimate(observed_points,
                                                                                adjusted_reference_points,
                                                                                final_cost);
  if (!pose_model)
  {
    m_log(2, "Pose estimation failed!");
    result.valid = false;
    return result;
  }

  m_log(1, "Estimated pose: x " + std::to_string(pose_model->tx) + " y " + std::to_string(pose_model->ty) +
           " z " + std::to_string(pose_model->tz) + " rotation " + std::to_string(pose_model->rz * 180.0 / M_PI));

  result.pose.linear() = Eigen::AngleAxisd(pose_model->rz, Eigen::Vector3d::UnitZ()).matrix();
  result.pose.translation() = Eigen::Vector3d(pose_model->tx, pose_model->ty, pose_model->tz);

  result.detected_nominal_size.x() = reference_points.box_width;
  result.detected_nominal_size.y() = reference_points.box_depth;
  result.detected_nominal_size.z() = reference_points.box_height;

  result.detected_left_width = reference_points.box_width / 2.0;
  result.detected_right_width = reference_points.box_width / 2.0;

  for (int i = 0; i < (ESTIMATOR_COUNT - 1) / 2; i++)
  {
    const int d = (ESTIMATOR_COUNT - 1) / 2;
    result.detected_right_width +=
      rfhr.detection_model.translations_x[i + d - 1] * reference_points.pixel_size_in_meters;
  }

  for (int i = 0; i < (ESTIMATOR_COUNT - 1) / 2; i++)
  {
    result.detected_left_width -=
      rfhr.detection_model.translations_x[i] * reference_points.pixel_size_in_meters;
  }

  result.valid = true;

  return result;
}

PackDetection::RansacFindHomographyResult PackDetection::FindObject(const cv::Mat & image_grayscale, const cv::Mat & depth_image,
                                                                    const cv::Mat & image_mask,
                                      const std::vector<cv::KeyPoint> & keypoints_image,
                                      const cv::Mat & descriptors_image,
                                      const BoolVector & valid_keypoints_image,
                                      ReferenceImageFeaturesVector & reference_image_features_vector,
                                      const Intrinsics & intrinsics,
                                      const Eigen::Affine3f & camera_pose)
{
  m_log(1, "FindObject started.");

  const int knn_K = m_config.knn_K;
  const float ratio_thresh = m_config.feature_ratio_thresh;

  for (ReferenceImageFeatures & reference_image_features : reference_image_features_vector)
  {
    const std::vector<cv::KeyPoint> keypoints_reference = reference_image_features.keypoints_reference;
    const cv::Mat descriptors_reference = reference_image_features.descriptors_reference;

    // Matching descriptor vectors with a FLANN based matcher
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    matcher->knnMatch(descriptors_reference, descriptors_image, knn_matches, knn_K);

    m_log(1, "Found " + std::to_string(knn_matches.size()) + " matches.");

    // compute good matches
    FloatVector matches_score;
    std::vector<cv::DMatch> good_matches;

    for (uint64 i = 0; i < knn_matches.size(); i++)
    {
      const float th = ratio_thresh * knn_matches[i].back().distance;
      const float d0 = knn_matches[i][0].distance;
      for (uint64 i2 = 0; i2 < knn_matches[i].size() - 1; i2++)
      {
        if (!valid_keypoints_image[knn_matches[i][i2].trainIdx])
          continue;

        const float d = knn_matches[i][i2].distance;
        if (d < th)
        {
          good_matches.push_back(knn_matches[i][i2]);
          const float score = (d > 0.0001f) ? (d0 / d) : 1.0f;
          matches_score.push_back(score);
        }
      }
    }

    m_log(1, "Found " + std::to_string(good_matches.size()) + " good matches.");
    if (good_matches.size() < 10)
    {
      m_log(3, "Not enough matches!");
      return RansacFindHomographyResult();
    }

    BoolVectorVector matches_compatibility(good_matches.size(), BoolVector(good_matches.size(), true));
    FloatVectorVector matches_scale(good_matches.size(), FloatVector(good_matches.size(), NAN));

    // prepare compatibility matrix
    for (uint64 i = 0; i < good_matches.size(); i++)
    {
      for (uint64 h = i + 1; h < good_matches.size(); h++)
      {
        const float REPROJ_THRESHOLD = m_config.reproj_threshold_px;
        const float MAX_SCALING = m_config.max_scaling;
        const float MAX_FEATURE_DIFF_ANGLE = m_config.max_feature_diff_angle;

        const cv::DMatch & m1 = good_matches[i];
        const cv::DMatch & m2 = good_matches[h];

        if (m1.queryIdx == m2.queryIdx ||
            m1.trainIdx == m2.trainIdx)
        {
          matches_compatibility[i][h] = false;
          matches_compatibility[h][i] = false;

          continue;
        }

        const cv::Point2f rpt0 = keypoints_reference[m1.queryIdx].pt;
        const cv::Point2f ipt0 = keypoints_image[m1.trainIdx].pt;

        const cv::Point2f rpt = keypoints_reference[m2.queryIdx].pt;
        const cv::Point2f ipt = keypoints_image[m2.trainIdx].pt;

        // angle between the two points in reference and image
        const float angle_r = std::atan2((rpt - rpt0).y, (rpt - rpt0).x);
        const float angle_i = std::atan2((ipt - ipt0).y, (ipt - ipt0).x);
        const Eigen::Matrix2f rotation = Eigen::Rotation2Df(angle_i - angle_r).matrix();

        const float ra0 = keypoints_reference[m1.queryIdx].angle / 180.0f * M_PI;
        const Eigen::Vector2f ra0v(std::cos(ra0), std::sin(ra0));
        const float ia0 = keypoints_image[m1.trainIdx].angle / 180.0f * M_PI;
        const Eigen::Vector2f ia0v(std::cos(ia0), std::sin(ia0));

        const float ra = keypoints_reference[m2.queryIdx].angle / 180.0f * M_PI;
        const Eigen::Vector2f rav(std::cos(ra), std::sin(ra));
        const float ia = keypoints_image[m2.trainIdx].angle / 180.0f * M_PI;
        const Eigen::Vector2f iav(std::cos(ia), std::sin(ia));

        if ((rotation * ra0v).dot(ia0v) < std::cos(MAX_FEATURE_DIFF_ANGLE))
        {
          matches_compatibility[i][h] = false;
          matches_compatibility[h][i] = false;

          continue;
        }

        if ((rotation * rav).dot(iav) < std::cos(MAX_FEATURE_DIFF_ANGLE))
        {
          matches_compatibility[i][h] = false;
          matches_compatibility[h][i] = false;

          continue;
        }

        const float ni = cv::norm(ipt0 - ipt);
        const float nr = cv::norm(rpt0 - rpt);

        if (ni < 1 || nr < 1)
        {
          matches_compatibility[i][h] = false;
          matches_compatibility[h][i] = false;
          continue;
        }

        // check distance compatibility
        if (ni > nr * MAX_SCALING + REPROJ_THRESHOLD)
        {
          matches_compatibility[i][h] = false;
          matches_compatibility[h][i] = false;
          continue;
        }
        if (nr > ni * MAX_SCALING + REPROJ_THRESHOLD)
        {
          matches_compatibility[i][h] = false;
          matches_compatibility[h][i] = false;
          continue;
        }
        if (ni < nr / MAX_SCALING - REPROJ_THRESHOLD)
        {
          matches_compatibility[i][h] = false;
          matches_compatibility[h][i] = false;
          continue;
        }
        if (nr < ni / MAX_SCALING - REPROJ_THRESHOLD)
        {
          matches_compatibility[i][h] = false;
          matches_compatibility[h][i] = false;
          continue;
        }

        {
          matches_scale[i][h] = ni / nr;
          matches_scale[h][i] = ni / nr;
        }
      }
    }

    // precompute graph sel -> comp
    Uint64VectorVector matches_comp_graph(good_matches.size());
    uint64 matches_comp_graph_total = 0;
    for (uint64 i = 0; i < good_matches.size(); i++)
    {
      for (uint64 h = 0; h < good_matches.size(); h++)
      {
        if (i != h && matches_compatibility[i][h])
          matches_comp_graph[i].push_back(h);
      }
      matches_comp_graph_total += matches_comp_graph[i].size();
    }
    m_log(1, "Found " + std::to_string(matches_comp_graph_total) + " compatible matches.");

    // for (uint64 i = 0; i < matches_compatibility.size(); i++)
    // {
    //   for (uint64 h = 0; h < matches_compatibility[i].size(); h++)
    //     std::cout << (matches_compatibility[i][h] ? "1" : "0") << " ";
    //   std::cout << std::endl;
    // }

    // for (float s : matches_score)
    //   std::cout << s << " ";
    // std::cout << std::endl;

    reference_image_features.matches = good_matches;
    reference_image_features.matches_score = matches_score;
    reference_image_features.matches_compatibility = matches_compatibility;
    reference_image_features.matches_comp_graph = matches_comp_graph;
    reference_image_features.matches_scale = matches_scale;
  }

  RansacFindHomographyResult rfhr = RansacFindHomography(image_grayscale,
                                                         image_mask,
                                                         keypoints_image,
                                                         reference_image_features_vector);

  m_log(1, "Consensus size: " + std::to_string(rfhr.consensus.size()) + ", consensus score: " + std::to_string(rfhr.score) + ".");

  if (!rfhr.homography.data)
  {
    m_log(2, "No object found!");
    return RansacFindHomographyResult();
  }

  m_log(1, "Refined consensus score is " + std::to_string(rfhr.detection_model_score) + ".");
  m_log(1, "Consensus size is " + std::to_string(rfhr.detection_model_consensus.size()) + ".");
  m_log(1, "Model final cost is " + std::to_string(rfhr.detection_model_final_cost) + ".");

  std::vector<cv::DMatch> & consensus_matches = rfhr.consensus_matches;
  for (uint64 i : rfhr.detection_model_consensus)
    consensus_matches.push_back(reference_image_features_vector[rfhr.reference_id].matches[i]);

  rfhr.valid = true;

  rfhr = FindObjectPose(image_grayscale, depth_image, reference_image_features_vector[rfhr.reference_id].reference_grayscale,
                        camera_pose, intrinsics, rfhr, reference_image_features_vector[rfhr.reference_id].reference_points);

  return rfhr;
}
