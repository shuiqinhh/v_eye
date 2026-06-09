#include "mbl.hpp"

#include <fstream>
#include <iostream>

namespace nav
{

Mbl::Mbl(const std::string & model_path)
{
  // Hard-coded intrinsics matching configs/orb_slam3.yaml
  K_ = (cv::Mat_<float>(3, 3) << 2601.5786f, 0, 778.7720f, 0, 2585.1271f, 378.0851f, 0, 0, 1);

  orb_ = cv::ORB::create(2000, 1.2f, 8, 20, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
  matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);

  load_model(model_path);
}

void Mbl::load_model(const std::string & path)
{
  std::ifstream f(path, std::ios::binary);
  if (!f.is_open()) {
    std::cerr << "[MBL] Cannot open model: " << path << std::endl;
    return;
  }

  uint32_t n = 0;
  f.read(reinterpret_cast<char *>(&n), sizeof(n));

  points_3d_.reserve(n);
  descriptors_ = cv::Mat(static_cast<int>(n), 32, CV_8U);

  for (uint32_t i = 0; i < n; ++i) {
    float xyz[3];
    f.read(reinterpret_cast<char *>(xyz), sizeof(xyz));
    points_3d_.emplace_back(xyz[0], xyz[1], xyz[2]);
    f.read(reinterpret_cast<char *>(descriptors_.ptr<uchar>(static_cast<int>(i))), 32);
  }
  f.close();
  std::cout << "[MBL] Loaded " << n << " model points." << std::endl;
}

MblResult Mbl::localize(const cv::Mat & bgr_image)
{
  MblResult result;

  if (points_3d_.empty() || descriptors_.rows < 10) return result;

  // 1. Extract ORB features from query image
  cv::Mat gray;
  cv::cvtColor(bgr_image, gray, cv::COLOR_BGR2GRAY);
  std::vector<cv::KeyPoint> query_kps;
  cv::Mat query_desc;
  orb_->detectAndCompute(gray, cv::Mat(), query_kps, query_desc);

  if (query_kps.empty()) return result;

  // 2. Brute-force match query -> model
  std::vector<std::vector<cv::DMatch>> knn_matches;
  matcher_->knnMatch(query_desc, descriptors_, knn_matches, 2);

  // 3. Lowe's ratio test to filter good matches
  const float ratio_thresh = 0.75f;
  std::vector<cv::Point2f> pts_2d;
  std::vector<cv::Point3f> pts_3d;
  pts_2d.reserve(knn_matches.size());
  pts_3d.reserve(knn_matches.size());

  for (const auto & knn : knn_matches) {
    if (knn.size() < 2) continue;
    if (knn[0].distance < ratio_thresh * knn[1].distance) {
      pts_2d.push_back(query_kps[static_cast<size_t>(knn[0].queryIdx)].pt);
      pts_3d.push_back(points_3d_[static_cast<size_t>(knn[0].trainIdx)]);
    }
  }

  if (pts_2d.size() < 8) return result;  // need at least 8 for PnP

  // 4. PnP + RANSAC
  cv::Mat rvec, tvec, inlier_mask;
  cv::solvePnPRansac(pts_3d, pts_2d, K_, cv::Mat(), rvec, tvec,
                     false, 100, 8.0f, 0.99f, inlier_mask);

  int inlier_count = 0;
  if (!inlier_mask.empty()) {
    inlier_count = cv::countNonZero(inlier_mask);
  }

  if (inlier_count < 8) return result;

  // 5. Refine with all inliers
  std::vector<cv::Point2f> inlier_2d;
  std::vector<cv::Point3f> inlier_3d;
  for (int i = 0; i < inlier_mask.rows; ++i) {
    if (inlier_mask.at<uchar>(i)) {
      inlier_2d.push_back(pts_2d[static_cast<size_t>(i)]);
      inlier_3d.push_back(pts_3d[static_cast<size_t>(i)]);
    }
  }
  cv::solvePnP(inlier_3d, inlier_2d, K_, cv::Mat(), rvec, tvec, true);

  // Convert to rotation matrix
  cv::Mat R_mat;
  cv::Rodrigues(rvec, R_mat);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      result.R_cw(r, c) = static_cast<float>(R_mat.at<double>(r, c));
  result.t_cw = Eigen::Vector3f(
    static_cast<float>(tvec.at<double>(0)),
    static_cast<float>(tvec.at<double>(1)),
    static_cast<float>(tvec.at<double>(2)));

  result.inliers = inlier_count;

  // 6. Compute feature distribution (SDE — area of fitted ellipse)
  if (inlier_count >= 3) {
    cv::Mat pts_mat(static_cast<int>(inlier_2d.size()), 2, CV_32F);
    for (size_t i = 0; i < inlier_2d.size(); ++i) {
      pts_mat.at<float>(static_cast<int>(i), 0) = inlier_2d[i].x;
      pts_mat.at<float>(static_cast<int>(i), 1) = inlier_2d[i].y;
    }
    cv::Mat covar, mean;
    cv::calcCovarMatrix(pts_mat, covar, mean, cv::COVAR_NORMAL | cv::COVAR_ROWS);
    float sx = std::sqrt(std::abs(covar.at<double>(0, 0)));
    float sy = std::sqrt(std::abs(covar.at<double>(1, 1)));
    float img_area = static_cast<float>(bgr_image.cols * bgr_image.rows);
    result.distribution_area = 3.14159f * sx * sy / img_area;
  }

  result.valid = true;
  return result;
}

}  // namespace nav
