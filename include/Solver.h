#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>

namespace TS_SfM {
namespace Solver {

  // Given intrinsic params and matchings nad kpts, Compute E and F matrix 
  bool SolveEpipolarConstraintRANSAC(
      const cv::Mat& K,
      const std::pair<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>>& pair_vv_kpts,
      const std::vector<cv::DMatch>& v_matches,
      cv::Mat& F, cv::Mat& E, std::vector<bool>& vb_mask, int& score,
      int max_iteration = 300, double threshold = 0.999);

  double ComputeEightPointsAlgorithm(const std::vector<cv::Point2d>& v_pts0,
                                     const std::vector<cv::Point2d>& v_pts1,
                                     cv::Mat& F);
  



}
}
