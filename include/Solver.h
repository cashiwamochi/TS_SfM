#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>

namespace TS_SfM {
namespace Solver {

  using SvdInEight = Eigen::JacobiSVD< Eigen::Matrix<double,8,9>, Eigen::ColPivHouseholderQRPreconditioner>;

  // Given intrinsic params and matchings nad kpts, Compute E and F matrix 
  bool SolveEpipolarConstraintRANSAC(
      const cv::Mat& K,
      const std::pair<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>>& pair_vv_kpts,
      const std::vector<cv::DMatch>& v_matches,
      cv::Mat& F, cv::Mat& E, std::vector<bool>& vb_mask, int& score,
      int max_iteration = 300, double threshold = 0.999);

  double ComputeEightPointsAlgorithm(const std::vector<cv::Point2f>& pts0,
                                     const std::vector<cv::Point2f>& pts1,
                                     cv::Mat& F);
  // double ComputeEightPointsAlgorithm(const std::vector<cv::KeyPoint>& kpts0,
  //                                    const std::vector<cv::KeyPoint>& kpts1,
  //                                    cv::Mat& F);
};

  inline double 
    Solver::ComputeEightPointsAlgorithm(const std::vector<cv::Point2f>& pts0,
                                        const std::vector<cv::Point2f>& pts1,
                                        cv::Mat& F) {
    assert(pts0.size() == 8);
    assert(pts1.size() == 8);

    double score = -1.0; 
    
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(8, 9);
    for(int i = 0; i < 8; i++) {
      const double x0 = (double)pts0[i].x;
      const double y0 = (double)pts0[i].y;
      const double x1 = (double)pts1[i].x;
      const double y1 = (double)pts1[i].y;

      Ad(i, 0) = x1 * x0; 
      Ad(i, 1) = x1 * y0; 
      Ad(i, 2) = x1; 
      Ad(i, 3) = y1 * x0; 
      Ad(i, 4) = y1 * y0; 
      Ad(i, 5) = y1; 
      Ad(i, 6) = x0; 
      Ad(i, 7) = y0; 
      Ad(i, 8) = 1.0; 
    }

    SvdInEight svd(Ad, Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "=====================" << std::endl;
    std::cout << svd.singularValues() << std::endl;
    std::cout << "=====================" << std::endl;
    std::cout << svd.matrixU() << std::endl;
    std::cout << "=====================" << std::endl;
    std::cout << svd.matrixV() << std::endl;
    std::cout << "=====================" << std::endl;

    return score;


    return score;
  }
  //
  // inline double 
  //   Solver::ComputeEightPointsAlgorithm(const std::vector<cv::KeyPoint>& kpts0,
  //                                       const std::vector<cv::KeyPoint>& kpts1,
  //                                       cv::Mat& F) {
  //   assert(kpts0.size() == 8);
  //   assert(kpts1.size() == 8);
  //
  //   double score = -1.0; 
  //  
  //   Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(8, 9);
  //   for(int i = 0; i < 8; i++) {
  //     const double x0 = (double)kpts0[i].pt.x;
  //     const double y0 = (double)kpts0[i].pt.y;
  //     const double x1 = (double)kpts1[i].pt.x;
  //     const double y1 = (double)kpts1[i].pt.y;
  //
  //     Ad(i, 0) = x1 * x0; 
  //     Ad(i, 1) = x1 * y0; 
  //     Ad(i, 2) = x1; 
  //     Ad(i, 3) = y1 * x0; 
  //     Ad(i, 4) = y1 * y0; 
  //     Ad(i, 5) = y1; 
  //     Ad(i, 6) = x0; 
  //     Ad(i, 7) = y0; 
  //     Ad(i, 8) = 1.0; 
  //   }
  // }
  

};
