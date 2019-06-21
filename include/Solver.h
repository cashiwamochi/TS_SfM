#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>

namespace TS_SfM {
namespace Solver {

    using SvdInEight1st = Eigen::JacobiSVD< Eigen::Matrix<double,8,9>,Eigen::ColPivHouseholderQRPreconditioner>;
    using SvdInEight2nd = Eigen::JacobiSVD< Eigen::Matrix<double,3,3>,Eigen::ColPivHouseholderQRPreconditioner>;

  // Given intrinsic params and matchings nad kpts, Compute E and F matrix 
  bool SolveEpipolarConstraintRANSAC(
      const cv::Mat& K,
      const std::pair<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>>& pair_vv_kpts,
      const std::vector<cv::DMatch>& v_matches,
      cv::Mat& F, cv::Mat& E, std::vector<bool>& vb_mask, int& score,
      int max_iteration = 300, double threshold = 0.999);

  void DrawEpipoleLines();

  double ComputeEightPointsAlgorithm(const std::vector<cv::Point2f>& pts0,
                                     const std::vector<cv::Point2f>& pts1,
                                     cv::Mat& F);
};

  inline double 
    Solver::ComputeEightPointsAlgorithm(const std::vector<cv::Point2f>& pts0,
                                        const std::vector<cv::Point2f>& pts1,
                                        cv::Mat& F) {
    assert(pts0.size() == 8);
    assert(pts1.size() == 8);

    double score = -1.0; 
    F = cv::Mat::zeros(3,3,CV_64F);
    
    Eigen::MatrixXd Ad = Eigen::MatrixXd::Zero(8, 9);
    // Eigen::Matrix<double, Eigen::Dynamic, 9> Ad(8, 9);
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

    SvdInEight1st svd_1st(Ad, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXd V = svd_1st.matrixV();
    Eigen::Matrix<double, 3, 3> m;
    // m << V(8,0), V(8,1), V(8,2),
    //      V(8,3), V(8,4), V(8,5),
    //      V(8,6), V(8,7), V(8,8);
    // std::cout << "=====================" << std::endl;
    // std::cout << V << std::endl;
    m << V(0,8), V(1,8), V(2,8),
         V(3,8), V(4,8), V(5,8),
         V(6,8), V(7,8), V(8,8);
    
    SvdInEight2nd svd_2nd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    Eigen::MatrixXd diag = svd_2nd.singularValues().asDiagonal();
    // std::cout << "=====================" << std::endl;
    // std::cout << diag << std::endl;
    // std::cout << "=====================" << std::endl;
    // std::cout << svd_2nd.matrixU() << std::endl;
    // std::cout << "=====================" << std::endl;
    // std::cout << svd_2nd.matrixV() << std::endl;
    // std::cout << "=====================" << std::endl;
    diag(2,2) = 0.0;
    // std::cout << "=====================" << std::endl;
    // std::cout << diag << std::endl;
    Eigen::Matrix<double, 3, 3> _F = svd_2nd.matrixU() * diag * svd_2nd.matrixV().transpose();

    // std::cout << _F << std::endl;
    // std::cout << "=====================" << std::endl;

    F.at<double>(0,0) = _F(0,0);
    F.at<double>(0,1) = _F(0,1);
    F.at<double>(0,2) = _F(0,2);
    F.at<double>(1,0) = _F(0,0);
    F.at<double>(1,1) = _F(1,1);
    F.at<double>(1,2) = _F(1,2);
    F.at<double>(2,0) = _F(0,0);
    F.at<double>(2,1) = _F(2,1);
    F.at<double>(2,2) = _F(2,2);

    return score;
  }

};
