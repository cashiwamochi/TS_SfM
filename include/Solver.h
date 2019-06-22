#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SVD>

namespace TS_SfM {
namespace Solver {

    using Matrix34f = Eigen::Matrix<float,3,4>;
    using Matrix44f = Eigen::Matrix<float,4,4>;

    using SvdInEight1st = Eigen::JacobiSVD< Eigen::Matrix<float,8,9>,Eigen::ColPivHouseholderQRPreconditioner>;
    using SvdInEight2nd = Eigen::JacobiSVD< Eigen::Matrix<float,3,3>,Eigen::ColPivHouseholderQRPreconditioner>;
    using SvdInTri = Eigen::JacobiSVD< Eigen::Matrix<float,4,4>,Eigen::ColPivHouseholderQRPreconditioner>;

  cv::Mat DecomposeE(const std::vector<cv::KeyPoint>& pts0,
                     const std::vector<cv::KeyPoint>& pts1,
                     const std::vector<cv::DMatch>& v_matches,
                     const cv::Mat& E);

  // Given intrinsic params and matchings nad kpts, Compute E and F matrix 
  bool SolveEpipolarConstraintRANSAC(
      const cv::Mat& image0, const cv::Mat& image1,
      const std::pair<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>>& pair_vv_kpts,
      const std::vector<cv::DMatch>& v_matches,
      cv::Mat& F, std::vector<bool>& vb_mask, int& score,
      int max_iteration = 400, float threshold = 1.5);

  std::vector<float> ComputeEpipolarDistances(const std::vector<cv::KeyPoint>& pts0,
                                              const std::vector<cv::KeyPoint>& pts1,
                                              const std::vector<cv::DMatch>& v_matches,
                                              const cv::Mat& F);
  float EvaluateFUsingEight(const std::vector<cv::Point2f>& pts0,
                            const std::vector<cv::Point2f>& pts1,
                            const cv::Mat& F);

  float ComputeEightPointsAlgorithm(const std::vector<cv::Point2f>& pts0,
                                    const std::vector<cv::Point2f>& pts1,
                                    cv::Mat& F);
};

  inline std::vector<float> Solver::ComputeEpipolarDistances(const std::vector<cv::KeyPoint>& pts0,
                                                             const std::vector<cv::KeyPoint>& pts1,
                                                             const std::vector<cv::DMatch>& v_matches,
                                                             const cv::Mat& F)
  {
    std::vector<float> vf_distances(v_matches.size(), -1.0);

    int idx = 0;
    for(auto m : v_matches) {
      cv::Mat pt0 = (cv::Mat_<float>(3,1) << pts0[m.queryIdx].pt.x, pts0[m.queryIdx].pt.y, 1.0);
      cv::Mat l = F * cv::Mat(pt0);
      cv::Point2f pt1(pts1[m.trainIdx].pt.x, pts1[m.trainIdx].pt.y);
      cv::Vec3f line(l.reshape(3).at<cv::Vec3f>());
      float _d = fabsf(line(0)*pt1.x + line(1)*pt1.y + line(2)) / sqrt(line(0)*line(0)+line(1)*line(1));

      vf_distances[idx] = _d;
      ++idx;
    }

    return vf_distances;
  }

  inline float Solver::EvaluateFUsingEight(const std::vector<cv::Point2f>& pts0,
                                        const std::vector<cv::Point2f>& pts1,
                                        const cv::Mat& F) {
    float distance = 0.0;

    for(int i = 0; i < 8; ++i) {
      cv::Mat pt = (cv::Mat_<float>(3,1) << pts0[i].x, pts0[i].y, 1.0);
      cv::Mat l = F * cv::Mat(pt);
      cv::Vec3f line(l.reshape(3).at<cv::Vec3f>());
      float _d = fabsf(line(0)*pts1[i].x + line(1)*pts1[i].y + line(2)) / sqrt(line(0)*line(0)+line(1)*line(1));

      distance += _d; 
    }

    return distance/8.0;
  };

  inline float
    Solver::ComputeEightPointsAlgorithm(const std::vector<cv::Point2f>& pts0,
                                        const std::vector<cv::Point2f>& pts1,
                                        cv::Mat& F) {
    assert(pts0.size() == 8);
    assert(pts1.size() == 8);

    float score = -1.0; 
    F = cv::Mat::zeros(3,3,CV_32F);
    
    Eigen::MatrixXf Af = Eigen::MatrixXf::Zero(8, 9);
    // Eigen::Matrix<float, Eigen::Dynamic, 9> Ad(8, 9);
    for(int i = 0; i < 8; i++) {
      const float x0 = pts0[i].x;
      const float y0 = pts0[i].y;
      const float x1 = pts1[i].x;
      const float y1 = pts1[i].y;

      Af(i, 0) = x1 * x0; 
      Af(i, 1) = x1 * y0; 
      Af(i, 2) = x1; 
      Af(i, 3) = y1 * x0; 
      Af(i, 4) = y1 * y0; 
      Af(i, 5) = y1; 
      Af(i, 6) = x0; 
      Af(i, 7) = y0; 
      Af(i, 8) = 1.0; 
    }

    SvdInEight1st svd_1st(Af, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::MatrixXf V = svd_1st.matrixV();
    Eigen::Matrix<float, 3, 3> m;
    // m << V(8,0), V(8,1), V(8,2),
    //      V(8,3), V(8,4), V(8,5),
    //      V(8,6), V(8,7), V(8,8);
    // std::cout << "=====================" << std::endl;
    // std::cout << V << std::endl;
    m << V(0,8), V(1,8), V(2,8),
         V(3,8), V(4,8), V(5,8),
         V(6,8), V(7,8), V(8,8);
    
    SvdInEight2nd svd_2nd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    Eigen::MatrixXf diag = svd_2nd.singularValues().asDiagonal();
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
    Eigen::Matrix<float, 3, 3> _F = svd_2nd.matrixU() * diag * svd_2nd.matrixV().transpose();

    // std::cout << _F << std::endl;
    // std::cout << "=====================" << std::endl;

    F.at<float>(0,0) = _F(0,0);
    F.at<float>(0,1) = _F(0,1);
    F.at<float>(0,2) = _F(0,2);
    F.at<float>(1,0) = _F(1,0);
    F.at<float>(1,1) = _F(1,1);
    F.at<float>(1,2) = _F(1,2);
    F.at<float>(2,0) = _F(2,0);
    F.at<float>(2,1) = _F(2,1);
    F.at<float>(2,2) = _F(2,2);

    score = Solver::EvaluateFUsingEight(pts0,pts1,F);

    return score;
  }

};
