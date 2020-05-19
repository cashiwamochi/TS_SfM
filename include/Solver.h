#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include <Eigen/Core>

#include <Eigen/Dense>
// #include <Open3D/Open3D.h>

using Matrix33f = Eigen::Matrix<float,3,3>;
using Matrix34f = Eigen::Matrix<float,3,4>;
using Matrix44f = Eigen::Matrix<float,4,4>;

namespace TS_SfM {
  template<typename _Tp, int _rows, int _cols>
  void cv2eigen(const cv::Mat& src,
                 Eigen::Matrix<_Tp, _rows, _cols>& dst )
  {
    for(int i = 0; i < src.rows; ++i) {
      for(int j = 0; j < src.cols; ++j) {
        dst(i,j) = src.at<_Tp>(i,j);
      }
    }
    return;
  }

  template<typename _Tp, int _rows, int _cols>
  void eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols>& src,
                cv::Mat& dst )
  {
    for(int i = 0; i < src.rows(); ++i) {
      for(int j = 0; j < src.cols(); ++j) {
        dst.at<_Tp>(i,j) = src(i,j);
      }
    }
    return;
  }

  class MapPoint;
  struct MatchObsAndLdmk;

namespace Solver {

    using SvdInEight1st = Eigen::JacobiSVD< Eigen::Matrix<float,8,9>,Eigen::ColPivHouseholderQRPreconditioner>;
    using SvdInEight2nd = Eigen::JacobiSVD< Eigen::Matrix<float,3,3>,Eigen::ColPivHouseholderQRPreconditioner>;
    using SvdInTri = Eigen::JacobiSVD< Eigen::Matrix<float,4,4>,Eigen::ColPivHouseholderQRPreconditioner>;

  cv::Mat DecomposeE(const std::vector<cv::KeyPoint>& pts0,
                     const std::vector<cv::KeyPoint>& pts1,
                     const std::vector<cv::DMatch>& v_matches,
                     const cv::Mat& K,
                     const cv::Mat& E);

  // Given intrinsic params and matchings nad kpts, Compute E and F matrix 
  bool SolveEpipolarConstraintRANSAC(
      const cv::Mat& image0, const cv::Mat& image1,
      const std::pair<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>>& pair_vv_kpts,
      const std::vector<cv::DMatch>& v_matches,
      cv::Mat& F, std::vector<bool>& vb_mask, int& score,
      int max_iteration = 800, float threshold = 0.9);

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

  std::vector<cv::Point3f> Triangulate(const std::vector<cv::KeyPoint>& v_pts0,
                                       const std::vector<cv::KeyPoint>& v_pts1,
                                       const std::vector<cv::DMatch>& v_matches_01,
                                       const cv::Mat& K,
                                       const cv::Mat& T_01/*3x4*/);

  std::vector<cv::Point3f> Triangulate(const std::vector<cv::KeyPoint>& v_pts0,
                                       const std::vector<cv::KeyPoint>& v_pts1,
                                       const std::vector<cv::DMatch>& v_matches_01,
                                       const cv::Mat& K,
                                       const cv::Mat& T_01/*3x4*/);

  float
    ComputeEightPointsAlgorithm(const std::vector<cv::Point2f>& pts0,
                                const std::vector<cv::Point2f>& pts1,
                                cv::Mat& F);

  cv::Mat SolvePnPRANSAC(const std::vector<cv::KeyPoint>& v_keypoints,
                         const std::vector<MapPoint>& v_mappoints,
                         const std::vector<MatchObsAndLdmk>& v_matches,
                         std::vector<bool> vb_inliers);

  cv::Mat SolvePnP(const std::vector<cv::Point3f>& v_landmarks_w,
                   const std::vector<cv::Point2f>& v_obs_pts_c,
                   const cv::Mat& K);

}; // Solver namespace

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
};
