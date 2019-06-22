#include "Solver.h"
#include <random>

namespace TS_SfM {
namespace Solver {

  cv::Mat DecomposeE(const std::vector<cv::KeyPoint>& pts0,
                     const std::vector<cv::KeyPoint>& pts1,
                     const std::vector<cv::DMatch>& v_matches,
                     const cv::Mat& E)
  {
    cv::Mat T_01 = cv::Mat::eye(4,4,CV_32FC1);
    using SvdInDecompE = Eigen::JacobiSVD< Eigen::Matrix<float,3,3>,Eigen::ColPivHouseholderQRPreconditioner>;
    Eigen::Matrix<float, 3, 3> _E;
    _E << E.at<float>(0,8), E.at<float>(1,8), E.at<float>(2,8),
          E.at<float>(3,8), E.at<float>(4,8), E.at<float>(5,8),
          E.at<float>(6,8), E.at<float>(7,8), E.at<float>(8,8);
    SvdInDecompE svd(_E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXf U = svd.matrixU();
    Eigen::MatrixXf V = svd.matrixV();

    Eigen::Matrix<float, 3, 3> W;
    W << 0.0, -1.0, 0.0,
         1.0,  0.0, 0.0,
         0.0,  0.0, 1.0;

    std::vector< Eigen34f > v_eig_T;
    v_eig_T.reserve(4);
    Eigen34f eig_T;

    eig_T.block(0,0,3,3) = U * W * V.transpose();
    eig_T.col(3) = U.col(3);
    v_eig_T.push_back(eig_T);

    eig_T.block(0,0,3,3) = U * W * V.transpose();
    eig_T.col(3) = -U.col(3);
    v_eig_T.push_back(eig_T);

    eig_T.block(0,0,3,3) = U * W.transpose() * V.transpose();
    eig_T.col(3) = U.col(3);
    v_eig_T.push_back(eig_T);

    eig_T.block(0,0,3,3) = U * W.transpose() * V.transpose();
    eig_T.col(3) = -U.col(3);
    v_eig_T.push_back(eig_T);

    return T_01;
  }


  bool SolveEpipolarConstraintRANSAC(
      const cv::Mat& image0, const cv::Mat& image1,
      const std::pair<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>>& pair_vv_kpts,
      const std::vector<cv::DMatch>& v_matches,
      cv::Mat& F,  std::vector<bool>& vb_mask, int& score,
      int max_iteration, float threshold)
  {
    bool is_solved = false; 

    // For 8-point algorithm, 8 matches are needed.
    // However it neeeds more points for safety.
    const size_t min_num_matches = 24;
    if(v_matches.size() < min_num_matches) {
      is_solved = false; 
    }
    else{
      // Generate RANSAC random sample idx
      std::mt19937_64 mt(std::random_device{}());
      std::uniform_int_distribution<int> dist(0, (int)v_matches.size()-1);
      std::vector<std::vector<int>> vv_sample_idx(max_iteration, std::vector<int>(8,-1));
      for(int i = 0; i < max_iteration; i++) {
        for(int j = 0; j < 8; ++j) {
          vv_sample_idx[i][j] = dist(mt); 
        }
      }

      int best_iter = -1;
      int best_score_inliers = -1;
      cv::Mat best_F;
      std::vector<bool> best_mask(v_matches.size(), false);
      for(int ransac_iter = 0; ransac_iter < max_iteration; ransac_iter++) {
        int current_score_inliers = -1;
        cv::Mat current_F;
        std::vector<bool> current_mask(v_matches.size(), false);

        std::vector<cv::DMatch> v_matches_01_ransac;
        std::vector<cv::Point2f> v_pts0, v_pts1;
        v_pts0.reserve(8);
        v_pts1.reserve(8);
        for(int i=0; i<8;i++) {
          v_pts0.push_back(cv::Point2f(
            pair_vv_kpts.first[v_matches[vv_sample_idx[ransac_iter][i]].queryIdx].pt.x,
            pair_vv_kpts.first[v_matches[vv_sample_idx[ransac_iter][i]].queryIdx].pt.y)); 
          v_pts1.push_back(cv::Point2f(
            pair_vv_kpts.second[v_matches[vv_sample_idx[ransac_iter][i]].trainIdx].pt.x,
            pair_vv_kpts.second[v_matches[vv_sample_idx[ransac_iter][i]].trainIdx].pt.y)); 
          v_matches_01_ransac.push_back(v_matches[vv_sample_idx[ransac_iter][i]]);
        }

        float score_8 = ComputeEightPointsAlgorithm(v_pts0, v_pts1, current_F);

        if(score_8 > 2.0) {
#if 0
          cv::Mat m_output;
          cv::drawMatches(image0, pair_vv_kpts.first,
                          image1, pair_vv_kpts.second,
                          v_matches_01_ransac, m_output);

          cv::imshow("viewer-matches-ransac", m_output);
          cv::waitKey(0);
#endif
          continue; 
        }

        std::vector<float> vf_ditances = ComputeEpipolarDistances(pair_vv_kpts.first,
                                                                  pair_vv_kpts.second,
                                                                  v_matches, current_F); 

        // Decision part 
        int mask_idx = 0;
        for(float d : vf_ditances) {
          if(d < threshold) {
            ++current_score_inliers; 
            current_mask[mask_idx] = true;
          } 
          ++mask_idx;
        }

        if(current_score_inliers > best_score_inliers) {
          best_score_inliers = current_score_inliers;
          best_iter = ransac_iter;
          best_F = current_F.clone();
          best_mask = current_mask;
          is_solved = true;
        }
      }

      F = best_F.clone();
      vb_mask = best_mask;
      score = best_score_inliers;
      // std::cout << "Score = " << best_score_inliers 
      //           << " / " << v_matches.size() <<  std::endl;
    }
  
    return is_solved;
  }
}; // Solver
}; // TS_SfM
