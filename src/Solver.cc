#include "Solver.h"
#include <random>

namespace TS_SfM {
namespace Solver {

  void DrawEpipoleLines() {
  
  };

  bool SolveEpipolarConstraintRANSAC(
      const cv::Mat& K,
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
      for(size_t i = 0; i < max_iteration; ++i) {
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
        }

        float score_8 = ComputeEightPointsAlgorithm(v_pts0, v_pts1, current_F);

        if(score_8 < 2.0) {
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
