#include "Solver.h"
#include <random>

namespace TS_SfM {
namespace Solver {

  bool SolveEpipolarConstraintRANSAC(
      const cv::Mat& K,
      const std::pair<std::vector<cv::KeyPoint>,std::vector<cv::KeyPoint>>& pair_vv_kpts,
      const std::vector<cv::DMatch>& v_matches,
      cv::Mat& F, cv::Mat& E, std::vector<bool>& vb_mask, int& score,
      int max_iteration, double threshold)
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
    std::uniform_int_distribution<int> dist(0, (int)v_matches.size());
    std::vector<std::vector<int>> vv_sample_idx(max_iteration, std::vector<int>(8,-1));
    for(size_t i = 0; i < max_iteration; ++i) {
      for(int j = 0; j < 8; ++j) {
        vv_sample_idx[i][j] = dist(mt); 
      }
    }

    std::cout << "ok" << std::endl;
    for(int ransac_iter = 0; ransac_iter < max_iteration; ransac_iter++) {
      cv::Mat _F;
#if 1
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
#endif

      double score_1st = ComputeEightPointsAlgorithm(v_pts0, v_pts1, _F);

    }
    
    }
  
    return is_solved;
  }
} // Solver
} // TS_SfM
