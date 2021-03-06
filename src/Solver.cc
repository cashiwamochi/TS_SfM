#include "Solver.h"
#include <random>

#include "Utils.h"

namespace TS_SfM {
namespace Solver {

  cv::Mat DecomposeE(const std::vector<cv::KeyPoint>& v_pts0,
                     const std::vector<cv::KeyPoint>& v_pts1,
                     const std::vector<cv::DMatch>& v_matches_01,
                     const cv::Mat& K,
                     const cv::Mat& E)
  {
    cv::Mat T_01 = cv::Mat::eye(3,4,CV_32FC1);
    using SvdInDecompE
      = Eigen::JacobiSVD<Eigen::Matrix<float,3,3>,Eigen::ColPivHouseholderQRPreconditioner>;
    Eigen::Matrix<float, 3, 3> _E;
    cv2eigen(E, _E);
    SvdInDecompE svd(_E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXf U = svd.matrixU();
    Eigen::MatrixXf Vt = svd.matrixV().transpose();

    Eigen::Matrix<float,3,3> W;
    W << 0.0, -1.0, 0.0,
         1.0,  0.0, 0.0,
         0.0,  0.0, 1.0;

    Matrix33f eK;
    cv2eigen(K, eK);

    std::vector< Matrix34f > v_eig_T;
    v_eig_T.reserve(4);
    Matrix34f eig_T;

    eig_T.block(0,0,3,3) = U * W * Vt;
    if(eig_T.block(0,0,3,3).determinant() < 0.0) {
      eig_T.block(0,0,3,3) *= -1.0;
    }
    eig_T.col(3) = U.col(2)/U.col(2).norm();
    v_eig_T.push_back(eK*eig_T);
    
    eig_T.block(0,0,3,3) = U * W * Vt;
    eig_T.col(3) = -U.col(2)/U.col(2).norm();
    if(eig_T.block(0,0,3,3).determinant() < 0.0) {
      eig_T.block(0,0,3,3) *= -1.0;
    }
    v_eig_T.push_back(eK*eig_T);
    
    eig_T.block(0,0,3,3) = U * W.transpose() * Vt;
    eig_T.col(3) = U.col(2)/U.col(2).norm();
    if(eig_T.block(0,0,3,3).determinant() < 0.0) {
      eig_T.block(0,0,3,3) *= -1.0;
    }
    v_eig_T.push_back(eK*eig_T);
    
    eig_T.block(0,0,3,3) = U * W.transpose() * Vt;
    eig_T.col(3) = -U.col(2)/U.col(2).norm();
    if(eig_T.block(0,0,3,3).determinant() < 0.0) {
      eig_T.block(0,0,3,3) *= -1.0;
    }
    v_eig_T.push_back(eK*eig_T);

    Matrix34f P;
    P << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0;
    P = eK * P;
    
    int correct_solution_idx = -1;
    int reconst_num_in_front_cam = 0;
    for(int i = 0; i < 4; ++i) {
      int count = 0;
      for(size_t n = 0; n < v_matches_01.size(); ++n) {
        const float x0 = v_pts0[v_matches_01[n].queryIdx].pt.x;
        const float y0 = v_pts0[v_matches_01[n].queryIdx].pt.y;
        const float x1 = v_pts1[v_matches_01[n].trainIdx].pt.x;
        const float y1 = v_pts1[v_matches_01[n].trainIdx].pt.y;
        Matrix44f A = Eigen::MatrixXf::Zero(4, 4);
        A.row(0) = x0*P.row(2) - P.row(0);
        A.row(1) = y0*P.row(2) - P.row(1);
        A.row(2) = x1*v_eig_T[i].row(2) - v_eig_T[i].row(0);
        A.row(3) = y1*v_eig_T[i].row(2) - v_eig_T[i].row(1);

        SvdInTri svd(A, Eigen::ComputeFullV);
        Eigen::MatrixXf pt4D_0 = svd.matrixV().col(3)/svd.matrixV()(3,3);
        Eigen::MatrixXf pt4D_1 = v_eig_T[i]*pt4D_0;
        if(pt4D_0(2) > 0.0 && pt4D_1(2) > 0.0) {
          ++count;
        }
      }

      if(count > reconst_num_in_front_cam) {
        reconst_num_in_front_cam = count;
        correct_solution_idx = i;
      }
    }
    
    Matrix34f eT = eK.inverse()*v_eig_T[correct_solution_idx]; 
    eigen2cv(eT, T_01);

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
    else {
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
        int current_score_inliers = 0;
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

        if(score_8 > 1.5) {
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
#if 0
          cv::Mat m_output;
          cv::drawMatches(image0, pair_vv_kpts.first,
                          image1, pair_vv_kpts.second,
                          v_matches_01_ransac, m_output);

          cv::imshow("viewer-matches-ransac", m_output);
          cv::waitKey(0);
#endif
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

  std::vector<cv::Point3f> Triangulate(const std::vector<cv::KeyPoint>& v_pts0,
                                       const std::vector<cv::KeyPoint>& v_pts1,
                                       const std::vector<cv::DMatch>& v_matches_01,
                                       const cv::Mat& K,
                                       const cv::Mat& T_01/*3x4*/)
  {
    std::vector<cv::Point3f> v_pt3D;
    // v_pt3D.resize(v_matches_01.size()); 

    Matrix33f eK;
    cv2eigen(K, eK);

    Matrix34f P;
    P << 1.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0;
    P = eK * P;

    Matrix34f eT_01;
    cv2eigen(T_01, eT_01);
    eT_01 = eK * eT_01;
    
    int reconst_num_in_front_cam = 0;

    std::vector<Eigen::Vector3d> vd_points;
    int count = 0;
    for(size_t n = 0; n < v_matches_01.size(); ++n) {
      const float x0 = v_pts0[v_matches_01[n].queryIdx].pt.x;
      const float y0 = v_pts0[v_matches_01[n].queryIdx].pt.y;
      const float x1 = v_pts1[v_matches_01[n].trainIdx].pt.x;
      const float y1 = v_pts1[v_matches_01[n].trainIdx].pt.y;

      Matrix44f A = Eigen::MatrixXf::Zero(4, 4);
      A.row(0) = x0*P.row(2) - P.row(0);
      A.row(1) = y0*P.row(2) - P.row(1);
      A.row(2) = x1*eT_01.row(2) - eT_01.row(0);
      A.row(3) = y1*eT_01.row(2) - eT_01.row(1);
  
      SvdInTri svd(A, Eigen::ComputeFullV);
      Eigen::MatrixXf pt4D_0 = svd.matrixV().col(3)/svd.matrixV()(3,3);
      Eigen::MatrixXf pt4D_1 = eT_01*pt4D_0;

      if(pt4D_0(2) > 0.0 && pt4D_1(2) > 0.0) {
        ++count;
        Eigen::Vector3d vec3{static_cast<double>(pt4D_0(0)), 
                             static_cast<double>(pt4D_0(1)),
                             static_cast<double>(pt4D_0(2))};
        vd_points.push_back(vec3);
      }
    }
  
    if(count > reconst_num_in_front_cam) {
      reconst_num_in_front_cam = count;
    }

    for(auto pt : vd_points) {
      v_pt3D.push_back(cv::Point3f((float)pt.x(), (float)pt.y(), (float)pt.z()));
    }

    return v_pt3D;
  }

  float
    ComputeEightPointsAlgorithm(const std::vector<cv::Point2f>& pts0,
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
    m << V(0,8), V(1,8), V(2,8),
         V(3,8), V(4,8), V(5,8),
         V(6,8), V(7,8), V(8,8);
    
    SvdInEight2nd svd_2nd(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
    
    Eigen::MatrixXf diag = svd_2nd.singularValues().asDiagonal();
    diag(2,2) = 0.0;
    Eigen::Matrix<float, 3, 3> _F = svd_2nd.matrixU() * diag * svd_2nd.matrixV().transpose();

    eigen2cv(_F, F);

    score = Solver::EvaluateFUsingEight(pts0,pts1,F);

    return score;
  }

  cv::Mat SolvePnP(const std::vector<cv::Point3f>& v_landmarks_w,
                   const std::vector<cv::Point2f>& v_obs_pts_c,
                   const cv::Mat& K) {
    cv::Mat cTw;




    return cTw;
  }

  cv::Mat SolvePnPRANSAC(const std::vector<cv::KeyPoint>& v_keypoints,
                         const std::vector<MapPoint>& v_mappoints,
                         const std::vector<MatchObsAndLdmk>& v_matches,
                         std::vector<bool> vb_inliers) 
  {
    cv::Mat cTw;


    return cTw;
  }

}; // Solver
}; // TS_SfM
