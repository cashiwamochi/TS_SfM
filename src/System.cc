#include "System.h"

#include "Frame.h"
#include "KPExtractor.h"

#include "Matcher.h"
#include "Solver.h"

#include "Reconstructor.h"
#include "Map.h"
#include "MapPoint.h"

#include "Viewer.h"

#include <functional>

namespace TS_SfM {
  System::System(const std::string& str_config_file) : m_config_file(str_config_file) {
    std::pair<SystemConfig, Camera> _pair_config = ConfigLoader::LoadConfig(str_config_file);  
    m_config = _pair_config.first;
    m_camera = _pair_config.second;
    
    m_vstr_image_names = ConfigLoader::ReadImagesInDir(m_config.str_path_to_images);
    m_vm_images = ConfigLoader::LoadImages(m_vstr_image_names);

    if (m_camera.f_cx < 1.0) {
      m_camera.f_cx *= (float)m_vm_images[0].rows; 
    }
    if (m_camera.f_cy < 1.0) {
      m_camera.f_cy *= (float)m_vm_images[0].cols; 
    }

    m_image_width = m_vm_images[0].cols;
    m_image_height = m_vm_images[0].rows;

    ShowConfig();
    m_v_frames.reserve((int)m_vm_images.size()); 

    // Matcher::MatcherConfig m_matcher_config = ConfigLoader::LoadMatcherConfig(str_config_file);  
    m_p_extractor.reset(new KPExtractor(m_image_width, m_image_height,
                        ConfigLoader::LoadExtractorConfig(str_config_file)));

    // m_p_map = std::make_shared<Map>();
    m_p_reconstructor.reset(new Reconstructor(str_config_file));
  }

  System::~System() {

  };

  void System::DrawEpiLines(const Frame& f0, const Frame& f1, 
                            const std::vector<cv::DMatch>& v_matches01, const std::vector<bool>& vb_mask,
                            const cv::Mat& F) const
  {
    cv::Mat output = f1.GetImage().clone();
    cv::Mat image0 = f0.GetImage().clone();
    int max_line_num = 20;
    int line_num = 0;
    std::vector<cv::KeyPoint> vkpts0 = f0.GetKeyPoints();
    std::vector<cv::KeyPoint> vkpts1 = f1.GetKeyPoints();
    for(size_t i = 0; i < vb_mask.size(); ++i) {
      if(vb_mask[i] && line_num < max_line_num) {
        cv::Mat pt0 = (cv::Mat_<float>(3,1) << vkpts0[i].pt.x, vkpts0[i].pt.y, 1.0);
        cv::Mat l = F * cv::Mat(pt0);
        cv::Vec3f line(l.reshape(3).at<cv::Vec3f>());

        cv::line(output,
                 cv::Point2f(0, -line(2)/line(1)),
                 cv::Point2f((float)output.cols-1.0, (-line(2)-line(0)*(output.cols-1))/line(1) ), 
                 cv::Scalar(0,255,0), 1);

        cv::circle(image0, cv::Point((int)vkpts0[i].pt.x, (int)vkpts0[i].pt.y), 3,cv::Scalar(0,0,255), 2);
      
        i += 30;
        line_num++;
      } 
    }

    cv::imshow("epipolar-line", output);
    cv::imshow("image0", image0);
    cv::waitKey();


    return;
  }

  void System::InitializeFrames(std::vector<Frame>& v_frames, std::vector<cv::Mat>& vm_images,
                                const std::shared_ptr<KPExtractor>& p_extractor)
  {
    for (size_t i = 0; i < vm_images.size(); i++) {
      Frame frame(i, vm_images[i], p_extractor); 
      v_frames.push_back(frame);
    }
    std::cout << " Done. " << std::endl;

    std::cout << "[LOG] "
              << "SfM pipeline starts ...";

#if 0
    for (size_t i = 0; i < v_frames.size(); i++) {
      // v_frames[i].ShowFeaturePoints();    
      v_frames[i].ShowFeaturePointsInGrids();    
    }
#endif

    std::cout << "[LOG] "
              << "Extracting Feature points ...";
    std::cout << " Done. " << std::endl;

    return;
  }

  // Initialization is done in 3-view geometry
  int System::InitializeGlobalMap(std::vector<std::reference_wrapper<Frame>>& v_frames) {
    int num_map_points = -1; 
    assert(v_frames.size() == 3);  

    Frame& frame_1st = v_frames[0].get();
    Frame& frame_2nd = v_frames[1].get();
    Frame& frame_3rd = v_frames[2].get();

    Matcher matcher(ConfigLoader::LoadMatcherConfig(m_config_file));

    std::vector<cv::DMatch> v_matches_12 = matcher.GetMatches(frame_1st,frame_2nd);
    std::vector<cv::DMatch> v_matches_13 = matcher.GetMatches(frame_1st,frame_3rd);
    std::vector<cv::DMatch> v_matches_23 = matcher.GetMatches(frame_2nd,frame_3rd);

    // Compute Fundamental Matrix
    cv::Mat mK = (cv::Mat_<float>(3,3) << m_camera.f_fx, 0.0, m_camera.f_cx,
                                          0.0, m_camera.f_fy, m_camera.f_cy,
                                          0.0,           0.0,           1.0);

    
    cv::Mat mF;
    std::vector<bool> vb_mask;
    int score;
    Solver::SolveEpipolarConstraintRANSAC(frame_1st.GetImage(), frame_2nd.GetImage(),  
                                          std::make_pair(frame_1st.GetKeyPoints(),frame_2nd.GetKeyPoints()),
                                          v_matches_12, mF, vb_mask, score);

    // remain only inlier matches
    std::vector<cv::DMatch> _v_matches_12 = v_matches_12;
    v_matches_12.clear();
    for(size_t i = 0; i < _v_matches_12.size(); i++) {
      if(vb_mask[i])  {
        v_matches_12.push_back(_v_matches_12[i]); 
      }
    }

    std::cout << "Score = " << score
              << " / " << v_matches_12.size() <<  std::endl;

    if(false) DrawEpiLines(frame_1st, frame_2nd, v_matches_12, vb_mask, mF);

    // decompose E
    cv::Mat mE = mK.t() * mF * mK;
    cv::Mat T_01 = Solver::DecomposeE(frame_1st.GetKeyPoints(), frame_2nd.GetKeyPoints(), v_matches_12, mK, mE);

    // Triangulation

    // Bundle Adjustment

    return num_map_points;
  }

  int System::FlexibleInitializeGlobalMap(std::vector<std::reference_wrapper<Frame>>& v_frames) {
    int num_map_points = -1; 

    int num_pair_frame = (int)v_frames.size();

    Matcher matcher(ConfigLoader::LoadMatcherConfig(m_config_file));
    std::vector<std::vector<cv::DMatch>> vv_matches;
    vv_matches.reserve(num_pair_frame);
    std::vector<std::pair<int,int>> v_pair_frames;
    v_pair_frames.reserve(num_pair_frame);

    for(int i = 0; i < (int)v_frames.size()-1; ++i) {
      int j = i + 1;
      v_pair_frames.push_back(std::make_pair(i,j));
      std::vector<cv::DMatch> v_matches_ij = matcher.GetMatches(v_frames[i],v_frames[j]);
      vv_matches.push_back(v_matches_ij);
    }

    // Compute Fundamental Matrix
    cv::Mat mK = (cv::Mat_<float>(3,3) << m_camera.f_fx, 0.0, m_camera.f_cx,
                                          0.0, m_camera.f_fy, m_camera.f_cy,
                                          0.0,           0.0,           1.0);

    int center_frame_idx = (int)(v_frames.size() - 1)/2;
    int distance_to_edge = (int)v_frames.size() - center_frame_idx;

    for(int step_from_center = 0; step_from_center < distance_to_edge-1; ++step_from_center) {
      if(center_frame_idx+step_from_center < (int)v_frames.size()) {
        // We compute matches from old to new always
        int src_frame_idx = center_frame_idx + step_from_center;
        int dst_frame_idx = center_frame_idx + step_from_center + 1;
        Frame& src_frame = v_frames[src_frame_idx].get();
        Frame& dst_frame = v_frames[dst_frame_idx].get();
        std::vector<cv::DMatch> v_matches = vv_matches[src_frame_idx];
        cv::Mat mF;
        std::vector<bool> vb_mask;
        int score;
        Solver::SolveEpipolarConstraintRANSAC(src_frame.GetImage(), dst_frame.GetImage(),  
                                              std::make_pair(src_frame.GetKeyPoints(),dst_frame.GetKeyPoints()),
                                              v_matches, mF, vb_mask, score);

        std::vector<cv::DMatch> _v_matches = v_matches;
        v_matches.clear();
        for(size_t i = 0; i < _v_matches.size(); i++) {
          if(vb_mask[i])  {
            v_matches.push_back(_v_matches[i]); 
          }
        }

        std::cout << "Score = " << score
                  << " / " << _v_matches.size() <<  std::endl;

        if(true) DrawEpiLines(src_frame, dst_frame, v_matches, vb_mask, mF);

        // decompose E
        cv::Mat mE = mK.t() * mF * mK;
        cv::Mat T_01 = Solver::DecomposeE(src_frame.GetKeyPoints(), dst_frame.GetKeyPoints(), v_matches, mK, mE);
        src_frame.SetMatchesToNew(v_matches);
        dst_frame.SetMatchesToOld(v_matches);

        // Triangulation
        std::vector<cv::Point3f> v_pts_3d = Solver::Triangulate(src_frame.GetKeyPoints(),
                                                                dst_frame.GetKeyPoints(),
                                                                v_matches, mK, T_01); 
      }

      if(center_frame_idx-step_from_center >= 0) {
        // We compute matches from old to new always
        int src_frame_idx = center_frame_idx + step_from_center - 1;
        int dst_frame_idx = center_frame_idx + step_from_center;
        Frame& src_frame = v_frames[src_frame_idx].get();
        Frame& dst_frame = v_frames[dst_frame_idx].get();
        std::vector<cv::DMatch> v_matches = vv_matches[src_frame_idx];
        cv::Mat mF;
        std::vector<bool> vb_mask;
        int score;
        Solver::SolveEpipolarConstraintRANSAC(src_frame.GetImage(), dst_frame.GetImage(),  
                                              std::make_pair(src_frame.GetKeyPoints(),dst_frame.GetKeyPoints()),
                                              v_matches, mF, vb_mask, score);

        std::vector<cv::DMatch> _v_matches = v_matches;
        v_matches.clear();
        for(size_t i = 0; i < _v_matches.size(); i++) {
          if(vb_mask[i])  {
            v_matches.push_back(_v_matches[i]); 
          }
        }

        std::cout << "Score = " << score
                  << " / " << _v_matches.size() <<  std::endl;

        if(true) DrawEpiLines(src_frame, dst_frame, v_matches, vb_mask, mF);

        // decompose E
        cv::Mat mE = mK.t() * mF * mK;
        cv::Mat T_01 = Solver::DecomposeE(src_frame.GetKeyPoints(), dst_frame.GetKeyPoints(), v_matches, mK, mE);

        src_frame.SetMatchesToNew(v_matches);
        dst_frame.SetMatchesToOld(v_matches);
        // Triangulation
        std::vector<cv::Point3f> v_pts_3d = Solver::Triangulate(src_frame.GetKeyPoints(), 
                                                                dst_frame.GetKeyPoints(),
                                                                v_matches, mK, T_01); 

      }
    }

    // Convert frames to keyframes
    // All data should be inserted to Map

#if 0
    {
      using namespace open3d;

      utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

      auto cloud_ptr = std::make_shared<geometry::PointCloud>();
      std::vector<Eigen::Vector3d> vd_points;
      for(int i = 0; i < 30; ++i) {
        for(int j = 0; j < 30; ++j) {
          Eigen::Vector3d vec3{(double)i, (double)j, 0.0};
          vd_points.push_back(vec3);
        }
      }
      cloud_ptr->points_ = vd_points;
      cloud_ptr->NormalizeNormals();
      visualization::DrawGeometries({cloud_ptr}, "PointCloud", 1600, 900);
      utility::LogInfo("End of the test.\n");

    }
#endif

    return num_map_points;
  }



  void System::Run() {
    std::cout << "[LOG] " 
              << "Start Processing ..."
              << std::endl;

    InitializeFrames(m_v_frames, m_vm_images, m_p_extractor);

    std::vector<std::reference_wrapper<Frame>> 
      v_ini_frames{m_v_frames[0], m_v_frames[1],m_v_frames[2]};
#if 0
    InitializeGlobalMap(v_ini_frames);
#else
    FlexibleInitializeGlobalMap(v_ini_frames);
#endif

    std::cout << std::endl;
    return;
  };

  void System::ShowConfig() {
    std::cout << "[Config.path2images] "
              << m_config.str_path_to_images
              << std::endl;

    std::cout << "[Images] " 
              << m_vm_images.size() 
              << std::endl;
    
    return;
  }

} // namespace
