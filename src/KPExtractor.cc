#include "KPExtractor.h"

namespace TS_SfM {
  KPExtractor::KPExtractor(const unsigned int image_width,
              const unsigned int image_height,
              const ExtractorConfig& _config)
    : m_image_width(image_width), m_image_height(image_height),
      m_num_vertical_grid((unsigned int)image_height/_config.grid_height), 
      m_num_horizontal_grid((unsigned int)image_width/_config.grid_width), m_config(_config)
  {
    // AKAZE Config
    cv::AKAZE::DescriptorType descriptor_type=cv::AKAZE::DESCRIPTOR_MLDB;
    int descriptor_size=0;
    int descriptor_channels=3;
    float threshold=m_config.threshold;
    int nOctaves=m_config.octaves;
    int nOctaveLayers=m_config.octavelayers;
    cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2;
    m_p_extractor
      = cv::AKAZE::create(descriptor_type,descriptor_size,descriptor_channels,threshold,
                          nOctaves, nOctaveLayers, diffusivity);

    SetGrids();
  }

  KPExtractor::KPExtractor()
  : m_image_width(0), m_image_height(0),
    m_num_vertical_grid(0), m_num_horizontal_grid(0), 
    m_config(ExtractorConfig())
  {
  }

  KPExtractor::~KPExtractor()
  {
  }

  void KPExtractor::ExtractFeaturePoints(cv::Mat m_input,
                                         std::vector<cv::KeyPoint>& v_kpts,
                                         cv::Mat& m_descriptors) {
    v_kpts.clear();
    m_descriptors.release();
    m_p_extractor->detectAndCompute(m_input, cv::noArray(), v_kpts, m_descriptors);
  
    return; 
  }

  std::vector<std::vector<unsigned int>> 
  KPExtractor::DistributeToGrids(const std::vector<cv::KeyPoint>& v_keypoints,
                                 const cv::Mat& m_descriptors,
                                 std::vector<std::vector<std::vector<cv::KeyPoint>>>& vvv_grid_kpts,
                                 std::vector<std::vector<cv::Mat>>& vvm_grid_descs) 
  {
    std::vector<KPData> v_kpdata;
    v_kpdata.reserve(m_descriptors.rows);

    for(int row = 0; row < m_descriptors.rows; row++) {
      v_kpdata.push_back(KPData(v_keypoints[row], m_descriptors.row(row))); 
     }

    // sort KeyPoint and Descriptors 
    std::sort(v_kpdata.begin(), v_kpdata.end(), KPData::cmp);

    vvv_grid_kpts.clear();
    vvm_grid_descs.clear();

    // initialize vectors for grids
    std::vector<std::vector<unsigned int>> 
      vv_num_grid_kpts(m_num_vertical_grid, std::vector<unsigned int>(m_num_horizontal_grid,0));
    vvv_grid_kpts.resize(m_num_vertical_grid);
    vvm_grid_descs.resize(m_num_vertical_grid);
    for(unsigned int i = 0; i < m_num_vertical_grid; i++) {
      vvv_grid_kpts[i].resize(m_num_horizontal_grid);
      vvm_grid_descs[i].resize(m_num_horizontal_grid);
      for(unsigned int j = 0; j < m_num_horizontal_grid; j++) {
        vvv_grid_kpts[i][j].clear(); 
        vvm_grid_descs[i][j].release();
      }
    }

    // insert keypoint data to each grids
    for(KPData data : v_kpdata) {
      std::pair<int,int> grid_idx = GetWhichGrid(data.kp.pt); 
      if(vv_num_grid_kpts[grid_idx.first][grid_idx.second]+1 > m_config.num_in_grid) {
        continue; 
      }
      vvv_grid_kpts[grid_idx.first][grid_idx.second].push_back(data.kp);
      vvm_grid_descs[grid_idx.first][grid_idx.second].push_back(data.descriptor);
      vv_num_grid_kpts[grid_idx.first][grid_idx.second]++;
    }

#if 0
    for(unsigned int row = 0; row < m_num_vertical_grid; row++) {
      for(unsigned int col = 0; col < m_num_horizontal_grid; col++) {
        unsigned int num_in_grid = (vvv_grid_kpts[row][col].size() < (size_t)m_config.num_in_grid)
                                   ? vvv_grid_kpts[row][col].size() : m_config.num_in_grid; 
      } 
    }
#endif
  
    
    return vv_num_grid_kpts;
  }

  inline std::pair<int, int> KPExtractor::GetWhichGrid(const cv::Point2f& pt) {
    int horizontal_grid_idx 
      = static_cast<int>((pt.x - (m_image_width%m_config.grid_width)/2.0)/(float)m_config.grid_width);
    int vertical_grid_idx 
      = static_cast<int>((pt.y - (m_image_height%m_config.grid_height)/2.0)/(float)m_config.grid_height);

    if(horizontal_grid_idx == m_num_horizontal_grid) {
      horizontal_grid_idx = m_num_horizontal_grid - 1;  
    }

    if(vertical_grid_idx == m_num_vertical_grid) {
      vertical_grid_idx = m_num_vertical_grid - 1;  
    }

    return std::make_pair(vertical_grid_idx, horizontal_grid_idx);
  }

  void KPExtractor::SetGrids() {
    std::vector< std::vector<std::pair<cv::Point2f,cv::Point2f>>> vv_grid_points; 
    vv_grid_points.resize(m_num_vertical_grid);

    int offset_horizontal = (m_image_width % m_config.grid_width)/2;
    int offset_vertical = (m_image_height % m_config.grid_height)/2;

    for(unsigned int row = 0; row < m_num_vertical_grid; row++) {
      std::vector<std::pair<cv::Point2f,cv::Point2f>> v_grid_points_horizontal;
      v_grid_points_horizontal.reserve(m_num_horizontal_grid);

      for(unsigned int col = 0; col < m_num_horizontal_grid; col++) {
        cv::Point2f top_left(offset_horizontal+col*m_config.grid_width,
                             offset_vertical+row*m_config.grid_height);
        cv::Point2f bottom_right(offset_horizontal+(col+1)*m_config.grid_width,
                                 offset_vertical+(row+1)*m_config.grid_height);
        
        if(col == 0) {
          top_left.x = 0.0; 
        }
        else if(col == m_num_horizontal_grid-1) {
          bottom_right.x = m_image_width; 
        }

        if(row == 0) {
          top_left.y = 0.0; 
        }
        else if(row == m_num_vertical_grid-1) {
          bottom_right.y = m_image_height; 
        }
        v_grid_points_horizontal.push_back(std::make_pair(top_left, bottom_right));  
      }
      vv_grid_points.push_back(v_grid_points_horizontal);
    }

    m_vvpair_grid_corners = vv_grid_points;

    return;
  }

  std::vector< std::vector<std::pair<cv::Point2f, cv::Point2f>>> KPExtractor::GetGrids() {
    return m_vvpair_grid_corners;
  }
  
};
