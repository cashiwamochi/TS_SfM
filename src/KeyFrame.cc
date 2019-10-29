#include "KeyFrame.h"
#include "Frame.h"

namespace TS_SfM {

  KeyFrame::KeyFrame(const Frame& f) 
  : m_id(f.m_id), m_m_image(f.GetImage()), m_v_kpts(f.GetKeyPoints()), m_m_descriptors(f.GetDescriptors()),
    m_vvv_grid_kpts(f.GetGridKeyPoints()), m_vvm_grid_descs(f.GetGridDescs()),
    m_vv_num_grid_kpts(f.GetGridKeyPointsNum()), m_num_assigned_kps(f.GetAssignedKeyPointsNum())
  {
  
  }



}; // namespace 
