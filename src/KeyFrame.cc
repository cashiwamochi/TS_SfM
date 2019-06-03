#include "KeyFrame.h"
#include "Frame.h"

namespace TS_SfM {

  KeyFrame::KeyFrame(const Frame& f) 
  : m_m_image(f.GetImage()), m_id(f.m_id)
  {
  
  }



}; // namespace 
