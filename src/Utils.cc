#include "Utils.h"
#include "Frame.h"

#include <cmath>

namespace TS_SfM {
  cv::Mat ChoiseDescriptor(const Frame& f0, const Frame& f1,
                           const cv::Point3f& p, const cv::DMatch& match)
  {
    cv::Mat m_desc;

    const cv::Mat _p = (cv::Mat_<float>(4,1) << p.x, p.y, p.z, 1.0);
    const cv::Mat pose0 = f0.GetPose();
    const cv::Mat pose1 = f1.GetPose();

    const cv::Mat pt_on_0 = pose0 * _p/cv::norm(pose0 * _p);
    const cv::Mat pt_on_1 = pose1 * _p/cv::norm(pose1 * _p);
    const cv::Mat z = (cv::Mat_<float>(3,1) << 0.0, 0.0, 1.0);

    double theta_0 = acos(z.dot(pt_on_0)) * 180.0/M_PI;
    double theta_1 = acos(z.dot(pt_on_1)) * 180.0/M_PI;
    // std::cout << theta_0 << std::endl;
    // std::cout << theta_1 << std::endl;
    // std::cout << "==================" << std::endl;

    if(theta_0 < theta_1) {
      m_desc = f0.GetDescriptors().row(match.queryIdx);
    }
    else {
      m_desc = f1.GetDescriptors().row(match.trainIdx);
    }
  
   
    return m_desc;
  };
}
