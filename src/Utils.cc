#include "Utils.h"
#include "Frame.h"

#include <cmath>

namespace TS_SfM {
  cv::Mat ChooseDescriptor(const Frame& f0, const Frame& f1,
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

  bool CheckIndex(const int& src_frame_idx, const int& dst_frame_idx,
      const std::vector<bool>& vb_initialized, const int length){

    if(src_frame_idx < 0 || dst_frame_idx > (int)length-1) {
      // Initialization is finised.
      return true;
    }

    if(!vb_initialized[src_frame_idx] && !vb_initialized[dst_frame_idx]) {
      // This case should not happend.
      std::cout << "[Warning] Wrong case, need to check !\n";
      return true;
    }

    return false;
  }

  Eigen::Vector2d ProjectToImage(const cv::Mat& K, const cv::Mat& cTw, const cv::Point3f& pt) {
    Eigen::Vector2d projected_point;
    
    cv::Mat _pt_on_map = (cv::Mat_<double>(4,1) << pt.x, pt.y, pt.z, 1.0);
    cv::Mat _pt = K * cTw.rowRange(0,3) * _pt_on_map;

    projected_point(0) = _pt_on_map.at<double>(0)/_pt_on_map.at<double>(3);
    projected_point(1) = _pt_on_map.at<double>(1)/_pt_on_map.at<double>(3);
    projected_point(2) = _pt_on_map.at<double>(2)/_pt_on_map.at<double>(3);

    return projected_point;
  }

  cv::Mat Inverse3x4(const cv::Mat& _pose) {
    cv::Mat result = cv::Mat::zeros(3,4,_pose.type());
    
    result.rowRange(0,3).colRange(0,3) = _pose.rowRange(0,3).colRange(0,3).t();
    result.rowRange(0,3).col(3) = -1.0 * _pose.rowRange(0,3).colRange(0,3).t() * _pose.rowRange(0,3).col(3);

    return result;
  }

  cv::Mat AppendRow(const cv::Mat& _pose) {
    cv::Mat pose = cv::Mat::eye(4,4,_pose.type());
    _pose.copyTo(pose.rowRange(0,3).colRange(0,4));
    return pose;
  }

}
