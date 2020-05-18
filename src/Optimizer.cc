#include "Optimizer.h"
#include "ConfigLoader.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Solver.h"
#include "Utils.h"

#include <unordered_map>

using namespace Eigen;

namespace TS_SfM {


void Optimizer::Run() {

  return;
}

bool Optimizer::SetData() {

  return false;
}


// Firstly i will implement a simple optimizer for verification.
  BAResult BundleAdjustmentBeta(std::vector<std::reference_wrapper<KeyFrame>> v_keyframes,
                                std::vector<std::reference_wrapper<MapPoint>> v_mappoints, const Camera& cam)
{
  BAResult result;
  double error_prior_optimization = 0.0;
  double error_after_optimization = 0.0;
  int total_obs_num = 0;
  const int center_frame_idx = static_cast<int>(v_keyframes.size() - 1)/2;
  cv::Mat K = (cv::Mat_<double>(3,3) << (double)cam.f_fx, 0.0, (double)cam.f_cx,
                                        0.0, (double)cam.f_fy, (double)cam.f_cy,
                                        0.0, 0.0, 1.0);

  /*Optimizer Setup ==================*/
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(true);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;

  linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
  );
  optimizer.setAlgorithm(solver);
  /*================== Optimizer Setup*/

  /*Problem Setup ======================*/
  // Set vertex to pose
  std::unordered_map<int, int> keyframeIdToVertexId;
  int v_camera_id = 0;
  for(auto keyframe : v_keyframes) {
    if(!keyframe.get().IsActivated()) {
      continue;
    }

    g2o::VertexSE3Expmap* vertex_se3 = new g2o::VertexSE3Expmap();
    vertex_se3->setId(v_camera_id);
    if(keyframe.get().m_id == center_frame_idx-1 || keyframe.get().m_id == center_frame_idx) {
      vertex_se3->setFixed(true);
    }

    Eigen::Matrix3d rot;
    Eigen::Vector3d t;
    cv::Mat _t = keyframe.get().GetPose();
    cv2eigen(keyframe.get().GetPoseTrans(), t);
    cv2eigen(keyframe.get().GetPoseRot(), rot);
    Eigen::Quaterniond q(rot);

    g2o::SE3Quat pose(q,t);

    vertex_se3->setEstimate(pose);
    optimizer.addVertex(vertex_se3);
    keyframeIdToVertexId[keyframe.get().m_id] = v_camera_id;
    ++v_camera_id;
  }

  int v_point_id = v_camera_id;
  int point_num = 0;
  double sum_diff2 = 0;

  // Set vertex to point 
  for (auto mappoint : v_mappoints){
    if(!mappoint.get().IsActivated() || mappoint.get().GetObsNum() < 2) {
      continue;
    }

    g2o::VertexSBAPointXYZ* v_p = new g2o::VertexSBAPointXYZ();
    v_p->setId(v_point_id);
    v_p->setMarginalized(true);
    v_p->setEstimate(Eigen::Vector3d(mappoint.get().GetPosition().x,
                                     mappoint.get().GetPosition().y,
                                     mappoint.get().GetPosition().z));

    optimizer.addVertex(v_p);
    // add edges
    for(int obs_idx = 0; obs_idx < (int)mappoint.get().GetObsNum(); ++obs_idx) {
      MatchInfo m = mappoint.get().GetMatchInfo(obs_idx);
      auto pt = v_keyframes[m.frame_id].get().GetObs(m.kpt_id);
      Eigen::Vector2d obs(pt.x, pt.y);

      g2o::EdgeSE3ProjectXYZ* edge = new g2o::EdgeSE3ProjectXYZ();

      edge->setVertex(0,
         dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(v_point_id)));
      edge->setVertex(1,
         dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(keyframeIdToVertexId[m.frame_id])));

      edge->setMeasurement(obs);
      edge->setInformation(Eigen::Matrix2d::Identity());

      g2o::RobustKernelHuber* r_k = new g2o::RobustKernelHuber;
      edge->setRobustKernel(r_k);
      r_k->setDelta(4.0);

      edge->fx = (double)cam.f_fx;
      edge->fy = (double)cam.f_fy;
      edge->cx = (double)cam.f_cx;
      edge->cy = (double)cam.f_cy;

      optimizer.addEdge(edge);
    }
    ++v_point_id;
  
  }
  /*====================== Problem Setup*/
  /* Optimization ========================*/
  optimizer.initializeOptimization();
  optimizer.optimize(50);
  /* ======================== Optimization*/
  /* Retrieve Desired Params ====================== */
  std::vector<Eigen::Vector3d> estPoints3d;
  std::vector<Eigen::Isometry3d> estPoses;

  for (int i = 0; i < v_camera_id; i++)
  {
    g2o::VertexSE3Expmap* VSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(i));
    g2o::SE3Quat CorrectedSiw = VSE3->estimate();
    Eigen::Isometry3d pose(Eigen::Isometry3d::Identity());
    pose.prerotate(CorrectedSiw.rotation());
    pose.pretranslate(CorrectedSiw.translation());
    estPoses.push_back(pose); //w2c
  }
  for (int i = v_camera_id; i < v_point_id; i++) {
    g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i));
    estPoints3d.push_back(vPoint->estimate());
  }

  /* ====================== Retrieve Desired Params */
  return result;
}


};
