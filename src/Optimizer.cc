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
  BAResult BundleAdjustmentBeta(std::vector<KeyFrame> v_keyframes,
                                std::vector<MapPoint> v_mappoints, const Camera& cam)
{
  BAResult result;
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
  int vertex_id = 0;
  for(KeyFrame keyframe : v_keyframes) {
    if(!keyframe.IsActivated()) {
      continue;
    }

    g2o::VertexSE3Expmap* vertex_se3 = new g2o::VertexSE3Expmap();
    vertex_se3->setId(vertex_id);
    if(keyframe.m_id == center_frame_idx-1 || keyframe.m_id == center_frame_idx) {
      vertex_se3->setFixed(true);
    }

    Eigen::Matrix3d rot;
    Eigen::Vector3d t;
    cv::Mat _t = keyframe.GetPose();
    cv2eigen(keyframe.GetPoseTrans(), t);
    cv2eigen(keyframe.GetPoseRot(), rot);
    Eigen::Quaterniond q(rot);

    g2o::SE3Quat pose(q,t);

    vertex_se3->setEstimate(pose);
    optimizer.addVertex(vertex_se3);
    keyframeIdToVertexId[keyframe.m_id] = vertex_id;
    ++vertex_id;
  }

  int point_id = vertex_id;
  int point_num = 0;
  double sum_diff2 = 0;

#if 1

  for (MapPoint mappoint : v_mappoints){
    if(!mappoint.IsActivated()) {
      continue;
    }

    g2o::VertexSBAPointXYZ* v_p = new g2o::VertexSBAPointXYZ();
    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->setEstimate(Eigen::Vector3d(mappoint.GetPosition().x,
                                     mappoint.GetPosition().y,
                                     mappoint.GetPosition().z));

    if (mappoint.GetObsNum() >= 2){
      optimizer.addVertex(v_p);
      for(int obs_idx = 0; obs_idx < (int)mappoint.GetObsNum(); ++obs_idx) {
        MatchInfo m = mappoint.GetMatchInfo(obs_idx);
        auto pt = v_keyframes[m.frame_id].GetObs(m.kpt_id);
        Eigen::Vector2d obs(pt.x, pt.y);

        g2o::EdgeSE3ProjectXYZ* edge = new g2o::EdgeSE3ProjectXYZ();

        edge->setVertex(0,
           dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(point_id)));
        edge->setVertex(1,
           dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(keyframeIdToVertexId[m.kpt_id])));

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
        ++point_id;
      }
    }
    else {
      std::cout << "[WARNING::Optimizer] something wrong in mapoint match_info" << std::endl;
     }
  
  }
#endif
  /*====================== Problem Setup*/


  /* Optimization ========================*/
  optimizer.initializeOptimization();
  optimizer.optimize(50);
  /* ======================== Optimization*/
  return result;
}


};













#if 0
class Sample {
public:
  static int uniform(int from, int to);
  static double uniform();
  static double gaussian(double sigma);
};

static double uniform_rand(double lowerBndr, double upperBndr){
  return lowerBndr + ((double) std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

static double gauss_rand(double mean, double sigma){
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

int Sample::uniform(int from, int to){
  return static_cast<int>(uniform_rand(from, to));
}

double Sample::uniform(){
  return uniform_rand(0., 1.);
}

double Sample::gaussian(double sigma){
  return gauss_rand(0., sigma);
}

int test(int argc, const char* argv[]){
  if (argc<2)
  {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] [STRUCTURE_ONLY] [DENSE]" << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)" << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)" << endl;
    cout << "STRUCTURE_ONLY: performe structure-only BA to get better point initializations (0 or 1; default: 0==false)" << endl;
    cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
    cout << endl;
    cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to 1==true." << endl;
    cout << endl;
    exit(0);
  }

  double PIXEL_NOISE = atof(argv[1]);
  double OUTLIER_RATIO = 0.0;

  if (argc>2)  {
    OUTLIER_RATIO = atof(argv[2]);
  }

  bool ROBUST_KERNEL = false;
  if (argc>3){
    ROBUST_KERNEL = atoi(argv[3]) != 0;
  }
  bool STRUCTURE_ONLY = false;
  if (argc>4){
    STRUCTURE_ONLY = atoi(argv[4]) != 0;
  }

  bool DENSE = false;
  if (argc>5){
    DENSE = atoi(argv[5]) != 0;
  }

  cout << "PIXEL_NOISE: " <<  PIXEL_NOISE << endl;
  cout << "OUTLIER_RATIO: " << OUTLIER_RATIO<<  endl;
  cout << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl;
  cout << "STRUCTURE_ONLY: " << STRUCTURE_ONLY<< endl;
  cout << "DENSE: "<<  DENSE << endl;



  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
  if (DENSE) {
    linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
  } else {
    linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
  }

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
  );
  optimizer.setAlgorithm(solver);


  vector<Vector3d> true_points;
  for (size_t i=0;i<500; ++i)
  {
    true_points.push_back(Vector3d((Sample::uniform()-0.5)*3,
                                   Sample::uniform()-0.5,
                                   Sample::uniform()+3));
  }

  double focal_length= 1000.;
  Vector2d principal_point(320., 240.);

  vector<g2o::SE3Quat,
      aligned_allocator<g2o::SE3Quat> > true_poses;
  g2o::CameraParameters * cam_params
      = new g2o::CameraParameters (focal_length, principal_point, 0.);
  cam_params->setId(0);

  if (!optimizer.addParameter(cam_params)) {
    assert(false);
  }

  int vertex_id = 0;
  for (size_t i=0; i<15; ++i) {
    Vector3d trans(i*0.04-1.,0,0);

    Eigen:: Quaterniond q;
    q.setIdentity();
    g2o::SE3Quat pose(q,trans);
    g2o::VertexSE3Expmap * v_se3
        = new g2o::VertexSE3Expmap();
    v_se3->setId(vertex_id);
    if (i<2){
      v_se3->setFixed(true);
    }
    v_se3->setEstimate(pose);
    optimizer.addVertex(v_se3);
    true_poses.push_back(pose);
    vertex_id++;
  }
  int point_id=vertex_id;
  int point_num = 0;
  double sum_diff2 = 0;

  cout << endl;
  unordered_map<int,int> pointid_2_trueid;
  unordered_set<int> inliers;

  for (size_t i=0; i<true_points.size(); ++i){
    g2o::VertexSBAPointXYZ * v_p
        = new g2o::VertexSBAPointXYZ();
    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->setEstimate(true_points.at(i)
                     + Vector3d(Sample::gaussian(1),
                                Sample::gaussian(1),
                                Sample::gaussian(1)));
    int num_obs = 0;
    for (size_t j=0; j<true_poses.size(); ++j){
      Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
      if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480){
        ++num_obs;
      }
    }
    if (num_obs>=2){
      optimizer.addVertex(v_p);
      bool inlier = true;
      for (size_t j=0; j<true_poses.size(); ++j){
        Vector2d z
            = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));

        if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480){
          double sam = Sample::uniform();
          if (sam<OUTLIER_RATIO){
            z = Vector2d(Sample::uniform(0,640),
                         Sample::uniform(0,480));
            inlier= false;
          }
          z += Vector2d(Sample::gaussian(PIXEL_NOISE),
                        Sample::gaussian(PIXEL_NOISE));
          g2o::EdgeProjectXYZ2UV * e
              = new g2o::EdgeProjectXYZ2UV();
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                       (optimizer.vertices().find(j)->second));
          e->setMeasurement(z);
          e->information() = Matrix2d::Identity();
          if (ROBUST_KERNEL) {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
          }
          e->setParameterId(0, 0);
          optimizer.addEdge(e);
        }
      }

      if (inlier){
        inliers.insert(point_id);
        Vector3d diff = v_p->estimate() - true_points[i];

        sum_diff2 += diff.dot(diff);
      }
      pointid_2_trueid.insert(make_pair(point_id,i));
      ++point_id;
      ++point_num;
    }
  }
  cout << endl;
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  if (STRUCTURE_ONLY){
    g2o::StructureOnlySolver<3> structure_only_ba;
    cout << "Performing structure-only BA:"   << endl;
    g2o::OptimizableGraph::VertexContainer points;
    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
      g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
      if (v->dimension() == 3)
        points.push_back(v);
    }
    structure_only_ba.calc(points, 10);
  }
  //optimizer.save("test.g2o");
  cout << endl;
  cout << "Performing full BA:" << endl;
  optimizer.optimize(10);
  cout << endl;
  cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << endl;
  point_num = 0;
  sum_diff2 = 0;
  for (unordered_map<int,int>::iterator it=pointid_2_trueid.begin();
       it!=pointid_2_trueid.end(); ++it){
    g2o::HyperGraph::VertexIDMap::iterator v_it
        = optimizer.vertices().find(it->first);
    if (v_it==optimizer.vertices().end()){
      cerr << "Vertex " << it->first << " not in graph!" << endl;
      exit(-1);
    }
    g2o::VertexSBAPointXYZ * v_p
        = dynamic_cast< g2o::VertexSBAPointXYZ * > (v_it->second);
    if (v_p==0){
      cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
      exit(-1);
    }
    Vector3d diff = v_p->estimate()-true_points[it->second];
    if (inliers.find(it->first)==inliers.end())
      continue;
    sum_diff2 += diff.dot(diff);
    ++point_num;
  }
  cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << endl;
  cout << endl;
}
#endif
