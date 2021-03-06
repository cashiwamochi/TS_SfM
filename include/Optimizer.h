#pragma once

// #include <Eigen/Core>
// #include <Eigen/StdVector>
// #include <Eigen/Geometry>
// #include <iostream>
//
// #include <g2o/stuff/command_args.h>
// #include "g2o/core/batch_stats.h"
// #include "g2o/core/sparse_optimizer.h"
// #include "g2o/core/block_solver.h"
// #include "g2o/core/solver.h"
// #include "g2o/core/optimization_algorithm_levenberg.h"
// #include "g2o/core/base_vertex.h"
// #include "g2o/core/base_binary_edge.h"
// #include "g2o/solvers/dense/linear_solver_dense.h"
// #include "g2o/solvers/structure_only/structure_only_solver.h"
// #include "g2o/solvers/pcg/linear_solver_pcg.h"
//
// // #include "EXTERNAL/ceres/autodiff.h"
//
// // #if defined G2O_HAVE_CHOLMOD
// #include "g2o/solvers/cholmod/linear_solver_cholmod.h"
// // #elif defined G2O_HAVE_CSPARSE
// #include "g2o/solvers/csparse/linear_solver_csparse.h"
// // #endif
// #include "g2o/solvers/pcg/linear_solver_pcg.h"
// #include "g2o/core/linear_solver.h"
// #include "g2o/core/optimization_algorithm_factory.h"
// #include "g2o/types/slam3d/types_slam3d.h"
// #include "g2o/types/slam3d_addons/types_slam3d_addons.h"
// #include "g2o/stuff/macros.h"

#include <Eigen/StdVector>
#include <iostream>
#include <stdint.h>

#include <unordered_set>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"



namespace TS_SfM {
  class KeyFrame;
  class MapPoint;
  class Map;
  struct Camera;

   struct BAResult {
    std::vector<KeyFrame> v_keyframes;
    std::vector<MapPoint> v_mappoints;
    float repro_error;
   };

  class Optimizer {
      public:
        Optimizer(){};
        ~Optimizer(){};
        void Run();
        bool SetData();


      private:
    

  };

  BAResult BundleAdjustmentBeta(std::vector<std::reference_wrapper<KeyFrame>> v_keyframes,
                                std::vector<std::reference_wrapper<MapPoint>> v_mappoints, const Camera& cam);

};
