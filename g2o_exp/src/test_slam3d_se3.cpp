#include <iostream>
#include <cstdio>

#include "g2o/core/jacobian_workspace.h"
#include "g2o/stuff/macros.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/edge_se3_prior.h"
#include "g2o/types/slam3d/se3quat.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include <tr1/unordered_map>

typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> >  SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearCSparseSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearCholmodSolver;
typedef g2o::LinearSolverPCG<SlamBlockSolver::PoseMatrixType> SlamLinearPCGSolver;
typedef g2o::LinearSolverDense<SlamBlockSolver::PoseMatrixType> SlamLinearDenseSolver;
typedef std::tr1::unordered_map<int, g2o::HyperGraph::Vertex*>     VertexIDMap;
typedef std::pair<int, g2o::HyperGraph::Vertex*> VertexIDPair;
typedef std::set<g2o::HyperGraph::Edge*> EdgeSet;

using namespace std;
using namespace g2o;
using namespace Eigen;

Isometry3D randomIsometry3d()
{
  Vector3D rotAxisAngle = Vector3D::Random();
  rotAxisAngle += Vector3D::Random();
  Eigen::AngleAxisd rotation(rotAxisAngle.norm(), rotAxisAngle.normalized());
  Isometry3D result = (Isometry3D)rotation.toRotationMatrix();
  result.translation() = Vector3D::Random();
  return result;
}

int main(int , char** )
{
  SparseOptimizer g;
  SlamLinearCholmodSolver * linearSolver = new SlamLinearCholmodSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver * solver = new SlamBlockSolver(linearSolver); 
  g2o::OptimizationAlgorithmLevenberg * algo = new g2o::OptimizationAlgorithmLevenberg(solver);
  g.setAlgorithm(algo);


    // add vertex
    VertexSE3 v0, v1, v2; 
    v0.setId(0); v1.setId(1); v2.setId(2); v0.setFixed(true);
    // v1.setEstimate(randomIsometry3d()); 
    // v2.setEstimate(randomIsometry3d());
    g.addVertex(&v0); g.addVertex(&v1); g.addVertex(&v2); 

    // add edge
    Eigen::Matrix<double,6,6,Eigen::ColMajor> information = Eigen::Matrix<double,6,6,Eigen::ColMajor>::Identity();
    EdgeSE3 e01, e12, e02; 
    e01.setVertex(0, &v0); e01.setVertex(1, &v1);  
    e12.setVertex(0, &v1); e12.setVertex(1, &v2); 
    e02.setVertex(0, &v0); e02.setVertex(1, &v2); 
    
    double update[6] = {1, 1, 1, 0.5, 0, 0}; 
    {
      Eigen::Map<Vector6d> dT(update);
      Isometry3D T = g2o::internal::fromVectorMQT(dT); 
      e01.setMeasurement(T); e12.setMeasurement(T); 
    }
    update[3] = 0.5; // update[4] = 0.1; 
    {
      Eigen::Map<Vector6d> dT(update);
      Isometry3D T = g2o::internal::fromVectorMQT(dT); 
      e02.setMeasurement(T); 
      Eigen::Matrix4d tmp = T.matrix();
      cout<<"e02: matrix: "<<endl<<tmp<<endl;
    }
    e01.setInformation(information); 
    e12.setInformation(information); 
    e02.setInformation(information); 
    
    g.addEdge(&e01); // g.addEdge(&e12); g.addEdge(&e02);

    g.setVerbose(true);
    g.initializeOptimization();
    g.optimize(20); 
  
    g.save("test_slam3d_se3.log");
    g.clear();
    return 1;

}
