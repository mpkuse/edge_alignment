#include <iostream>

// Eigen - Linear Algebra
#include <Eigen/Core>
#include <Eigen/Dense>


#include <ceres/ceres.h>

using namespace Eigen;
using namespace std;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::LossFunctionWrapper;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;


class Residue {
public:
  Residue( MatrixXd& _a )
  {
      A = _a;
  }

  template <typename T>
  bool operator()(const T* const x, T* residual) const
//  bool operator()(const vector<T*> m, T* residual) const
  {
      for( int j=0 ; j<34 ; j++ )
      {
          residual[j] = T(0.0);
          for( int i=0 ; i<27 ; i++ )
              residual[j] += A(j,i) * x[i];
          residual[j] -= T(0.1);
      }

//    residual[0] = T(a(0)) - exp(m[0] * T(a(0)) );
    return true;
  }
 private:
  MatrixXd A;

};
