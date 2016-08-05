/**
  @file     EAResidue.h
  @Author   Manohar Kuse <mpkuse@connect.ust.hk>
  @date     5th Aug, 2016
  @brief    Custom ceres residue function for EA (edge alignment). See paper
            Kuse M., Shen S. Robust Camera Motion Estimation using Direct Edge Alignment and
            Sub-gradient Method. International Conference on Robotics and Automation (ICRA-2016),
            Stockholm, Sweden
  */
#include <iostream>

// Eigen - Linear Algebra
#include <Eigen/Core>
#include <Eigen/Dense>


#include <ceres/ceres.h>
#include <ceres/loss_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/rotation.h>

#include <ceres/cubic_interpolation.h>


using namespace Eigen;
using namespace std;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::LossFunctionWrapper;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::LocalParameterization;
using namespace ceres;



// Solving sample problem minimize_x || Ax||_2
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


class EAResidue {
public:
    /// Set the constants of the problem
    /// @param list_edge_ref : a list of 3d points in reference image 3xN
    /// @param now_dist_transform_eig : distance transform of now frame as an image (row x cols)
    //EAResidue( MatrixXd& __list_edge_ref, MatrixXd& __now_dist_transform_eig, Matrix3d& K )
    EAResidue( double lx, double ly, double lz,
               BiCubicInterpolator<Grid2D<double,2> >& interpolated_a, Matrix3d& K ) : lx(lx), ly(ly), lz(lz), cost(interpolated_a)
    {

        fx = ( K(0,0) );
        fy = ( K(1,1) );
        cx = ( K(0,2) );
        cy = ( K(1,2) );
    }


    template <typename T>
    bool operator()(const T* const Q, const T* const t, T* residual) const
    {
        T R[9];
        ceres::QuaternionToRotation(Q, R); //use `ceres::QuaternionRotatePoint`

//        for( int i =0 ; i<list_edge_ref.cols() ; i++ )
        //{
            T _x = T(lx);
            T _y = T(ly);
            T _z = T(lz);

            // xd = R*x + t
            T _xd = t[0] + R[0] * _x + R[3] * _y + R[6] * _z;
            T _yd = t[1] + R[1] * _x + R[4] * _y + R[7] * _z;
            T _zd = t[2] + R[2] * _x + R[5] * _y + R[8] * _z;

            // de-homegenous and project
            T _u = T(fx) * _xd / (_zd+.001) + T(cx);
            T _v = T(fy) * _yd / (_zd+.001) + T(cy);

//            double readVal = now_dist_transform_eig( _v, _u);
//            residual[i] = T( now_dist_transform_eig( (int)_v, (int)_u) );
            cost.Evaluate(_u, _v, &residual[0] );


//            residual[i] = R[0] * _x + R[3] * _y + R[6] * _z;
//            residual[i] = R[1] * _x + R[4] * _y + R[7] * _z;
//            residual[i] = R[2] * _x + R[5] * _y + R[8] * _z;
        //}

        return true;
    }

private:
    //MatrixXd list_edge_ref;
    double lx, ly, lz;
//    MatrixXd now_dist_transform_eig;
    double fx, fy, cx, cy;
    const BiCubicInterpolator< Grid2D<double,2> >& cost;
};
