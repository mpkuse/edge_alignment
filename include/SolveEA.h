#pragma once

#include <iostream>
#include <cmath>

// EIGEN - Linear Algebra
#include <Eigen/Dense>
#include <Eigen/Core>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

// Ceres - Nonlinear Solver
#include <ceres/ceres.h>
#include <ceres/loss_function.h>

#include <EAResidue.h>

using namespace std;
using namespace cv;
using namespace Eigen;

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::LossFunction;
using ceres::LossFunctionWrapper;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;



class SolveEA
{
public:
    SolveEA();
    void setRefFrame(const Mat &rgb, const Mat &depth);
    void setNowFrame(const Mat &rgb, const Mat &depth);

    void setAsCERESProblem();

    // Testing/Verify Functions
    void _verify3dPts();
    void _sampleCERESProblem();



private:
    Matrix3d K; ///< Camera intrinsics at the resolution of input
    double fx, fy, cx, cy; ///< same as `K`

    cv::Mat ref_im, ref_depth; ///< ref images
    cv::Mat now_im, now_depth; ///< now images

    cv::Mat ref_edge, now_edge; /// edges. note: edges of now are stored as (255-x).

    cv::Mat now_dist_transform;
    MatrixXd now_dist_transform_eig;
    cv::Mat now_dist_transform_display;

    MatrixXd list_edge_ref; ///< List of x,y,z at edge pixels of ref image


    // helpers
    void cordList_2_mask(Eigen::MatrixXd &list, Eigen::MatrixXi &mask);
    void cordList_2_mask(MatrixXd &list, cv::Mat &mask);
    void sOverlay(const Mat &src, const Mat &mask, cv::Mat& dst, const Vec3b &color);
};

