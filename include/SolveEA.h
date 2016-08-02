#pragma once

#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>



using namespace std;
using namespace cv;
using namespace Eigen;



class SolveEA
{
public:
    SolveEA();
    void setRefFrame(const Mat &rgb, const Mat &depth);
    void setNowFrame(const Mat &rgb, const Mat &depth);


private:
    cv::Mat ref_im, ref_depth; ///< ref images
    cv::Mat now_im, now_depth; ///< now images

    cv::Mat ref_edge, now_edge; /// edges. note: edges of now are stored as (255-x).

    cv::Mat now_dist_transform;
    MatrixXd now_dist_transform_eig;

    MatrixXd list_edge_ref; ///< List of x,y,z at edge pixels of ref image
};

