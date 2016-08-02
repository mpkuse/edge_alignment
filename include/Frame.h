/**
 * @file   Frame.h
 * @Author Manohar Kuse <mpkuse@connect.ust.hk>
 * @date   4th Jan, 2016
 * @brief  Holds the processed data from a time `t`. Data includes, original frames, rectified, 3d points, edge map
 *
 */

#ifndef __FRAME__t__H
#define __FRAME__t__H

// Std headers
#include <cstdlib>
#include <queue>
#include <iostream>
#include <vector>

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// Linear Algebra
#include <Eigen/Core>
#include <Eigen/Dense>

// OpenCV Headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
//#include <opencv2/gpu/gpu.hpp>
#include <opencv2/calib3d.hpp>

using namespace Eigen;
using namespace std;





/// @class  Holds the processed data from a time `t`. Data includes, original frames, rectified, 3d points, edge map etc.
class Frame
{
public:
    Frame();
};

#endif //__FRAME__t__H
