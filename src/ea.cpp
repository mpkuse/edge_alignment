#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/atomic.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lockfree/queue.hpp>

#include <Eigen/Dense>
using namespace Eigen;
using namespace  std;


#include "ConcurrentQueue.h"

//color image queue
ConcurrentQueue<cv::Mat> colorQueue;
ConcurrentQueue<int> d;


//depth image queue
ConcurrentQueue<cv::Mat> depthQueue;


// Color image call back
void imgRcvd( const sensor_msgs::ImageConstPtr& msg )
{
    try
    {
        cv::Mat im = cv_bridge::toCvCopy(msg, "bgr8")->image;
        colorQueue.push(im);
//        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
//        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


//depth image call back
void depthRcvd( const sensor_msgs::ImageConstPtr& msg )
{
    try
    {
        cv::Mat depth = cv_bridge::toCvCopy(msg, "")->image;
        depthQueue.push(depth);
//        cv::imshow("view-depth", cv_bridge::toCvShare(msg, "")->image);
//        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void queueConsumer( )
{
    cv::Mat im, depth;
    ros::Rate rate(30);
    while( ros::ok() )
    {
        cout << "size : "<< colorQueue.getSize() << "  " << depthQueue.getSize() << endl;
        if( colorQueue.getSize() < 1 || depthQueue.getSize() < 1 )
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        cout << "here\n";
        bool pop_im_flag = colorQueue.try_pop(im);
        bool pop_depth_flag = depthQueue.try_pop(depth);
        cout << "here2\n";


        ros::spinOnce();
        rate.sleep();
    }
}


int main( int argc, char ** argv )
{
    ros::init(argc, argv, "edge_alignment_node" );

    ros::NodeHandle nh;

    ros::Subscriber sub_depth = nh.subscribe( "/camera/depth/image", 10, depthRcvd );
    ros::Subscriber sub_img   = nh.subscribe( "/camera/rgb/image_color", 10, imgRcvd );

    boost::thread queueConsumerThread(&queueConsumer);
    queueConsumerThread.join();


}
