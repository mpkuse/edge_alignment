#include <ros/ros.h>
#include <ctime>

#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/atomic.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lockfree/queue.hpp>

#include <ros/package.h>

#include <Eigen/Dense>
using namespace Eigen;
using namespace  std;


#include <ConcurrentQueue.h>
#include <SolveEA.h>

//color image queue
ConcurrentQueue<cv::Mat> colorQueue;


//depth image queue
ConcurrentQueue<cv::Mat> depthQueue;


// Color image call back
void imgRcvd( const sensor_msgs::ImageConstPtr& msg )
{
    try
    {
        cv::Mat im = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat im_resized;
        cv::resize( im, im_resized, cv::Size(), 0.5, 0.5 );

        colorQueue.push(im_resized);
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
        //depth mask nan
        cv::Mat mask = cv::Mat(depth != depth);
        depth.setTo( 0, mask );

        cv::Mat depth_resized;
        cv::resize( depth, depth_resized, cv::Size(), 0.5, 0.5 );

        depthQueue.push(depth_resized);

//        cv::imshow("view-depth", cv_bridge::toCvShare(msg, "")->image);
//        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void write_to_opencv_file( int nFrame, cv::Mat& im, cv::Mat& depth )
{
    char filename[500];

    sprintf( filename, "%s/data/%d.xml", ros::package::getPath("edge_alignment").c_str(), nFrame );
    ROS_INFO( "Write to %s", filename );
    cv::FileStorage fs( filename, cv::FileStorage::WRITE );
    fs << "im" << im ;
    fs << "depth" << depth;
    fs.release();
}

void queueConsumer( )
{
    cv::Mat im, depth;
    ros::Rate rate(30);
    int nFrame = 0;
    while( ros::ok() )
    {
        //TODO :
        // This synchronization is not perfect. Should ideally also push the timestamps to the queue and do
        // it based on time stamps of the pushed images
        cout << "size : "<< colorQueue.getSize() << "  " << depthQueue.getSize() << endl;
        if( colorQueue.getSize() < 1 || depthQueue.getSize() < 1 )
        {
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        bool pop_im_flag = colorQueue.try_pop(im);
        bool pop_depth_flag = depthQueue.try_pop(depth);



        cout << "im.type() : " << type2str( im.type() ) << endl;
        cout << "depth.type() : " << type2str( depth.type() ) << endl;
        double minVal, maxVal;
        cv::minMaxLoc( depth, &minVal, &maxVal );
        cout << "depth min : "<< minVal << endl;
        cout << "depth max : "<< maxVal << endl;

        // save to file
        write_to_opencv_file( nFrame, im, depth );

        cv::imshow( "im", im );
        cv::Mat falseColorsMap;
        applyColorMap(depth, falseColorsMap, cv::COLORMAP_AUTUMN);
        cv::imshow( "depth", depth );
        cv::waitKey(1);


        ros::spinOnce();
        rate.sleep();
        nFrame++;
    }
}

/*
int main( int argc, char ** argv )
{
    ros::init(argc, argv, "edge_alignment_node" );

    ros::NodeHandle nh;

    ros::Subscriber sub_depth = nh.subscribe( "/camera/depth/image", 10, depthRcvd );
    ros::Subscriber sub_img   = nh.subscribe( "/camera/rgb/image_color", 10, imgRcvd );

    boost::thread queueConsumerThread(&queueConsumer);
    queueConsumerThread.join();

}
*/

#define TIC(x) x = clock();
#define TOC(msg, x) cout << "ELAPSED TIME : "<< msg << float(clock() - x)/1000.0 << " mili-sec " << endl;

//Load 2 images from file
int main( int argc, char ** argv )
{
    cv::FileStorage fs;
    clock_t start;

    TIC( start );
    ROS_INFO( "start");
    //read reference image
    char filename[500];
    sprintf( filename, "%s/data/%d.xml", ros::package::getPath("edge_alignment").c_str(), 3 );
    fs.open( filename, cv::FileStorage::READ );
    cv::Mat ref_im, ref_depth;
    fs["im"] >> ref_im;
    fs["depth"] >> ref_depth;
    fs.release();
    TOC( "read ref in ", start );
    ROS_INFO( "end");


    //read now image
    sprintf( filename, "%s/data/%d.xml", ros::package::getPath("edge_alignment").c_str(), 8 );
    fs.open( filename, cv::FileStorage::READ );
    cv::Mat now_im, now_depth;
    fs["im"] >> now_im;
    fs["depth"] >> now_depth;


    cv::imshow( "ref_im", ref_im );
    cv::imshow( "now_im", now_im );
    cv::waitKey(0);

    SolveEA * ea = new SolveEA();
    TIC(start);
    ea->setRefFrame( ref_im, ref_depth );
    TOC("ref frame processed in ", start );

    TIC(start);
    ea->setNowFrame( now_im, now_depth );
    TOC("now frame processed in ", start );

    // Verify
    ea->_verify3dPts();

    // TODO.
    // use `now_dist_transform`, `list_edge_ref`, camera_intrinsics and set up a CERES problem
    //ea->_sampleCERESProblem();
    ea->setAsCERESProblem();




    cv::waitKey(0);

}
