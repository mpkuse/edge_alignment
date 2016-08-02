#pragma once

#include <iostream>
#include <string>
using namespace std;
#include <queue>
#include <boost/thread/mutex.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>


/// @class Thread safe implementation of a queue
template<typename Data>
class ConcurrentQueue
{
private:
    std::queue<Data> the_queue;
    mutable boost::mutex the_mutex;
    boost::condition_variable the_condition_variable;
    boost::atomic<int> len;
public:
    ConcurrentQueue()
    {
        len = 0;
    }

    void push(Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_queue.push(data);
        lock.unlock();
        the_condition_variable.notify_one();
        len++;
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_queue.empty();
    }

    bool try_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if(the_queue.empty())
        {
            return false;
        }

        popped_value=the_queue.front();
        the_queue.pop();
        len--;
        return true;
    }

    void wait_and_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        while(the_queue.empty())
        {
            the_condition_variable.wait(lock);
        }

        popped_value=the_queue.front();
        the_queue.pop();
        len--;
    }

    int getSize()
    {
//        boost::mutex::scoped_lock lock(the_mutex);
//        int len = the_queue.size();
////        lock.unlock();
//        the_condition_variable.notify_one();
        return len;
    }

};


string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}


void sOverlay(cv::Mat im, Eigen::MatrixXi mask)
{

    assert( im.data != NULL );
    assert( (im.rows == mask.rows()) && "Image and mask rows must match");
    assert( (im.cols == mask.cols()) && "Image and mask cols must match");
    assert( (im.channels() == 3) && "im must be 3 channel");

    cv::Mat xim = im.clone();

    for( int j=0 ; j<mask.cols() ; j++ )
    {
        for( int i=0 ; i<mask.rows() ; i++ )
        {
            if( mask(i,j) > 0 )
            {
                xim.at<cv::Vec3b>(i,j)[0] = 0;
                xim.at<cv::Vec3b>(i,j)[1] = 0;
                xim.at<cv::Vec3b>(i,j)[2] = 255;
            }
        }
    }

    cv::imshow( "marked_image", xim );
    cv::waitKey(30);
}
