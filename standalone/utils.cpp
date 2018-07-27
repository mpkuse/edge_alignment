#include "utils.h"



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

string mat_info( const cv::Mat& im )
{
    double min, max;
    cv::minMaxLoc(im, &min, &max);

    string s = "rows: "+to_string( im.rows )+"; cols: "+to_string( im.cols )+"; type:"+type2str( im.type() );
    s +=  "; min="+to_string(min)+"; max="+to_string(max);
    return s;
}


// #define get_distance_transform_debug(msg) msg
#define get_distance_transform_debug(msg)
void get_distance_transform( const cv::Mat& input, cv::Mat& out_distance_transform )
{
    // Thresholds that influcence this:
    // a. Gaussian Blur size
    // b. Threshold for the gradient map. How you compute gradient also matter. here i m using Laplacian operator. Results will differ with Sobel for example.
    // c. Window size for median blur
    // d. Params for distance transform computation.

    //
    // Edge Map
    //
    cv::Mat _blur, _gray;
    cv::GaussianBlur( input, _blur, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
    cv::cvtColor( _blur, _gray, CV_RGB2GRAY );

    cv::Mat _laplacian, _laplacian_8uc1;
    cv::Laplacian( _gray, _laplacian, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( _laplacian, _laplacian_8uc1 );
    get_distance_transform_debug( cv::imshow( "_laplacian_8uc1", _laplacian_8uc1) );

    //
    // Threshold gradients
    // TODO - use cv::Threshold
    cv::Mat B = cv::Mat::ones( _laplacian.rows, _laplacian.cols, CV_8UC1 ) * 255;
    for( int v=0 ; v<_laplacian.rows ; v++ )
    {
        for( int u=0 ; u<_laplacian.cols ; u++ )
        {
            if( _laplacian_8uc1.at<uchar>(v,u) > 35 )
            {
                B.at<uchar>(v,u) = 0;
            }
        }
    }

    //
    // Suppress noise with median filter
    cv::Mat B_filtered;
    medianBlur( B, B_filtered, 3 );
    get_distance_transform_debug( cv::imshow( "edge map", B_filtered ) );

    //
    // Distance Transform
    //
    cv::Mat dist;
    distanceTransform(B_filtered, dist, cv::DIST_L2, 3);
    normalize(dist, dist, 0, 1., cv::NORM_MINMAX);
    get_distance_transform_debug( imshow("Distance Transform Image", dist) );

    out_distance_transform = dist;

}


void get_aX( const cv::Mat& imA, const cv::Mat& imA_depth, const Eigen::Matrix3d& K, Eigen::MatrixXd& a_X )
{
    double factor=5000.; // as stated on TUM website: 5000 corresponds to 1m, 10000 corresponds to 2m and so on.
    double fx = K(0,0), fy=K(1,1), cx=K(0,2), cy=K(1,2);


    //
    // Image Gradient
    //
    cv::Mat imA_blur, imA_gray;
    cv::GaussianBlur( imA, imA_blur, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
    cv::cvtColor( imA_blur, imA_gray, CV_RGB2GRAY );

    cv::Mat imA_laplacian, imA_laplacian_8uc1;
    cv::Laplacian( imA_gray, imA_laplacian, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( imA_laplacian, imA_laplacian_8uc1 );
    cv::imshow( "imA_laplacian_8uc1",imA_laplacian_8uc1);


    int npixels = imA.rows*imA.cols;
    Eigen::MatrixXd all_aX = Eigen::MatrixXd( 4, npixels ); // stores all 3d co-ordinates
    Eigen::VectorXd all_grad = Eigen::VectorXd( npixels ); // stores gradients at those points
    Eigen::MatrixXd all_aU = Eigen::MatrixXd( 3, npixels );  // images point co-ordinates

    //
    // (u,v) --> (X,Y,Z)
    //
    int c = 0 ;
    for( int v=0 ; v<imA.rows ; v++ )
    {
        for( int u=0 ; u<imA.cols ; u++ )
        {
            double Z = double( imA_depth.at<ushort>( v, u ) )/ factor ;
            double X = (u - cx) * Z / fx;
            double Y = (v - cy) * Z / fy;
            // cout << "X,Y,Z" << X << "," << Y << "," << Z << endl;

            all_aX( 0, c ) = X;
            all_aX( 1, c ) = Y;
            all_aX( 2, c ) = Z;
            all_aX( 3, c ) = 1.;

            all_grad( c ) = imA_laplacian_8uc1.at<uchar>(v, u );

            all_aU( 0, c ) = u;
            all_aU( 1, c ) = v;
            all_aU( 2, c ) = 1.0;
            c++;

        }
    }


    //
    // Filter - Only keep 3d points with high gradient. Z=0 means invalid depth and are to be ignored
    int n = 0;
    double threshold = 35;
    for( int i=0 ; i<all_grad.size() ; i++ ) {
        if( all_grad(i) > threshold && all_aX(2,i) > 0 ) n++;
    }
    cout << n << " pts out of " << npixels << " have a large gradient. Used threshold=" << threshold << "\n";

    a_X = Eigen::MatrixXd::Zero( 4, n );
    int k=0;
    for( int i=0 ; i<all_grad.size() ; i++ )
    {
        if( all_grad(i) > threshold && all_aX(2,i) > 0 )
        {
            a_X( 0, k ) = all_aX( 0, i );
            a_X( 1, k ) = all_aX( 1, i );
            a_X( 2, k ) = all_aX( 2, i );
            a_X( 3, k ) = all_aX( 3, i );
            k++;
        }
    }
}


// Reprojects the 3D points a_X (in frame-of-ref of imA) using the transformation b_T_a (pose of a in frame-of-ref of b).
// K : Camera intrinsic. b_u is the output. as of now the distortion params are ignored.
void reproject( const Eigen::MatrixXd& a_X, const Eigen::Matrix4d& b_T_a, const Eigen::Matrix3d& K, Eigen::MatrixXd& b_u )
{
    Eigen::MatrixXd b_X = b_T_a * a_X;

    Eigen::MatrixXd unvn = Eigen::MatrixXd( 3, b_X.cols() );
    for( int i=0 ; i<b_X.cols() ; i++ )
    {
        unvn(0,i) = b_X(0,i) / b_X(2,i);
        unvn(1,i) = b_X(1,i) / b_X(2,i);
        unvn(2,i) = 1.0;
    }

    b_u = K * unvn;
}

void s_overlay( const cv::Mat& im, const Eigen::MatrixXd& uv, const char * win_name )
{
    assert( uv.rows() == 3 );
    assert( im.rows > 0 && im.cols > 0 && im.channels() == 3 );

    cv::Mat im_out = im.clone();
    for( int i=0 ; i<uv.cols() ; i++ )
    {
        cv::Vec3b color = cv::Vec3d(0,0,255);
        im_out.at<cv::Vec3b>( uv(1,i), uv(0,i) ) = color;
    }

    cv::imshow( win_name, im_out );
}
