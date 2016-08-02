#include <SolveEA.h>

#define DEBUG_PRINT( x ) cout << x;

SolveEA::SolveEA()
{
    ref_im.data = NULL;
    ref_depth.data = NULL;
    now_im.data = NULL;
    now_depth.data = NULL;
}

///
/// depth is CV_32F
///   Computes the matrix `list_edge_ref` (3xN)
void SolveEA::setRefFrame(const Mat &rgb, const Mat &depth)
{
    //
    // Allocate if null, else copy
    if( ref_im.data == NULL )
    {
        DEBUG_PRINT( "setRefFrame : clone\n" );
        ref_im = rgb.clone();
        ref_depth = depth.clone();

    }
    else
    {
        DEBUG_PRINT( "setRefFrame : reuse memory\n" );
        rgb.copyTo(ref_im);
        depth.copyTo(ref_depth);
    }
    cv::Canny( ref_im, ref_edge, 150, 100, 3, true );




    // TODO
    // make a 3xN Matrix of 3d points in edges of reference frame. N is #of edge pixels
    int N = cv::countNonZero( ref_edge );
    cout << "# of non-zero elements "<< N << "\n";
    list_edge_ref = MatrixXd::Zero(3,N); // list of edges (X,Y,Z)


    // TODO : save this as an array
    // camera intrinsic
    double fx, fy, cx, cy;
    // K : 525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0
    fx = .5* 525.0;
    fy = .5* 525.0;
    cx = .5* 319.5;
    cy = .5* 239.5;

    // loop thru edge pixels
    int c=0;
    for( int yy=0 ; yy<ref_im.rows ; yy++ )
    {
        for( int xx=0 ; xx<ref_im.cols ; xx++ )
        {
            if( ref_edge.at<uchar>(yy,xx) > 0 )
            {
                float Z = ref_depth.at<float>(yy,xx);
                float X = Z * (xx-cx) / fx;
                float Y = Z * (yy-cy) / fy;

                list_edge_ref(0,c) = X;
                list_edge_ref(1,c) = Y;
                list_edge_ref(2,c) = Z;
                c++;
            }
        }
    }
    cout << "c ::: "<< c << endl;
}

/// depth is CV_32F
///     Computes distance transform of now frame `now_dist_transform_eig` or `now_dist_transform`
void SolveEA::setNowFrame(const Mat &rgb, const Mat &depth)
{
    //
    // Allocate if null, else copy
    if( now_im.data == NULL )
    {
        DEBUG_PRINT( "setNowFrame : clone\n" );
        now_im = rgb.clone();
        now_depth = depth.clone();
    }
    else
    {
        DEBUG_PRINT( "setNowFrame : reuse memory\n" );
        rgb.copyTo(now_im);
        depth.copyTo(now_depth);
    }
    cv::Canny( now_im, now_edge, 150, 100, 3, true );


    //
    // make distance transform image of now frame
    now_edge = 255 - now_edge;
    cv::distanceTransform( now_edge, now_dist_transform,  CV_DIST_L2, CV_DIST_MASK_PRECISE );
    cv::normalize(now_dist_transform, now_dist_transform, 0.0, 255.0, cv::NORM_MINMAX);
    cv::cv2eigen( now_dist_transform, now_dist_transform_eig );

//    cv::Mat dist_transform_display;
//    now_dist_transform.convertTo(dist_transform_display, CV_8UC1);
//    cv::imshow( "dist_trans", dist_transform_display );
//    cv::imshow( "now_edge", now_edge );
//    cout << now_edge << endl;
//    cv::waitKey(0);

}
