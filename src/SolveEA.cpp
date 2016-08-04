#include <SolveEA.h>

#define DEBUG_PRINT( x ) cout << x;

SolveEA::SolveEA()
{
    ref_im.data = NULL;
    ref_depth.data = NULL;
    now_im.data = NULL;
    now_depth.data = NULL;

    // TODO : Write a function to set `K` (Camera-Intrinsics)
    // K : 525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0
    K = Matrix3d::Zero();
    fx = .5* 525.0;
    fy = .5* 525.0;
    cx = .5* 319.5;
    cy = .5* 239.5;
    K(0,0) = fx;
    K(1,1) = fy;
    K(0,2) = cx;
    K(1,2) = cy;
    K(2,2) = 1.0;
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




    //
    // make a 3xN Matrix of 3d points in edges of reference frame. N is #of edge pixels
    int N = cv::countNonZero( ref_edge );
    cout << "# of non-zero elements "<< N << "\n";
    list_edge_ref = MatrixXd::Zero(3,N); // list of edges (X,Y,Z)




    // loop thru edge pixels
    int c=0;
    for( int yy=0 ; yy<ref_im.rows ; yy++ )
    {
        for( int xx=0 ; xx<ref_im.cols ; xx++ )
        {
            if( ref_edge.at<uchar>(yy,xx) > 0 )
            {
                double Z = ref_depth.at<float>(yy,xx);
                Z = (Z==0)?1.0:Z; //to avoid division by zero
                double X = Z * (xx-cx) / fx;
                double Y = Z * (yy-cy) / fy;

                list_edge_ref(0,c) = X;
                list_edge_ref(1,c) = Y;
                list_edge_ref(2,c) = Z;
                c++;
            }
        }
    }
    assert( c == N ); //just to verify that number of edge pixels same as length of the list
//    cout << "c ::: "<< c << endl;
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


    now_dist_transform.convertTo(now_dist_transform_display, CV_8UC1);
//    cv::imshow( "dist_trans", dist_transform_display );
//    cv::imshow( "now_edge", now_edge );
//    cout << now_edge << endl;
//    cv::waitKey(0);

}

/// @brief use `now_dist_transform`, `list_edge_ref`, camera_intrinsics and set up a CERES problem.
/// It is assumed that setRefFrame() and setNowFrame() have been called before calling this function
/// TODO : set bool variables to be sure that above calls have been made before calling this function
void SolveEA::setAsCERESProblem()
{

}

/// @brief Verify the variable `list_edge_ref` ie a 3xN list of 3d points of reference frame.
void SolveEA::_verify3dPts()
{
    // Project 3d points

    MatrixXd _3d_transformed = list_edge_ref; //TODO, transform with R, T
    ArrayXXd lastRow_inv = _3d_transformed.row(2).array().inverse();
    for( int i=0 ; i<3 ; i++ )
            _3d_transformed.row(i).array() *= lastRow_inv;



    // here, i am assured projected_edges last row will be 1.0
    MatrixXd projected_edges_list = K * _3d_transformed;



//    MatrixXi mask = MatrixXi::Zero(ref_im.rows, ref_im.cols);
    cv::Mat mask = cv::Mat::zeros(ref_im.rows, ref_im.cols, CV_8UC1);
    cordList_2_mask( projected_edges_list, mask );


//    cv::Mat mask1;
//    cv::eigen2cv( mask, mask1 );
    cv::Mat _overlay;
    sOverlay( now_im, mask, _overlay, cv::Vec3b(0,0,255));
    cv::imshow( "mask", mask );
    cv::imshow( "_overlay", _overlay );

    cv::imshow( "distance_trans", now_dist_transform_display);

}


/// Given a list of co-ordinates make a mask of out of it
void SolveEA::cordList_2_mask(MatrixXd &list, Eigen::MatrixXi &mask)
{
    assert( mask.rows() > 0 && mask.cols() > 0 );

    for( int i=0 ; i<list.cols() ; i++ )
    {
        if( list(0,i)<0 || list(0,i)>mask.cols() ||  list(1,i)<0 || list(1,i)>mask.rows()) {
            continue;
        }

        int xx = list(0,i);
        int yy = list(1,i);

        printf( "mask(%d,%d) = 1", yy, xx );
        mask(yy,xx) = 1;
    }
}
/// @mask is single channel 8UC_1
void SolveEA::cordList_2_mask(MatrixXd &list, cv::Mat &mask)
{
    assert( mask.rows > 0 && mask.cols > 0 );

    for( int i=0 ; i<list.cols() ; i++ )
    {
        if( list(0,i)<0 || list(0,i)>mask.cols ||  list(1,i)<0 || list(1,i)>mask.rows) {
            continue;
        }

        int xx = (int)list(0,i);
        int yy = (int)list(1,i);

        mask.at<uchar>(yy,xx) = 255;
    }

}
/// @brief Overlay a mask over an image
void SolveEA::sOverlay(const Mat &src, const Mat &mask, Mat &dst, const Vec3b &color)
{
    assert( src.channels() == 3 && mask.rows == src.rows && mask.cols == src.cols && "SolveEA::sOverlay : sizes do not match");

    dst = src.clone();
    for( int j=0 ; j<mask.cols ; j++ )
    {
        for( int i=0 ; i<mask.rows ; i++ )
        {
            if( mask.at<uchar>(i,j) > 0 )
                dst.at<cv::Vec3b>(i,j) = color;
        }
    }
}
