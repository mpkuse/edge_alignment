#include <iostream>
using namespace std;

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>



// Eigen
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

// ceres
#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <ceres/loss_function.h>
using namespace ceres;
// using namespace ceres::Grid2D;


#include "utils.h"
#include "PoseManipUtils.h"


int main()
{
    //
    // Load Image A and its Depth image
    cv::Mat imA = cv::imread( "../rgb-d/rgb/1.png");
    cv::Mat imA_depth = cv::imread( "../rgb-d/depth/1.png", CV_LOAD_IMAGE_ANYDEPTH);


    //
    // Load Image B. No depth is needed for this
    cv::Mat imB = cv::imread( "../rgb-d/rgb/5.png");


    //
    // Print Info on Images
    cout << "==========INFO ON IMages ===========\n";
    cout << "imA.type "<< mat_info( imA ) << endl;  // 640x480 CV_8UC3
    cout << "imA_depth.type "<< mat_info( imA_depth ) << endl; // 640x480 CV_16UC1
    cout << "imB.type "<< mat_info( imB ) << endl;  // 640x480 CV_8UC3

    cv::imshow( "A", imA );
    cv::imshow( "B", imB );
    cv::imshow( "imA_depth", imA_depth );

    double min, max;
    cv::minMaxLoc(imA_depth, &min, &max);
    cout << "imA_depth min,max: " << min << " " << max  << endl;
    //
    // DONE printing debug info on images
    cout << "=========== Done ============\n";


    //
    // Intrinsic Calibration. Obtained from https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
    Eigen::Matrix3d K;
    // double fx = 517.3, fy = 516.5, cx = 318.6, cy = 255.3;
    double fx = 525., fy = 525., cx = 319.5, cy = 239.5;
    K << fx, 0., cx , 0., fy, cy, 0., 0., 1.;
    cout << "K\n" << K << endl;

    Eigen::Matrix<double,5,1> D; //d0, d1, d2, d3, d4
    D << 0.2624,	-0.9531,	-0.0054,	0.0026,	1.1633;
    cout << "D\n" << D << endl;



    //
    // Get 3D points of imA as a 4xN matrix. ^aX. These 3d points are in frame-of-ref of imA.
    Eigen::MatrixXd a_X;
    get_aX( imA, imA_depth, K, a_X );

    //
    // Distance Transform of edges of imB
    cv::Mat disTrans;
    get_distance_transform( imB, disTrans );
    Eigen::MatrixXd e_disTrans;
    cv::cv2eigen( disTrans, e_disTrans );
    imshow("Distance Transform Image", disTrans); //numbers between 0 and 1.



    #if 1
    //
    // Verify if everything is OK.
    // Use the 3d points are reproject of imA and overlay those reprojected points.
    cout << "a_X\n" << a_X.leftCols(10) << endl;

    for( int ss=0 ; ss<150 ; ss+=10 )
    {
        cout << ":::::::" <<  ss << " to " << ss+10 << endl;
        cout << a_X.block( 0, ss, 4, 10  ) << endl;
    }

    Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
    Eigen::MatrixXd b_u_rep;
    reproject( a_X, M, K, b_u_rep );
    cout << "b_u\n" << b_u_rep.leftCols(10) << endl;
    // s_overlay( imB, b_u_rep );
    #endif

    ////////////////////////////////////////////////////////////////////////////
    ///////////////////// Setup non-linear Least Squares ///////////////////////
    ////////////////////////////////////////////////////////////////////////////




    // Using `a_X` and `disTrans` setup the non-linear least squares problem
    cout << "e_disTrans.shape = " << e_disTrans.rows() << ", " << e_disTrans.cols() << endl;
    ceres::Grid2D<double,1> grid( e_disTrans.data(), 0, e_disTrans.cols(), 0, e_disTrans.rows() );
    ceres::BiCubicInterpolator< ceres::Grid2D<double,1> > interpolated_imb_disTrans( grid );

    /*
    cv::Mat m = cv::Mat::zeros( disTrans.rows, disTrans.cols, CV_32FC1 );
    for( int r=0 ; r<disTrans.rows ; r++ )
    {
        for( int c=0 ; c<disTrans.cols ; c++ )
        {
            double __r;
            interpolated_imb_disTrans.Evaluate( c,r, &__r );
            m.at<float>(r,c) = (float) __r;
            // cout << __r << " ";
        }
        // cout << "\n";
    }

    cout << mat_info( disTrans ) << endl;
    cout << mat_info( m ) << endl;
    cv::imshow( "disTrans", disTrans );
    cv::imshow( "disTrans_from_interpolation", m );
    cv::waitKey(0);
    */

    // return 0;

    // Initial Guess
    Eigen::Matrix4d b_T_a_optvar = Eigen::Matrix4d::Identity();
    double b_quat_a[10], b_t_a[10];
    PoseManipUtils::eigenmat_to_raw( b_T_a_optvar, b_quat_a, b_t_a );
    cout << "Initial Guess : " << PoseManipUtils::prettyprintMatrix4d( b_T_a_optvar ) << endl;
    Eigen::MatrixXd b_u;
    reproject( a_X, b_T_a_optvar, K, b_u );
    s_overlay( imB, b_u, "initial" );


    // Residues for each 3d points
    ceres::Problem problem;
    for( int i=0 ; i< a_X.cols() ; i+=30 )
    {
        // ceres::CostFunction * cost_function = EAResidue::Create( K, a_X.col(i), interpolated_imb_disTrans);

        ceres::CostFunction * cost_function = EAResidue::Create( fx,fy,cx,cy,  a_X(0,i),a_X(1,i),a_X(2,i), interpolated_imb_disTrans);
        problem.AddResidualBlock( cost_function, new CauchyLoss(1.), b_quat_a, b_t_a );
    }

    ceres::LocalParameterization * quaternion_parameterization = new ceres::QuaternionParameterization;
    problem.SetParameterization( b_quat_a, quaternion_parameterization );

    // Run
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_QR;
    Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    std::cout << summary.FullReport() << "\n";


    PoseManipUtils::raw_to_eigenmat( b_quat_a, b_t_a, b_T_a_optvar );
    cout << "Final Guess : " << PoseManipUtils::prettyprintMatrix4d( b_T_a_optvar ) << endl;


    reproject( a_X, b_T_a_optvar, K, b_u );
    s_overlay( imB, b_u, "final" );

    cv::waitKey(0);

    // const double data[] = {1.0, 3.0, -1.0, 4.0,
    //                     3.6, 2.1,  4.2, 2.0,
    //                    2.0, 1.0,  3.1, 5.2};
    // ceres::Grid2D<double, 1>  grid(data, 0,3, 0,4);
}
