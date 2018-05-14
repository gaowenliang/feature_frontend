#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include <camera_model/camera_models/CameraFactory.h>
#include <code_utils/eigen_utils.h>
#include <code_utils/math_utils/Polynomial.h>
#include <code_utils/sys_utils/eigen_file_io.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "reproject_test" );
    ros::NodeHandle n( "~" );

    std::string camera_model_file;
    std::string image_mask;
    std::string data_save_path;

    n.getParam( "camera_model", camera_model_file );
    n.getParam( "mask", image_mask );
    n.getParam( "data_save_path", data_save_path );
    camera_model::CameraPtr cam;

    std::cout << "#INFO: camera config is " << camera_model_file << std::endl;
    cam = camera_model::CameraFactory::instance( )->generateCameraFromYamlFile( camera_model_file );

    std::cout << cam->parametersToString( ) << std::endl;

    cv::Mat mask;
    mask = cv::imread( image_mask, cv::IMREAD_GRAYSCALE );
    if ( mask.empty( ) )
    {
        mask = cv::Mat( cam->imageHeight( ), cam->imageWidth( ), CV_8UC1, cv::Scalar( 255 ) );
    }

    vector< double > params;
    cam->writeParameters( params );
    std::cout << cam->parametersToString( ) << std::endl;

    std::cout << "#INFO: LOADing camera config is DONE." << camera_model_file << std::endl;

    int rows = cam->imageHeight( );
    int cols = cam->imageWidth( );

    std::vector< std::vector< Eigen::Vector3d > > image_vectors;
    image_vectors.resize( rows );
    for ( int row_index = 0; row_index < rows; ++row_index )
    {
        image_vectors.at( row_index ).resize( cols );
    }

    int error_num = 0;
    int num       = 0;
    for ( int row_index = 0; row_index < rows; ++row_index )
    {
        for ( int col_index = 0; col_index < cols; ++col_index )
        {
            Eigen::Vector2d p_u( col_index, row_index );
            Eigen::Vector3d P;

            //            Eigen::Vector2d p_u2;
            cam->liftSphere( p_u, P );
            //            cam->spaceToPlane( P, p_u2 );

            P.normalize( );

            image_vectors[row_index][col_index] = P;

            // std::cout << row_index << " " << col_index << " : "
            //           << image_vectors[row_index][col_index].transpose( ) << "\n";
        }
    }
    std::vector< std::vector< double > > error_angle;
    error_angle.resize( rows );
    for ( int row_index = 0; row_index < rows; ++row_index )
    {
        error_angle.at( row_index ).resize( cols, 0.0 );
    }

    Eigen::MatrixXd error_angle_mat( rows, cols );

    std::ofstream out_t;
    out_t.open( data_save_path + "data_t.txt", std::ios::trunc );
    out_t << std::setprecision( 10 );

    int num_sum      = 0;
    double angle_sum = 0;
    for ( int row_index = 0; row_index < rows; ++row_index )
    {
        for ( int col_index = 0; col_index < cols; ++col_index )
        {

            Eigen::Vector3d p00 = image_vectors[row_index][col_index];

            double sum_angle = 0.0;
            double sum_pixel = 0.0;

            if ( col_index != 0 && row_index != 0 )
            {
                Eigen::Vector3d p_1_1 = image_vectors[row_index - 1][col_index - 1];
                double angle = acos( p00.normalized( ).dot( p_1_1.normalized( ) ) );
                sum_angle += angle;
                sum_pixel += sqrt( 2 );
            }
            if ( row_index != 0 && col_index != ( cols - 1 ) )
            {
                Eigen::Vector3d p_11 = image_vectors[row_index - 1][col_index + 1];
                double angle         = acos( p00.normalized( ).dot( p_11.normalized( ) ) );
                sum_angle += angle;
                sum_pixel += sqrt( 2 );
            }
            if ( col_index != ( cols - 1 ) && row_index != ( rows - 1 ) )
            {
                Eigen::Vector3d p11 = image_vectors[row_index + 1][col_index + 1];
                double angle        = acos( p00.normalized( ).dot( p11.normalized( ) ) );
                sum_angle += angle;
                sum_pixel += sqrt( 2 );
            }
            if ( col_index != 0 && row_index != ( rows - 1 ) )
            {
                Eigen::Vector3d p1_1 = image_vectors[row_index + 1][col_index - 1];
                double angle         = acos( p00.normalized( ).dot( p1_1.normalized( ) ) );
                sum_angle += angle;
                sum_pixel += sqrt( 2 );
            }

            if ( row_index != 0 )
            {
                Eigen::Vector3d p_10 = image_vectors[row_index - 1][col_index];
                double angle         = acos( p00.normalized( ).dot( p_10.normalized( ) ) );
                sum_angle += angle;
                sum_pixel += 1.0;
            }
            if ( col_index != 0 )
            {
                Eigen::Vector3d p0_1 = image_vectors[row_index][col_index - 1];
                double angle         = acos( p00.normalized( ).dot( p0_1.normalized( ) ) );
                sum_angle += angle;
                sum_pixel += 1.0;
            }
            if ( col_index != ( cols - 1 ) )
            {
                Eigen::Vector3d p01 = image_vectors[row_index][col_index + 1];
                double angle        = acos( p00.normalized( ).dot( p01.normalized( ) ) );
                sum_angle += angle;
                sum_pixel += 1.0;
            }
            if ( row_index != ( rows - 1 ) )
            {
                Eigen::Vector3d p10 = image_vectors[row_index + 1][col_index];
                double angle        = acos( p00.normalized( ).dot( p10.normalized( ) ) );
                sum_angle += angle;
                sum_pixel += 1.0;
            }

            double angle_avg        = sum_angle / sum_pixel;
            double angle_avg_degree = angle_avg * 57.29;
            // std::cout << "sum_angle " << sum_angle << " sum_pixel " << sum_pixel << "\n";

            if ( mask.at< uchar >( row_index, col_index ) > 10 )
            {
                if ( angle_avg_degree < 1 && angle_avg_degree > 0.001 )
                {
                    error_angle[row_index][col_index] = angle_avg;
                    error_angle_mat( row_index, col_index ) = angle_avg;

                    out_t << row_index << " " << col_index << " " << angle_avg_degree << " "
                          << angle_avg << '\n';

                    ++num_sum;
                    angle_sum += angle_avg;
                }
            }
        }
    }

    double avg_all_angle        = angle_sum / num_sum;
    double avg_all_angle_degree = avg_all_angle * 57.29;
    // std::cout << "num_sum " << num_sum << " avg_all_angle " << avg_all_angle << "\n";

    for ( int row_index = 0; row_index < rows; ++row_index )
    {
        for ( int col_index = 0; col_index < cols; ++col_index )
        {
            if ( error_angle[row_index][col_index] == 0.0 )
            {
                error_angle_mat( row_index, col_index ) = avg_all_angle;

                out_t << row_index << " " << col_index << " " << avg_all_angle_degree << " "
                      << avg_all_angle << '\n';
            }
        }
    }

    out_t.close( );

    std::cout << "[#INFO] write data start! \n";

    sys_utils::io::writeMatrixToBinary< Eigen::MatrixXd >( data_save_path + "data_mat_t", error_angle_mat );

    std::cout << "[#INFO] write data Done! \n";

    // read data test
    //{
    //    std::ofstream out_t2;
    //    out_t2.open( data_save_path + "data_t2.txt", std::ios::trunc );
    //    out_t2 << std::setprecision( 10 );
    //
    //    Eigen::MatrixXd error_angle_mat2( rows, cols );
    //    parseMatrixFromBinary< Eigen::MatrixXd >( data_save_path + "data_mat_t.txt",
    //    error_angle_mat2 );
    //    for ( int row_index = 0; row_index < rows; ++row_index )
    //    {
    //        for ( int col_index = 0; col_index < cols; ++col_index )
    //        {
    //            out_t2 << row_index << " " << col_index << " "
    //                   << error_angle_mat2( row_index, col_index ) * 57.29 << " "
    //                   << error_angle_mat2( row_index, col_index ) << '\n';
    //        }
    //    }
    //    out_t2.close( );
    //}

    std::cout << "All Done ! \n";

    return 0;
}
