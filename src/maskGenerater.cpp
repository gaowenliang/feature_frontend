#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include <camera_model/camera_models/CameraFactory.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "maskGenerater" );
    ros::NodeHandle n( "~" );

    camera_model::CameraPtr cam;

    std::string camera_model_file = "blue280_fisheye_camera_calib.yaml";
    std::string output_image      = "blue280_fisheye_camera_calib_mask.jpg";
    int maxFOV                    = 90;
    int minFOV                    = 0;

    n.getParam( "camera_model", camera_model_file );
    n.getParam( "maxFOV", maxFOV );
    n.getParam( "minFOV", minFOV );
    n.getParam( "output_image", output_image );

    std::cout << "#INFO: camera config is " << camera_model_file << std::endl;
    cam = camera_model::CameraFactory::instance( )->generateCameraFromYamlFile( camera_model_file );
    std::vector< double > params;
    cam->writeParameters( params );
    std::cout << cam->parametersToString( ) << std::endl;
    std::cout << "#INFO: LOADing camera config is DONE." << camera_model_file << std::endl;

    double maxMaskIncidentAngle = maxFOV / 2 * ( M_PI / 180.0 );
    double minMaskIncidentAngle = minFOV / 2 * ( M_PI / 180.0 );
    std::cout << "[#INFO] max mask FOV: " << maxFOV << " degrees," << std::endl
              << "[#INFO] min mask FOV: " << minFOV << " degrees," << std::endl
              << "[#INFO] max mask Incident Angle: " << maxMaskIncidentAngle << " rad" << std::endl
              << "[#INFO] min mask Incident Angle: " << minMaskIncidentAngle << " rad" << std::endl;

    int w, h;
    w = ( int )( cam->imageWidth( ) );
    h = ( int )( cam->imageHeight( ) );

    cv::Mat mask_output( h, w, CV_8UC1, cv::Scalar( 0 ) );

    int i     = 0;
    double t0 = cv::getTickCount( );

    for ( int row_index = 0; row_index < h; ++row_index )
    {
        for ( int col_index = 0; col_index < w; ++col_index, ++i )
        {
            Eigen::Vector2d p_u( col_index, row_index );
            Eigen::Vector3d P;

            Eigen::Vector2d p_u2;
            cam->liftSphere( p_u, P );
            cam->spaceToPlane( P, p_u2 );

            double theta = acos( P( 2 ) / P.norm( ) );

            if ( theta <= maxMaskIncidentAngle && theta > minMaskIncidentAngle )
                mask_output.at< char >( row_index, col_index ) = 255;
            //            else if ( theta > maskIncidentAngle )
            //                mask_output.at< char >( col_index, row_index ) = 0;
        }
    }

    double t1 = cv::getTickCount( );
    std::cout << "Total cost Time: " << 1000 * ( t1 - t0 ) / cv::getTickFrequency( ) << " ms" << std::endl;

    cv::namedWindow( "mask_output", cv::WINDOW_NORMAL );
    cv::imshow( "mask_output", mask_output );
    cv::imwrite( output_image, mask_output );
    cv::waitKey( 0 );

    return 0;
}
