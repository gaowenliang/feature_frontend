#include "feature_frontend/feature_lib/frontendBase/frontendcamera.h"
#include <camera_model/camera_models/CameraFactory.h>
#include <iostream>
#include <opencv2/core/eigen.hpp>

using namespace frontend;

FrontendCamera::FrontendCamera( std::string camera_file )
{
    readCameraFromYamlFile( camera_file );
}

bool
FrontendCamera::readCameraFromYamlFile( const std::string& filename )
{
    std::cout << "Load Camera file:" << filename << std::endl;

    m_cam = camera_model::CameraFactory::instance( )->generateCameraFromYamlFile( filename );

    std::cout << "Camera" << std::endl //
              << m_cam->parametersToString( ) << std::endl;

    cv::FileStorage fs( filename, cv::FileStorage::READ );

    if ( !fs.isOpened( ) )
    {
        std::cerr << "ERROR: Wrong path to settings 'camera_config_file': " << filename << std::endl;
        return false;
    }

    m_is_rectify = fs["is_rectify"];

    if ( m_is_rectify )
    {
        cv::Mat cv_R_cc2;
        fs["rectifyRotation"] >> cv_R_cc2;

        cv::cv2eigen( cv_R_cc2, m_R_cc2 );
        std::cout << "R_cc2 " << std::endl << m_R_cc2 << std::endl;
    }

    fs.release( );

    return true;
}

std::vector< Eigen::Vector3d >
FrontendCamera::undistortedPoints( const std::vector< cv::Point2f > pts )
{
    std::vector< Eigen::Vector3d > un_pts;
    for ( auto& pt : pts )
    {
        Eigen::Vector2d a( pt.x, pt.y );
        Eigen::Vector3d b;
        m_cam->liftProjective( a, b );

        if ( m_is_rectify )
            un_pts.push_back( ( m_R_cc2 * b ).normalized( ) );
        else
            un_pts.push_back( b.normalized( ) );
    }

    return un_pts;
}

camera_model::CameraPtr
FrontendCamera::camera( ) const
{
    return m_cam;
}

int
FrontendCamera::row( )
{
    return m_cam->imageHeight( );
}

int
FrontendCamera::col( )
{
    return m_cam->imageWidth( );
}

void
FrontendCamera::setR_cc2( const Eigen::Matrix3d& R_cc2 )
{
    m_R_cc2 = R_cc2;
}
