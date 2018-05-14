#include "feature_frontend/feature_lib/frontendBase/frontendcamera.h"
#include <camera_model/camera_models/CameraFactory.h>
#include <code_utils/sys_utils/eigen_file_io.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/core/eigen.hpp>

using namespace frontend;

FrontendCamera::FrontendCamera( std::string camera_file )
{
    readCameraFromYamlFile( camera_file );
}

FrontendCamera::FrontendCamera( std::string camera_file, std::string error_file )
{
    readCameraFromYamlFile( camera_file );

    error_angle_mat.resize( camera( )->imageHeight( ), camera( )->imageWidth( ) );

    sys_utils::io::parseMatrixFromBinary< Eigen::MatrixXd >( error_file, error_angle_mat );
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

bool
FrontendCamera::readCameraFromYamlFile( const std::string& filename, const std::string& error_file )
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

    error_angle_mat.resize( camera( )->imageHeight( ), camera( )->imageWidth( ) );

    sys_utils::io::parseMatrixFromBinary< Eigen::MatrixXd >( error_file, error_angle_mat );
    std::cout << "Load Error Angle file:" << error_file << std::endl;

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
        {
            un_pts.push_back( ( m_R_cc2 * b ).normalized( ) );
        }
        else
        {
            un_pts.push_back( b.normalized( ) );
        }
    }

    return un_pts;
}

std::vector< double >
FrontendCamera::getErrorAngle( std::vector< cv::Point2f > points )
{
    std::vector< double > angles;

    for ( auto& pt : points )
    {
        // std::cout << pt.y << " " << pt.x << "\n";
        // std::cout << error_angle_mat( pt.y, pt.x ) << "\n";

        if ( pt.x >= 0 && pt.x <= camera( )->imageWidth( ) && pt.y >= 0
             && pt.y <= camera( )->imageHeight( ) )
            angles.push_back( error_angle_mat( pt.y, pt.x ) );
        else
            angles.push_back( 15 / 57.29 ); // 5 degree error if track out
    }
    return angles;
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
