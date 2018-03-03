#ifndef FrontendCamera_H
#define FrontendCamera_H

#include <camera_model/camera_models/CameraFactory.h>

namespace frontend
{
//!
//! \brief The Camera class
//! mono tracker need 1 camera
//! stereo tracker need 2 camera
//!
class FrontendCamera
{
    public:
    FrontendCamera( ) {}
    FrontendCamera( std::string camera_file );

    bool readCameraFromYamlFile( const std::string& filename );

    std::vector< Eigen::Vector3d > undistortedPoints( const std::vector< cv::Point2f > pts );
    camera_model::CameraPtr camera( ) const;
    int row( );
    int col( );

    void setR_cc2( const Eigen::Matrix3d& R_cc2 );

    private:
    camera_model::CameraPtr m_cam;
    int m_is_rectify;
    Eigen::Matrix3d m_R_cc2;
};
}

#endif // FrontendCamera_H
