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
    FrontendCamera( std::string camera_file, std::string error_file );

    bool readCameraFromYamlFile( const std::string& filename );
    bool readCameraFromYamlFile( const std::string& filename, const std::string& error_file );

    std::vector< Eigen::Vector3d > undistortedPoints( const std::vector< cv::Point2f > pts );
    std::vector< double > getErrorAngle( std::vector< cv::Point2f > points );

    camera_model::CameraPtr camera( ) const;
    int row( );
    int col( );

    void setR_cc2( const Eigen::Matrix3d& R_cc2 );

    private:
    camera_model::CameraPtr m_cam;
    Eigen::MatrixXd error_angle_mat;

    int m_is_rectify;
    Eigen::Matrix3d m_R_cc2;
};
}

#endif // FrontendCamera_H
