#ifndef FEATURESTEREO_H
#define FEATURESTEREO_H

#include "../frontendBase/featuretracker.h"

#define MAX_WINDOW_SIZE 10

namespace frontend
{

class FeatureStereoTracker : public frontend::FeatureTracker
{
    public:
    FeatureStereoTracker( std::string left_cam_file,
                          std::string right_cam_file, //
                          std::string left_feature_file,
                          std::string right_feature_file );

    FeatureStereoTracker( std::string left_cam_file,
                          std::string right_cam_file,
                          std::string left_err_file,
                          std::string right_err_file,
                          std::string left_feature_file,
                          std::string right_feature_file );

    void process( bool* frontendCtrl );
    cv::Mat draw( );
    void readImages( const std::vector< cv::Mat >& image_buf );
    void getPoints( std::vector< std::vector< Eigen::Vector3d > >& points, std::vector< int >& id );
    void getPoints( std::vector< std::vector< Eigen::Vector3d > >& points,
                    std::vector< std::vector< double > >& error_angle,
                    std::vector< int >& id );
    void getPoints( std::vector< std::vector< Eigen::Vector3d > >& points,
                    std::vector< uchar >& status,
                    std::vector< int >& id );
    std::vector< Eigen::Vector3d > getUndistortedPointsRight( );

    private:
    void trackFeatures( const cv::Mat& left_image, const cv::Mat& right_image, bool is_track_right );

    public:
    frontend::FeatureBase* m_tracker_r;
    cv::Ptr< cv::SparsePyrLKOpticalFlow > m_sparseFlowLeft2Right;
    cv::Rect rightRect;
};

typedef boost::shared_ptr< FeatureStereoTracker > FeatureStereoPtr;
}
#endif // FEATURESTEREO_H
