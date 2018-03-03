#ifndef FEATUREMONO_H
#define FEATUREMONO_H

#include "../frontendBase/featuretracker.h"

namespace frontend
{

class FeatureMonoTracker : public frontend::FeatureTracker
{
    public:
    FeatureMonoTracker( std::string cam_file, std::string feature_file );

    void process( bool* is_detect_new_feature );
    cv::Mat draw( );
    void readImages( const std::vector< cv::Mat >& image_buf );
    void getPoints( std::vector< std::vector< Eigen::Vector3d > >& points, std::vector< int >& id );

    private:
    void trackFeatures( const cv::Mat& image );
    void rejectWithF( );
};

typedef boost::shared_ptr< FeatureMonoTracker > FeatureMonoPtr;
}
#endif // FEATUREMONO_H
