#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include "frontendbase.h"

namespace frontend
{
class FeatureTracker : public FeatureBase
{
    public:
    virtual void readImages( const std::vector< cv::Mat >& image_buf ) = 0;
    virtual void getPoints( std::vector< std::vector< Eigen::Vector3d > >&, std::vector< int >& ) = 0;
    virtual void process( bool* ) = 0;
    virtual cv::Mat draw( )       = 0;
};

typedef boost::shared_ptr< FeatureTracker > FrontendPtr;
}
#endif // FEATURETRACKER_H
