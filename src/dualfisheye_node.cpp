#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include "feature_frontend/dualFisheye/dualfisheyetracker.h"

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "track_multi" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug );

    DualFisheyeTracker* tracker = new DualFisheyeTracker( n );

    ros::spin( );

    if ( !ros::ok( ) )
        delete tracker;

    return 0;
}
