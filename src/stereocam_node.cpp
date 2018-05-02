#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include "feature_frontend/stereoTrackerRos/stereotrackerRos.h"

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "track_stereo" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug );

    StereoTracker* tracker = new StereoTracker( );
    tracker->readParameters( n );

    ros::spin( );

    if ( !ros::ok( ) )
        delete tracker;

    return 0;
}
