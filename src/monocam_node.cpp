#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include "feature_frontend/monotrackerRos/monotrackerRos.h"
#include <ros/ros.h>

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "track_mono" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug );

    MonoTracker* tracker = new MonoTracker( );
    tracker->readParameters( n );

    ros::spin( );

    if ( !ros::ok( ) )
        delete tracker;

    return 0;
}
