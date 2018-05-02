#include "feature_frontend/monotrackerRos/monotrackerRos.h"
#include <sensor_msgs/Image.h>

MonoTracker::MonoTracker( )
: max_id( 0 )
, FREQ( 100 )
{
}

void
MonoTracker::readParameters( ros::NodeHandle nh )
{
    std::vector< std::string > cam_files;
    std::vector< std::string > angle_files;
    std::vector< std::string > feature_files;

    cam_files.push_back( ros_utils::readParam< std::string >( nh, "cam_config_file" ) );
    angle_files.push_back( ros_utils::readParam< std::string >( nh, "error_config_file" ) );
    feature_files.push_back(
    ros_utils::readParam< std::string >( nh, "feature_config_file" ) );

    image_in_buf.resize( 1 );

    tracker = frontend::FrontendInit::newFrontend( )->init( frontend::MONO, cam_files, angle_files, feature_files );

    FREQ = tracker->freq( );

    std::string image_sub_topic = ros_utils::readParam< std::string >( nh, "image_topic" );

    subImgs = nh.subscribe< sensor_msgs::Image >( image_sub_topic,
                                                  100, //
                                                  &MonoTracker::image_callback,
                                                  this,
                                                  ros::TransportHints( ).tcpNoDelay( true ) );

    if ( tracker->isShowTrack( ) )
    {
        std::string cameraName = tracker->camera( )->cameraName( );
        pubTtrackImage = nh.advertise< sensor_msgs::Image >( "feature_" + cameraName, 1000 );
        //        cv::namedWindow( tracker->camera( )->cameraName( ), cv::WINDOW_NORMAL );
    }

    pointsPub = nh.advertise< sensor_msgs::PointCloud >( "feature", 1000 );
}

void
MonoTracker::image_callback( const sensor_msgs::ImageConstPtr& img_msg )
{
    TicToc t_track;

    //    ROS_INFO( "Recieving image %lf", img_msg->header.stamp.toSec( ) );
    if ( first_image_flag )
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec( );
    }

    // frequency control
    if ( round( 1.0 * detect_count / ( img_msg->header.stamp.toSec( ) - first_image_time ) ) <= FREQ )
    {
        frondendCtrl[0] = true;
        frondendCtrl[1] = true;
        // reset the frequency control
        if ( abs( 1.0 * detect_count / ( img_msg->header.stamp.toSec( ) - first_image_time ) - FREQ )
             < 0.01 * FREQ )
        {
            first_image_time = img_msg->header.stamp.toSec( );
            detect_count     = 0;
        }
    }
    else
    {
        frondendCtrl[0] = false;
        frondendCtrl[1] = false;
    }

    image_ptr = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::MONO8 );

    image_in_buf[0] = image_ptr->image.rowRange( 0, tracker->row( ) );

    track( img_msg->header.stamp, frondendCtrl );

    ROS_INFO( "frontend all costs %f", t_track.toc( ) );
}

void
MonoTracker::track( ros::Time now_t, bool* is_frondendCtrl )
{
    TicTocPart t_track;

    tracker->updateNewPoints( max_id );
    ROS_DEBUG( "updateNewPoints costs %f", t_track.toc( ) );

    tracker->readImages( image_in_buf );
    ROS_DEBUG( "readImages costs %f", t_track.toc( ) );

    tracker->process( is_frondendCtrl );
    ROS_DEBUG( "process costs %f", t_track.toc( ) );

    if ( is_frondendCtrl[1] )
    {
        detect_count++;
        pubFeature( now_t );
    }
    ROS_DEBUG( "pubFeature costs %f", t_track.toc( ) );

    if ( is_frondendCtrl[1] )
    {

        if ( tracker->isShowTrack( ) && tracker->isTracked( ) )
        {
            cv::Mat image_show = tracker->draw( );
            cv_bridge::CvImage outImage;
            outImage.header.stamp    = now_t;
            outImage.header.frame_id = "track";
            outImage.encoding        = "bgr8";
            outImage.image           = image_show;
            pubTtrackImage.publish( outImage );
            // cv::imshow( tracker->camera( )->cameraName( ), image_show );
            // cv::waitKey( 10 );
        }
    }
    ROS_DEBUG( "draw costs %f", t_track.toc( ) );
}

void
MonoTracker::pubFeature( ros::Time now_t )
{
    sensor_msgs::PointCloudPtr feature_points( new sensor_msgs::PointCloud );
    feature_points->header.stamp    = now_t;
    feature_points->header.frame_id = "world";
    sensor_msgs::ChannelFloat32 error_of_point; // id channel of each object
    sensor_msgs::ChannelFloat32 id_of_point;    // id channel of each object
    sensor_msgs::ChannelFloat32 cams_of_point;  // id channel of each object

    if ( !tracker->isTracked( ) )
        return;

    std::vector< std::vector< Eigen::Vector3d > > un_pts;
    std::vector< std::vector< double > > angle_pts;
    std::vector< int > id;

    //    tracker->getPoints( un_pts, id );
    tracker->getPoints( un_pts, angle_pts, id );

    for ( unsigned int j = 0; j < id.size( ); j++ )
    {
        int p_id = id[j];
        //    std::cout << "un_pts_r " << un_pts[j].x( ) << " " <<
        //    un_pts[j].y( ) << std::endl;

        geometry_msgs::Point32 p;
        p.x = un_pts[0][j]( 0 );
        p.y = un_pts[0][j]( 1 );
        p.z = un_pts[0][j]( 2 );
        //            ROS_DEBUG( "point_out--  %lf %lf %lf", p.x, p.y, p.z );
        feature_points->points.push_back( p );
        id_of_point.values.push_back( p_id );
        cams_of_point.values.push_back( 0 );
        error_of_point.values.push_back( angle_pts[0][j] );
    }

    feature_points->channels.push_back( id_of_point );
    feature_points->channels[0].name = "id_point";
    feature_points->channels.push_back( cams_of_point );
    feature_points->channels[1].name = "id_camera";
    feature_points->channels.push_back( error_of_point );
    feature_points->channels[2].name = "angle_per_pix";

    pointsPub.publish( feature_points );
}
