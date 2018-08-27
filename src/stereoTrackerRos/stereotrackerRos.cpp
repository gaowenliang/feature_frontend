#include "feature_frontend/stereoTrackerRos/stereotrackerRos.h"

void
StereoTracker::readParameters( ros::NodeHandle nh )
{
    NUM_OF_CAM = ros_utils::readParam< int >( nh, "camera_num_stereo" );
    p_subImgs.resize( NUM_OF_CAM * 2 );

    int max_freq = 1;

    for ( int camera_index = 0; camera_index < NUM_OF_CAM; camera_index++ )
    {
        std::string prefix = boost::str( boost::format( "camera%d/" ) % camera_index );

        std::vector< std::string > cam_files;
        std::vector< std::string > angle_files;
        std::vector< std::string > feature_files;

        cam_files.push_back( ros_utils::readParam< std::string >( nh, prefix + "cam_config_file_left" ) );
        cam_files.push_back( ros_utils::readParam< std::string >( nh, prefix + "cam_config_file_right" ) );

        angle_files.push_back( ros_utils::readParam< std::string >( nh, prefix + "error_config_left" ) );
        angle_files.push_back( ros_utils::readParam< std::string >( nh, prefix + "error_config_right" ) );

        feature_files.push_back( ros_utils::readParam< std::string >( nh, //
                                                                      prefix + "feature_config_file" ) );
        feature_files.push_back( ros_utils::readParam< std::string >( nh, //
                                                                      prefix + "feature_config_file" ) );

        image_in_buf.resize( 2 );
        image_ptr.resize( 2 );

        tracker
        = frontend::FrontendInit::newFrontend( )->init( frontend::STEREO, cam_files, angle_files, feature_files );

        max_freq = max_freq > tracker->freq( ) ? max_freq : tracker->freq( );

        std::string image_sub_topic_left
        = ros_utils::readParam< std::string >( nh, prefix + "image_topic_left" );
        image_topic.push_back( image_sub_topic_left );

        std::string image_sub_topic_right
        = ros_utils::readParam< std::string >( nh, prefix + "image_topic_right" );
        image_topic.push_back( image_sub_topic_right );

        if ( image_sub_topic_left != image_sub_topic_right )
        {
            p_subImgs[2 * camera_index]
            = new ros_utils::ImageSubscriber( nh,
                                              image_sub_topic_left, //
                                              20,
                                              ros::TransportHints( ).tcpNoDelay( true ) );

            p_subImgs[2 * camera_index + 1]
            = new ros_utils::ImageSubscriber( nh,
                                              image_sub_topic_right, //
                                              20,
                                              ros::TransportHints( ).tcpNoDelay( true ) );

            ros_utils::App2ImgSynchronizer* sync
            = new ros_utils::App2ImgSynchronizer( ros_utils::AppSync2Images( 30 ),
                                                  *( p_subImgs[2 * camera_index] ), //
                                                  *( p_subImgs[2 * camera_index + 1] ) );

            sync->registerCallback( boost::bind( &StereoTracker::stereo_callback, //
                                                 this,
                                                 _1,
                                                 _2 ) );
        }
        else
        {
            ROS_DEBUG( "combined stereo images " );

            oneSubImgs
            = nh.subscribe< sensor_msgs::Image >( image_sub_topic_left,
                                                  10, //
                                                  &StereoTracker::stereo_callback1,
                                                  this,
                                                  ros::TransportHints( ).tcpNoDelay( true ) );
        }

        if ( tracker->isShowTrack( ) )
        {
            std::string cameraName = tracker->camera( )->cameraName( );
            pubTtrackImage = nh.advertise< sensor_msgs::Image >( "feature_" + cameraName, 1000 );
            //            cv::namedWindow( tracker->camera( )->cameraName( ),
            //            cv::WINDOW_NORMAL );
        }
    }

    FREQ = max_freq;

    pointsPub = nh.advertise< sensor_msgs::PointCloud >( "feature", 1000 );
}

void
StereoTracker::stereo_callback( const sensor_msgs::ImageConstPtr& img_msg_l,
                                const sensor_msgs::ImageConstPtr& img_msg_r )
{
    TicToc t_track;

    //    ROS_INFO( "Recieving image %lf", img_msg_l->header.stamp.toSec( ) );
    if ( first_image_flag )
    {
        first_image_flag = false;
        first_image_time = img_msg_l->header.stamp.toSec( );
    }

    // frequency control
    if ( round( 1.0 * detect_count / ( img_msg_l->header.stamp.toSec( ) - first_image_time ) ) <= FREQ )
    {
        frondendCtrl[0] = true;
        frondendCtrl[1] = true;
        // reset the frequency control
        if ( abs( 1.0 * detect_count / ( img_msg_l->header.stamp.toSec( ) - first_image_time ) - FREQ )
             < 0.01 * FREQ )
        {
            first_image_time = img_msg_l->header.stamp.toSec( );
            detect_count     = 0;
        }
    }
    else
    {
        frondendCtrl[0] = false;
        frondendCtrl[1] = false;
    }

    image_ptr[0] = cv_bridge::toCvCopy( img_msg_l, sensor_msgs::image_encodings::MONO8 );
    image_ptr[1] = cv_bridge::toCvCopy( img_msg_r, sensor_msgs::image_encodings::MONO8 );

    image_in_buf[0] = image_ptr[0]->image.rowRange( 0, tracker->row( ) );
    image_in_buf[1] = image_ptr[1]->image.rowRange( 0, tracker->row( ) );

    track( img_msg_l->header.stamp, frondendCtrl );

    ROS_INFO( "frontend all costs %f", t_track.toc( ) );
}

void
StereoTracker::stereo_callback1( const sensor_msgs::ImageConstPtr& img_msg )
{
    TicToc t_track;

    // ROS_INFO( "Recieving image %lf", img_msg->header.stamp.toSec( ) );
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

    image_ptr[0] = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::MONO8 );
    // image_ptr[1] = cv_bridge::toCvCopy( img_msg, sensor_msgs::image_encodings::MONO8 );

    image_in_buf[0] = image_ptr[0]->image.rowRange( 0, tracker->row( ) );
    image_in_buf[1] = image_ptr[0]->image.rowRange( tracker->row( ), //
                                                    2 * tracker->row( ) );

    track( img_msg->header.stamp, frondendCtrl );

    ROS_INFO( "frontend all costs %f", t_track.toc( ) );
}

void
StereoTracker::track( ros::Time now_t, bool* is_frondendCtrl )
{
    TicToc t_track;

    tracker->updateNewPoints( max_id );
    ROS_DEBUG( "updateNewPoints costs %f", t_track.toc( ) );

    tracker->readImages( image_in_buf );

    ROS_DEBUG( "readImages costs %f", t_track.toc( ) );

    tracker->process( is_frondendCtrl );
    ROS_DEBUG( "process costs %f", t_track.toc( ) );

    if ( frondendCtrl[1] )
    {
        detect_count++;
        pubFeature( now_t );
    }
    ROS_DEBUG( "pubFeature costs %f", t_track.toc( ) );

    if ( frondendCtrl[1] )
    {

        if ( tracker->isShowTrack( ) && tracker->isTracked( ) )
        {
            //            cv::Mat show_tmp_trans;
            cv::Mat image_show = tracker->draw( );
            //            cv::transpose( image_show, show_tmp_trans );
            cv_bridge::CvImage outImage;
            outImage.header.stamp    = now_t;
            outImage.header.frame_id = "track";
            outImage.encoding        = "bgr8";
            outImage.image           = image_show;
            pubTtrackImage.publish( outImage );
            //            cv::imshow( tracker->camera( )->cameraName( ), show_tmp_trans );
            //            cv::waitKey( 10 );
        }
    }
    ROS_DEBUG( "draw costs %f", t_track.toc( ) );
}

void
StereoTracker::pubFeature( ros::Time now_t )
{
    sensor_msgs::PointCloudPtr feature_points( new sensor_msgs::PointCloud );
    ros::Duration td( -0.0877 );
    feature_points->header.stamp    = now_t + td;
    feature_points->header.frame_id = "world";
    sensor_msgs::ChannelFloat32 error_of_point; // id channel of each object
    sensor_msgs::ChannelFloat32 id_of_point;    // id channel of each object
    sensor_msgs::ChannelFloat32 cams_of_point;  // id channel of each object

    if ( !tracker->isTracked( ) )
        return;

    std::vector< std::vector< Eigen::Vector3d > > un_pts;
    std::vector< std::vector< double > > angle_pts;
    std::vector< int > id;
    std::vector< uchar > status;

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

        //        if ( !status[j] )
        //            continue;

        p.x = un_pts[1][j]( 0 );
        p.y = un_pts[1][j]( 1 );
        p.z = un_pts[1][j]( 2 );
        feature_points->points.push_back( p );
        id_of_point.values.push_back( p_id );
        cams_of_point.values.push_back( 1 );
        error_of_point.values.push_back( angle_pts[1][j] );
    }

    feature_points->channels.push_back( id_of_point );
    feature_points->channels[0].name = "id_point";
    feature_points->channels.push_back( cams_of_point );
    feature_points->channels[1].name = "id_camera";
    feature_points->channels.push_back( error_of_point );
    feature_points->channels[2].name = "angle_per_pix";

    pointsPub.publish( feature_points );
}
