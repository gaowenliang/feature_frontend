#include "feature_frontend/dualFisheye/dualfisheyetracker.h"

#define OMP 1

using namespace ros_utils;

DualFisheyeTracker::DualFisheyeTracker( ) { FREQ = 100; }

DualFisheyeTracker::DualFisheyeTracker( ros::NodeHandle nh )
{
    FREQ = 100;
    readParameters( nh );
}

void
DualFisheyeTracker::readParameters( ros::NodeHandle nh )
{

    NumOfStereo = readParam< int >( nh, "camera_num_stereo" );
    NumOfMono   = readParam< int >( nh, "camera_num_mono" );
    NumOfCamera = NumOfStereo + NumOfMono;

    p_Trackers.resize( NumOfCamera );

    int maxFreq     = 0;
    int cameraIndex = 0;
    for ( auto& tracker : p_Trackers )
    {
        std::string prefix = boost::str( boost::format( "camera%d/" ) % cameraIndex );

        tracker = new RosTrack( );
        std::vector< std::string > cam_files;
        std::vector< std::string > feature_files;

        if ( cameraIndex < NumOfStereo )
        {
            cam_files.push_back(
            ros_utils::readParam< std::string >( nh, prefix + "cam_config_file_left" ) );
            cam_files.push_back(
            ros_utils::readParam< std::string >( nh, prefix + "cam_config_file_right" ) );

            feature_files.push_back(
            ros_utils::readParam< std::string >( nh, //
                                                 prefix + "feature_config_file" ) );
            feature_files.push_back(
            ros_utils::readParam< std::string >( nh, //
                                                 prefix + "feature_config_file" ) );

            tracker->image_in_buf.resize( 2 );

            tracker->tracker = frontend::FrontendInit::newFrontend( )->init( frontend::STEREO, //
                                                                             cam_files,
                                                                             feature_files );
        }
        else
        {
            cam_files.push_back(
            ros_utils::readParam< std::string >( nh, prefix + "cam_config_file" ) );
            feature_files.push_back(
            ros_utils::readParam< std::string >( nh, prefix + "feature_config_file" ) );

            tracker->image_in_buf.resize( 1 );

            tracker->tracker = frontend::FrontendInit::newFrontend( )->init( frontend::MONO, //
                                                                             cam_files,
                                                                             feature_files );
        }

        if ( tracker->tracker->isShowTrack( ) )
        {
            std::string cameraName = tracker->tracker->camera( )->cameraName( );

            tracker->pubTtrackImage
            = nh.advertise< sensor_msgs::Image >( "feature_" + cameraName, 1000 );
            //                cv::namedWindow( cameraName, cv::WINDOW_NORMAL );
        }

        maxFreq = maxFreq > tracker->tracker->freq( ) ? maxFreq : tracker->tracker->freq( );

        std::string imageSubTopic = readParam< std::string >( nh, prefix + "image_topic" );

        tracker->p_subImgs = new ImageSubscriber( nh,
                                                  imageSubTopic, //
                                                  20,
                                                  ros::TransportHints( ).tcpNoDelay( true ) );
        ++cameraIndex;
    }

    FREQ = maxFreq;

    App6ImgSynchronizer* sync = new App6ImgSynchronizer( AppSync6Images( 30 ),
                                                         *( p_Trackers[0]->p_subImgs ), //
                                                         *( p_Trackers[1]->p_subImgs ),
                                                         *( p_Trackers[2]->p_subImgs ),
                                                         *( p_Trackers[3]->p_subImgs ),
                                                         *( p_Trackers[4]->p_subImgs ),
                                                         *( p_Trackers[5]->p_subImgs ) );

    sync->registerCallback( boost::bind( &DualFisheyeTracker::multi_mix_callback, //
                                         this,
                                         _1,
                                         _2,
                                         _3,
                                         _4,
                                         _5,
                                         _6 ) );

    pubPoints = nh.advertise< sensor_msgs::PointCloud >( "feature", 1000 );
}

void
DualFisheyeTracker::multi_mix_callback( const sensor_msgs::ImageConstPtr& imgMsgL,
                                        const sensor_msgs::ImageConstPtr& imgMsgF,
                                        const sensor_msgs::ImageConstPtr& imgMsgR,
                                        const sensor_msgs::ImageConstPtr& imgMsgB,
                                        const sensor_msgs::ImageConstPtr& imgMsgU,
                                        const sensor_msgs::ImageConstPtr& imgMsgD )
{
    TicToc t_track;

    //    ROS_INFO( "Recieving image %lf", img_msg->header.stamp.toSec( ) );
    if ( firstImageFlag )
    {
        firstImageFlag = false;
        firstImageTime = imgMsgL->header.stamp.toSec( );
    }

    // frequency control
    if ( round( 1.0 * detectCount / ( imgMsgL->header.stamp.toSec( ) - firstImageTime ) ) <= FREQ )
    {
        frondendCtrl[0] = true;
        frondendCtrl[1] = true;
        // reset the frequency control
        if ( abs( 1.0 * detectCount / ( imgMsgL->header.stamp.toSec( ) - firstImageTime ) - FREQ )
             < 0.01 * FREQ )
        {
            firstImageTime = imgMsgL->header.stamp.toSec( );
            detectCount    = 0;
        }
    }
    else
    {
        frondendCtrl[0] = false;
        frondendCtrl[1] = false;
    }

    p_Trackers[0]->image_ptr = cv_bridge::toCvCopy( imgMsgL, sensor_msgs::image_encodings::MONO8 );
    p_Trackers[1]->image_ptr = cv_bridge::toCvCopy( imgMsgF, sensor_msgs::image_encodings::MONO8 );
    p_Trackers[2]->image_ptr = cv_bridge::toCvCopy( imgMsgR, sensor_msgs::image_encodings::MONO8 );
    p_Trackers[3]->image_ptr = cv_bridge::toCvCopy( imgMsgB, sensor_msgs::image_encodings::MONO8 );
    p_Trackers[4]->image_ptr = cv_bridge::toCvCopy( imgMsgU, sensor_msgs::image_encodings::MONO8 );
    p_Trackers[5]->image_ptr = cv_bridge::toCvCopy( imgMsgD, sensor_msgs::image_encodings::MONO8 );

    for ( auto& tracker : p_Trackers )
    {
        if ( tracker->tracker->Type( ) == frontend::STEREO )
        {
            tracker->image_in_buf[0]
            = tracker->image_ptr->image.rowRange( 0, tracker->tracker->row( ) );
            tracker->image_in_buf[1]
            = tracker->image_ptr->image.rowRange( tracker->tracker->row( ), //
                                                  2 * tracker->tracker->row( ) );
        }
        else
            tracker->image_in_buf[0]
            = tracker->image_ptr->image.rowRange( 0, tracker->tracker->row( ) );
    }

    track( imgMsgL->header.stamp, frondendCtrl );

    ROS_INFO( "Frontend costs %f", t_track.toc( ) );
}

void
DualFisheyeTracker::track( ros::Time now_t, bool* is_frondendCtrl )
{
    TicTocPart t_track;

    {
        for ( auto& tracker : p_Trackers )
            tracker->tracker->updateNewPoints( max_id );
        ROS_DEBUG( "updateNewPoints costs %f", t_track.toc( ) );
    }

    int camera_index = 0;

#if OMP
#pragma omp parallel for
#endif
    for ( camera_index = 0; camera_index < NumOfCamera; ++camera_index )
        p_Trackers[camera_index]->tracker->readImages( p_Trackers[camera_index]->image_in_buf );

#if OMP
#pragma omp parallel for
#endif
    for ( camera_index = 0; camera_index < NumOfCamera; ++camera_index )
    {
        p_Trackers[camera_index]->tracker->process( is_frondendCtrl );
    }
    ROS_DEBUG( "process costs %f", t_track.toc( ) );

    if ( is_frondendCtrl[1] )
    {
        detectCount++;
        pubFeature( now_t );
    }
    ROS_DEBUG( "pubFeature costs %f", t_track.toc( ) );

    if ( is_frondendCtrl[1] )
    {
        bool is_show_tmp = false;

#if OMP
#pragma omp parallel for
#endif
        for ( camera_index = 0; camera_index < NumOfCamera; ++camera_index )
        {
            //            if ( t_track.tocEnd( ) > 1 / FREQ * 1000 )
            //                continue;

            auto tracker = p_Trackers[camera_index];
            if ( tracker->tracker->isShowTrack( ) && tracker->tracker->isTracked( ) )
            {
                cv::Mat show_tmp_trans;
                cv::Mat image_show = tracker->tracker->draw( );
                //                cv::transpose( image_show, show_tmp_trans );

                cv_bridge::CvImage outImage;
                outImage.header.stamp    = now_t;
                outImage.header.frame_id = "track";
                outImage.encoding        = "bgr8";
                outImage.image           = image_show;
                tracker->pubTtrackImage.publish( outImage );

                //                cv::imshow( tracker->camera( )->cameraName( ),
                //                show_tmp_trans );
                is_show_tmp |= true;
            }
        }

        //        if ( is_show_tmp )
        //            cv::waitKey( 10 );
    }
    ROS_DEBUG( "draw costs %f", t_track.toc( ) );
}

void
DualFisheyeTracker::pubFeature( ros::Time now_t )
{
    sensor_msgs::PointCloudPtr feature_points( new sensor_msgs::PointCloud );
    feature_points->header.stamp    = now_t;
    feature_points->header.frame_id = "world";
    sensor_msgs::ChannelFloat32 error_of_point; // id channel of each object
    sensor_msgs::ChannelFloat32 id_of_point;    // id channel of each object
    sensor_msgs::ChannelFloat32 cams_of_point;  // id channel of each object
    bool tracked = false;

    int cnt_num = 0;

    int total_num    = 0;
    int camera_index = -1;
    for ( auto& tracker : p_Trackers )
    {
        ++camera_index;

        if ( !tracker->tracker->isTracked( ) )
            continue;
        else
            tracked |= true;

        std::vector< std::vector< Eigen::Vector3d > > un_pts;
        std::vector< std::vector< double > > angle_pts;
        std::vector< int > id;
        //        tracker->tracker->getPoints( un_pts, id );
        tracker->tracker->getPoints( un_pts, angle_pts, id );

        total_num += id.size( );

        for ( unsigned int j = 0; j < id.size( ); j++ )
        {
            int p_id = id[j];

            //    std::cout << "un_pts_r " << un_pts[j].x( ) << " " <<
            //    un_pts[j].y( ) << std::endl;
            //  std::cout << "id " << p_id << std::endl;
            //  std::cout << "un_pts_r " << un_pts_r[j].x( ) << " " <<
            //  un_pts_r[j].y( ) << std::endl;

            geometry_msgs::Point32 pt;
            pt.x = un_pts[0][j]( 0 );
            pt.y = un_pts[0][j]( 1 );
            pt.z = un_pts[0][j]( 2 );

            //            ROS_DEBUG( "point_out--  %lf %lf %lf", p.x, p.y, p.z );

            if ( tracker->tracker->Type( ) == frontend::STEREO )
            {
                //                continue;

                feature_points->points.push_back( pt );
                id_of_point.values.push_back( p_id );
                error_of_point.values.push_back( angle_pts[0][j] );

                cams_of_point.values.push_back( 0 );

                geometry_msgs::Point32 pt_r;
                pt_r.x = un_pts[1][j]( 0 );
                pt_r.y = un_pts[1][j]( 1 );
                pt_r.z = un_pts[1][j]( 2 );

                feature_points->points.push_back( pt_r );
                id_of_point.values.push_back( p_id );
                error_of_point.values.push_back( angle_pts[1][j] );

                cams_of_point.values.push_back( 1 );
            }
            else if ( tracker->tracker->Type( ) == frontend::MONO )
            {
                //                continue;

                feature_points->points.push_back( pt );
                id_of_point.values.push_back( p_id );
                error_of_point.values.push_back( angle_pts[0][j] );

                cams_of_point.values.push_back( camera_index - NumOfStereo );

                if ( ( camera_index - NumOfStereo ) == 1 )
                    ++cnt_num;
            }
        }
    }

    feature_points->channels.push_back( id_of_point );
    feature_points->channels[0].name = "id_point";
    feature_points->channels.push_back( cams_of_point );
    feature_points->channels[1].name = "id_camera";
    feature_points->channels.push_back( error_of_point );
    feature_points->channels[2].name = "angle_per_pix";

    ROS_DEBUG( "cnt_num %d", cnt_num );

    ROS_DEBUG( "all feature size %d", total_num );
    if ( tracked )
        pubPoints.publish( feature_points );
}
