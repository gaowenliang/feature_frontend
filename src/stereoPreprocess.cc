#include <code_utils/cv_utils.h>
#include <code_utils/sys_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

ros::Publisher image_pub_0;
ros::Publisher image_pub_1;
ros::Subscriber image_sub;
ros::Subscriber image_sub_1;

cv_utils::fisheye::PreProcess* preprocess;

bool is_image_show       = false;
double down_sample_scale = 0.5;

void
callback_0( const sensor_msgs::Image::ConstPtr& img )
{
    double time_start = sys_utils::timeInSeconds( );

    //    ros::Duration dt = ros::Time::now( ) - img->header.stamp;
    //    std::cout << "dt " << dt.toSec( ) * 1000 << " ms" << std::endl;

    cv::Mat image_input = cv_bridge::toCvCopy( img, "mono8" )->image;

    cv_bridge::CvImage outImg;
    outImg.header   = img->header;
    outImg.encoding = img->encoding;
    outImg.image    = preprocess->do_preprocess( image_input );
    image_pub_0.publish( outImg );

    if ( is_image_show )
    {
        cv::imshow( "src_0", image_input );
        cv::imshow( " outImg.image0", outImg.image );
        std::cout << " outImg.image0 " << outImg.image.rows << " " << outImg.image.cols << std::endl;
        std::cout << "cost: " << ( sys_utils::timeInSeconds( ) - time_start ) * 1000 << std::endl;
        cv::waitKey( 20 );
    }
    //    std::cout << "cost: " << ( sys_utils::timeInSeconds( ) - time_start ) * 1000 <<
    //    std::endl;
}

void
callback_1( const sensor_msgs::Image::ConstPtr& img )
{
    double time_start = sys_utils::timeInSeconds( );

    cv::Mat image_input = cv_bridge::toCvCopy( img, "mono8" )->image;

    cv_bridge::CvImage outImg;
    outImg.header   = img->header;
    outImg.encoding = img->encoding;
    outImg.image    = preprocess->do_preprocess( image_input );
    image_pub_1.publish( outImg );

    if ( is_image_show )
    {
        cv::imshow( "src_1", image_input );
        cv::imshow( " outImg.image1", outImg.image );
        std::cout << " outImg.image1 " << outImg.image.rows << " " << outImg.image.cols << std::endl;
        std::cout << "cost: " << ( sys_utils::timeInSeconds( ) - time_start ) * 1000 << std::endl;
        cv::waitKey( 20 );
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "test_pub" );
    ros::NodeHandle n( "~" );

    n.param( "down_sample_scale", down_sample_scale, 1.0 );
    n.param( "is_image_show", is_image_show, false );

    preprocess = new cv_utils::fisheye::PreProcess( cv::Size( 1280, 1024 ),
                                                    cv::Size( 600, 600 ),
                                                    cv::Point( 1280 / 2, 1024 / 2 ),
                                                    down_sample_scale );

    image_pub_0 = n.advertise< sensor_msgs::Image >( "/stereo/left/image", 1 );
    image_pub_1 = n.advertise< sensor_msgs::Image >( "/stereo/right/image", 1 );

    image_sub = n.subscribe< sensor_msgs::Image >( "/stereo/left/image_raw",
                                                   1,
                                                   callback_0,
                                                   ros::TransportHints( ).tcpNoDelay( ) );
    image_sub_1 = n.subscribe< sensor_msgs::Image >( "/stereo/right/image_raw",
                                                     1,
                                                     callback_1,
                                                     ros::TransportHints( ).tcpNoDelay( ) );

    ros::spin( );

    return 0;
}
