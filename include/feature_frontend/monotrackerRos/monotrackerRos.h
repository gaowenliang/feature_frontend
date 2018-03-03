#ifndef MONOTRACKER_H
#define MONOTRACKER_H

#include "../feature_lib/frontendinit.h"
#include <code_utils/ros_utils.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>

using namespace frontend;

class RosTrackMono
{
    public:
    frontend::FrontendPtr tracker;
    cv_bridge::CvImageConstPtr image_ptr;
    ros::Publisher pubTtrackImage;
    ros::Subscriber subImgs;
};

class MonoTracker : public RosTrackMono
{
    public:
    MonoTracker( );

    void readParameters( ros::NodeHandle nh );

    void image_callback( const sensor_msgs::ImageConstPtr& img_msg );

    void track( ros::Time now_t, bool* is_frondendCtrl );

    void pubFeature( ros::Time now_t );

    private:
    std::vector< cv::Mat > image_in_buf;
    ros::Publisher pointsPub;

    unsigned int max_id;
    bool first_image_flag = true;
    double first_image_time;
    int detect_count = 1;
    bool frondendCtrl[2]; // 0: DETECT_NEW_FEATURE 1:PUB_THIS_FRAME
    int FREQ;
};

#endif // MONOTRACKER_H
