#ifndef STEREOTRACKER_H
#define STEREOTRACKER_H

#include "../feature_lib/frontendinit.h"
#include <code_utils/ros_utils.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud.h>

using namespace frontend;

class RosTrackStereo
{
    public:
    frontend::FrontendPtr tracker;
    std::vector< cv_bridge::CvImageConstPtr > image_ptr;
    std::vector< ros_utils::ImageSubscriber* > p_subImgs;
    ros::Publisher pubTtrackImage;
};

class StereoTracker : public RosTrackStereo
{
    public:
    StereoTracker( ) {}

    void readParameters( ros::NodeHandle nh );

    void stereo_callback( const sensor_msgs::ImageConstPtr& img_msg_l,
                          const sensor_msgs::ImageConstPtr& img_msg_r );

    void track( ros::Time now_t, bool* is_frondendCtrl );

    void pubFeature( ros::Time now_t );

    private:
    int NUM_OF_CAM;
    std::vector< cv::Mat > image_in_buf;

    ros::Publisher pointsPub;

    unsigned int max_id   = 0;
    bool first_image_flag = true;
    double first_image_time;
    int detect_count = 1;
    bool frondendCtrl[2]; // 0: DETECT_NEW_FEATURE 1:PUB_THIS_FRAME
    int FREQ;
};

#endif // STEREOTRACKER_H
