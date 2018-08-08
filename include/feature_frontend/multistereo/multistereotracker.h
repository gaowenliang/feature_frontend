#ifndef MULTISTEREOTRACKER_H
#define MULTISTEREOTRACKER_H

#include "../feature_lib/frontendinit.h"
#include <code_utils/ros_utils.h>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud.h>

using namespace frontend;

class RosTrack
{
    public:
    RosTrack( ) {}
    frontend::FrontendPtr tracker;
    std::vector< cv::Mat > image_in_buf;
    ros_utils::ImageSubscriber* p_subImgs;
    cv_bridge::CvImageConstPtr image_ptr;
    ros::Publisher pubTtrackImage;
};

class MultiStereoTracker
{
    public:
    MultiStereoTracker( );
    MultiStereoTracker( ros::NodeHandle nh );

    void readParameters( ros::NodeHandle nh );

    void multi4_callback( const sensor_msgs::ImageConstPtr& imgMsgL,
                          const sensor_msgs::ImageConstPtr& imgMsgF,
                          const sensor_msgs::ImageConstPtr& imgMsgR,
                          const sensor_msgs::ImageConstPtr& imgMsgB );
    void multi3_callback( const sensor_msgs::ImageConstPtr& imgMsgL,
                          const sensor_msgs::ImageConstPtr& imgMsgF,
                          const sensor_msgs::ImageConstPtr& imgMsgR );
    void multi2_callback( const sensor_msgs::ImageConstPtr& imgMsgL,
                          const sensor_msgs::ImageConstPtr& imgMsgF );

    void track( ros::Time now_t, bool* is_frondendCtrl );

    void pubFeature( ros::Time now_t );

    private:
    int NumOfStereo;
    std::vector< RosTrack* > p_Trackers;
    ros::Publisher pubPoints;

    unsigned int max_id = 0;
    bool firstImageFlag = true;
    double firstImageTime;
    int detectCount = 1;
    bool frondendCtrl[2]; // 0: DETECT_NEW_FEATURE 1:PUB_THIS_FRAME
    int FREQ;
};

#endif // DUALFISHEYETRACKER_H
