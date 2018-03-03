#ifndef FEATUREBASE_H
#define FEATUREBASE_H

#include "detectorfast.h"
#include "feature_utils.h"
#include "frontendcamera.h"
#include "frontendparameter.h"
#include "tic_toc.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

namespace frontend
{

class FeatureBase : public FrontendCamera
{
    public:
    FeatureBase( );
    FeatureBase( std::string camera_param_file, std::string feature_param_file );

    void readInPramFile( std::string camera_param_file, std::string feature_param_file );
    void updateNewPoints( unsigned int& max_id );
    void detectNewFeatures( const cv::Mat& image );
    void readInImage( const cv::Mat& image_in );
    void updateCntTrack( );
    void copyBack( cv::Mat image_this, std::vector< cv::Point2f > pts_this );
    void copyBack( cv::Mat image_this );
    void copyBack( );
    std::vector< Eigen::Vector3d > getUndistortedPoints( );
    std::vector< Eigen::Vector3d > getUndistortedPoints( Eigen::Matrix3d R_cc2 );

    public:
    cv::Mat image_pre( ) const;
    cv::Mat image_this( ) const;
    std::vector< cv::Point2f > getPtsThis( ) const;
    std::vector< cv::Point2f > getPtsPre( ) const;
    std::vector< int > getIdsPts( ) const;
    std::vector< int > getCntTrack( ) const;
    std::vector< cv::Point2f >& ptsThis( );
    std::vector< cv::Point2f >& ptsPre( );
    std::vector< int >& cntTrack( );
    std::vector< int >& idsPts( );

    bool isEqualize( ) const;
    bool isRansac( ) const;
    bool isTracked( ) const;
    bool isShowTrack( ) const;
    int freq( ) const;
    double F_Th( );
    FrontendType Type( );
    void setType( FrontendType type );

    cv::Ptr< cv::SparsePyrLKOpticalFlow > getSparseFlow( ) const;

    private:
    FrontendParameter m_param;
    DetectorFast* m_detector;
    cv::Ptr< cv::SparsePyrLKOpticalFlow > m_sparseFlow;

    cv::Mat m_image_pre;
    cv::Mat m_image_this;
    std::vector< cv::Point2f > m_pts_new;
    std::vector< cv::Point2f > m_pts_this;
    std::vector< cv::Point2f > m_pts_pre;
    std::vector< int > m_cnt_track;
    std::vector< int > m_ids_pts;
    bool m_tracked;
};
}

#endif // FEATUREBASE_H
