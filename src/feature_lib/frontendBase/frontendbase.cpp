#include "feature_frontend/feature_lib/frontendBase/frontendbase.h"
#include "opencv2/features2d.hpp"

using namespace frontend;

FeatureBase::FeatureBase( )
: m_tracked( false )
{
    m_detector   = new DetectorFast( );
    m_sparseFlow = cv::SparsePyrLKOpticalFlow::create( );
}

FeatureBase::FeatureBase( std::string camera_param_file, std::string feature_param_file )
: m_tracked( false )
{
    readInPramFile( camera_param_file, feature_param_file );
}

void
FeatureBase::readInPramFile( std::string camera_param_file, std::string feature_param_file )
{
    readCameraFromYamlFile( camera_param_file );

    m_param.readFromYamlFile( feature_param_file );

    m_detector = new DetectorFast( m_param.m_min_dist );
    m_detector->initMaskWithSquareNum( m_param.m_mask,
                                       3,
                                       3,
                                       camera( )->imageWidth( ), //
                                       camera( )->imageHeight( ) );

    m_sparseFlow = cv::SparsePyrLKOpticalFlow::create( );
}

void
FeatureBase::updateNewPoints( unsigned int& max_id )
{
    ROS_DEBUG_STREAM( "addadd size " << m_pts_new.size( ) );

    if ( !m_pts_new.empty( ) )
    {
        for ( auto& pt : m_pts_new )
        {
            m_pts_pre.push_back( pt );
            m_ids_pts.push_back( max_id++ );
            m_cnt_track.push_back( 1 );
        }
        m_pts_new.clear( );
    }
}

void
FeatureBase::detectNewFeatures( const cv::Mat& image )
{
    TicTocPart t_track;

    int new_num = m_param.m_max_cnt - m_pts_this.size( );
    //    std::cout << "m_param.m_max_cnt " << m_param.m_max_cnt << std::endl;
    //    std::cout << "new_num " << new_num << std::endl;
    //    new_num = m_param.m_max_cnt;

    if ( new_num <= 0 )
        return;
    m_pts_new.clear( );

    // cv::goodFeaturesToTrack( image, m_pts_new, new_num, 0.1, m_param.m_min_dist, cv::Mat( )
    // );
    //    ROS_DEBUG( "goodFeaturesToTrack costs %f", t_track.toc( ) );

    m_detector->detectNewFeatures( image, m_pts_this, new_num, m_pts_new );

    ROS_DEBUG_STREAM( camera( )->cameraName( ) << " FastFeatureDetector costs " << t_track.toc( ) << " ms, add size "
                                               << m_pts_new.size( )
                                               << " total size "
                                               << m_pts_this.size( ) );
}

void
FeatureBase::readInImage( const cv::Mat& image_in )
{
    if ( image_in.empty( ) )
        return;
    m_image_this = image_in;
}

void
FeatureBase::updateCntTrack( )
{
    for ( auto& n : m_cnt_track )
        n++;
}

std::vector< Eigen::Vector3d >
FeatureBase::getUndistortedPoints( )
{
    return undistortedPoints( getPtsThis( ) );
}

std::vector< Eigen::Vector3d >
FeatureBase::getUndistortedPoints( Eigen::Matrix3d R_cc2 )
{
    setR_cc2( R_cc2 );
    return undistortedPoints( getPtsThis( ) );
}

cv::Mat
FeatureBase::image_pre( ) const
{
    return m_image_pre;
}

cv::Mat
FeatureBase::image_this( ) const
{
    return m_image_this;
}

std::vector< int >
FeatureBase::getIdsPts( ) const
{
    return m_ids_pts;
}

bool
FeatureBase::isTracked( ) const
{
    return !m_pts_pre.empty( );
}

bool
FeatureBase::isShowTrack( ) const
{
    return m_param.m_is_SHOW_TRACK;
}

int
FeatureBase::freq( ) const
{
    return m_param.m_freq;
}

double
FeatureBase::F_Th( )
{
    return m_param.m_F_THRESHOLD;
}

FrontendType
FeatureBase::Type( )
{
    return m_param.m_type;
}

void
FeatureBase::setType( FrontendType type )
{
    m_param.m_type = type;
}

cv::Ptr< cv::SparsePyrLKOpticalFlow >
FeatureBase::getSparseFlow( ) const
{
    return m_sparseFlow;
}

std::vector< int >
FeatureBase::getCntTrack( ) const
{
    return m_cnt_track;
}

std::vector< cv::Point2f >&
FeatureBase::ptsThis( )
{
    return m_pts_this;
}

std::vector< cv::Point2f >&
FeatureBase::ptsPre( )
{
    return m_pts_pre;
}

std::vector< int >&
FeatureBase::cntTrack( )
{
    return m_cnt_track;
}

bool
FeatureBase::isEqualize( ) const
{
    return m_param.m_is_EQUALIZE;
}

bool
FeatureBase::isRansac( ) const
{
    return m_param.m_is_RANSAC;
}

std::vector< cv::Point2f >
frontend::FeatureBase::getPtsThis( ) const
{
    return m_pts_this;
}

std::vector< cv::Point2f >
FeatureBase::getPtsPre( ) const
{
    return m_pts_pre;
}

void
frontend::FeatureBase::copyBack( cv::Mat image_this, std::vector< cv::Point2f > pts_this )
{
    m_image_pre = image_this;
    m_pts_pre   = pts_this;
}

void
FeatureBase::copyBack( cv::Mat image_this )
{
    m_image_pre = image_this;
    m_pts_pre   = m_pts_this;
}

void
FeatureBase::copyBack( )
{
    m_image_pre = m_image_this;
    m_pts_pre   = m_pts_this;
}

std::vector< int >&
frontend::FeatureBase::idsPts( )
{
    return m_ids_pts;
}
