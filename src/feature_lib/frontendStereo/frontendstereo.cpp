#include "feature_frontend/feature_lib/frontendStereo/frontendstereo.h"

using namespace frontend;

FeatureStereoTracker::FeatureStereoTracker( std::string left_cam_file,
                                            std::string right_cam_file,
                                            std::string left_feature_file,
                                            std::string right_feature_file )
{
    this->setType( frontend::STEREO );
    this->readInPramFile( left_cam_file, left_feature_file );

    m_tracker_r = new FeatureBase( );
    m_tracker_r->readInPramFile( right_cam_file, right_feature_file );
    m_sparseFlowLeft2Right = cv::SparsePyrLKOpticalFlow::create( );

    int widthEdge = 5;
    rightRect     = cv::Rect( widthEdge,
                          widthEdge,
                          m_tracker_r->camera( )->imageWidth( ) - widthEdge,
                          m_tracker_r->camera( )->imageHeight( ) - widthEdge );
}

void
FeatureStereoTracker::process( bool* frontendCtrl )
{
    bool is_detect_new_feature = frontendCtrl[0];
    bool is_track_right        = frontendCtrl[1];

    TicTocPart t_track;

    if ( image_this( ).empty( ) || m_tracker_r->image_this( ).empty( ) )
        return;

    cv::Mat left_image;
    cv::Mat right_image;

    if ( isEqualize( ) )
    {
        // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        cv::Ptr< cv::CLAHE > clahe = cv::createCLAHE( );
        TicToc t_c;
        clahe->apply( image_this( ), left_image );
        clahe->apply( m_tracker_r->image_this( ), right_image );
        ROS_DEBUG( "CLAHE costs: %fms", t_c.toc( ) );
    }
    else
    {
        left_image  = image_this( );
        right_image = m_tracker_r->image_this( );
    }

    if ( !ptsPre( ).empty( ) )
        trackFeatures( left_image, right_image, is_track_right );

    if ( is_detect_new_feature )
        detectNewFeatures( left_image );

    copyBack( left_image );

    ROS_DEBUG( "track costs %f", t_track.toc( ) );

    ROS_DEBUG_STREAM( "---" << camera( )->cameraName( ) << " costs " << t_track.tocEnd( ) );
}

void
FeatureStereoTracker::trackFeatures( const cv::Mat& left_image, const cv::Mat& right_image, bool is_track_right )
{
    std::vector< uchar > left_status, right_status;
    std::vector< float > left_err, right_err;

    // track from last image to current image
    //    cv::calcOpticalFlowPyrLK( image_pre( ),
    //                              left_image, //
    //                              ptsPre( ),
    //                              ptsThis( ),
    //                              left_status,
    //                              left_err,
    //                              cv::Size( 21, 21 ),
    //                              3 );

    getSparseFlow( )->calc( image_pre( ), left_image, ptsPre( ), ptsThis( ), left_status, left_err );
    if ( isRansac( ) && !ptsThis( ).empty( ) && !left_status.empty( ) )
    {
        cv::Mat F_fund;
        std::vector< uchar > F_status;
        F_fund = cv::findFundamentalMat( ptsPre( ), ptsThis( ), cv::FM_RANSAC, 0.5, 0.99, F_status );
        reduce_vector( ptsThis( ), F_status );
        reduce_vector( cntTrack( ), F_status );
        reduce_vector( idsPts( ), F_status );

        if ( isShowTrack( ) )
            reduce_vector( ptsPre( ), F_status );
    }

    if ( is_track_right && !ptsThis( ).empty( ) )
    {
        // track from left image to right image
        // cv::calcOpticalFlowPyrLK( left_image,
        //                           right_image,
        //                           ptsThis( ),
        //                           m_tracker_r->ptsThis( ), //
        //                           right_status,
        //                           right_err,
        //                           cv::Size( 15, 15 ),
        //                           3 );
        m_sparseFlowLeft2Right->calc( left_image,
                                      right_image,
                                      ptsThis( ),
                                      m_tracker_r->ptsThis( ), //
                                      right_status,
                                      right_err );

        if ( isRansac( ) && !m_tracker_r->ptsThis( ).empty( ) )
        {

            cv::Mat R_fund;
            std::vector< uchar > R_status;
            if ( !ptsThis( ).empty( ) //
                 && !m_tracker_r->ptsThis( ).empty( ) )
            {
                R_fund = cv::findFundamentalMat( ptsThis( ),
                                                 m_tracker_r->ptsThis( ), //
                                                 cv::FM_RANSAC,
                                                 3.0,
                                                 0.99,
                                                 R_status );

                for ( loop_index_t index = 0; index < loop_index_t( m_tracker_r->ptsThis( ).size( ) ); ++index )
                    if ( R_status[index] )
                        if ( !rightRect.contains( m_tracker_r->ptsThis( )[index] ) ) // max y
                        {
                            R_status[index] = 0;
                        }
                        else
                        {
                            float x = ( ptsThis( )[index] - m_tracker_r->ptsThis( )[index] ).x;
                            float y = ( ptsThis( )[index] - m_tracker_r->ptsThis( )[index] ).y;
                            //  float dis = sqrt( x * x + y * y );
                            //  std::cout << "id " << ids[i] << " dis " << dis
                            //  << " depth "
                            //            << 0.25 * 204.5 / dis << std::endl;

                            float dis_sqre = x * x + y * y;
                            if ( dis_sqre > 6400 ) // 80 pixcel, 0.69 m
                                R_status[index] = 0;
                        }
                    else
                        continue;

                reduce_vector( ptsThis( ), R_status );
                reduce_vector( m_tracker_r->ptsThis( ), R_status );
                reduce_vector( cntTrack( ), R_status );
                reduce_vector( idsPts( ), R_status );
                if ( isShowTrack( ) )
                    reduce_vector( ptsPre( ), R_status );
            }
        }
    }

    updateCntTrack( );
}

cv::Mat
FeatureStereoTracker::draw( )
{
    if ( !image_pre( ).empty( ) //
         && !getPtsPre( ).empty( )
         && !m_tracker_r->getPtsThis( ).empty( ) )
    {
        int height = row( );
        int width  = col( );

        cv::Mat output( height * 2, width * 2, CV_8UC3, cv::Scalar( 0, 0, 0 ) );

        cv::cvtColor( image_pre( ), output( cv::Rect( 0, 0, width, height ) ), CV_GRAY2BGR );
        cv::cvtColor( image_this( ), output( cv::Rect( width, 0, width, height ) ), CV_GRAY2BGR );
        cv::cvtColor( m_tracker_r->image_this( ), output( cv::Rect( width, height, width, height ) ), CV_GRAY2BGR );

        std::vector< cv::Point2f > pts_pre_l  = getPtsPre( );
        std::vector< cv::Point2f > pts_this_l = getPtsThis( );
        std::vector< cv::Point2f > pts_this_r = m_tracker_r->getPtsThis( );
        std::vector< int > track_cnt          = getCntTrack( );

        // previous image to this image
        for ( loop_index_t i = 0; i < loop_index_t( pts_pre_l.size( ) ); i++ )
        {
            cv::Point2f left = pts_pre_l[i];
            cv::Point2f right( pts_this_l[i].x + width, pts_this_l[i].y );
            cv::line( output, left, right, cv::Scalar( 255, 0, 0 ), 1, cv::LINE_AA );
        }

        // left image to this image
        for ( loop_index_t i = 0; i < pts_pre_l.size( ); i++ )
        {
            cv::Point2f left( pts_this_l[i].x + width, pts_this_l[i].y );
            cv::Point2f right( pts_this_r[i].x + width, pts_this_r[i].y + height );
            cv::line( output, left, right, cv::Scalar( 255, 0, 0 ), 1, cv::LINE_AA );
        }

        for ( size_t i = 0; i < pts_pre_l.size( ); i++ )
        {
            // float x   = ( pts_this_l[i] - pts_this_r[i] ).x;
            // float y   = ( pts_this_l[i] - pts_this_r[i] ).y;
            // float dis = sqrt( x * x + y * y );
            // std::cout << "id " << ids[i] << " dis " << dis << " depth " << 0.25 * 204.5 /
            // dis
            // << std::endl;

            cv::Point2f last = pts_pre_l[i];
            cv::Point2f left( pts_this_l[i].x + width, pts_this_l[i].y );
            cv::Point2f right( pts_this_r[i].x + width, pts_this_r[i].y + height );

            double len = std::min( 1.0, 1.0 * track_cnt[i] / 10 );
            //                double len = std::min( 1.0, 1.0 * track_cnt[i] / WINDOW_SIZE );

            cv::circle( output, last, 1, cv::Scalar( 255 * ( 1 - len ), 0, 255 * len ), 2 );
            cv::circle( output, left, 1, cv::Scalar( 0, 255, 0 ), 2 );
            cv::circle( output, right, 1, cv::Scalar( 0, 255, 255 ), 2 );
        }
        ptsThis( ).clear( );

        return output;
    }
    else
    {
        ptsThis( ).clear( );

        return cv::Mat( );
    }
}

void
FeatureStereoTracker::readImages( const std::vector< cv::Mat >& image_buf )
{
    readInImage( image_buf[0] );
    m_tracker_r->readInImage( image_buf[1] );
}

void
FeatureStereoTracker::getPoints( std::vector< std::vector< Eigen::Vector3d > >& points, std::vector< int >& id )
{
    points.push_back( getUndistortedPoints( ) );
    points.push_back( m_tracker_r->getUndistortedPoints( ) );
    id = getIdsPts( );
}

std::vector< Eigen::Vector3d >
FeatureStereoTracker::getUndistortedPointsRight( )
{
    return m_tracker_r->getUndistortedPoints( );
}
