#include "feature_frontend/feature_lib/frontendMono/frontendmono.h"

using namespace frontend;

FeatureMonoTracker::FeatureMonoTracker( std::string cam_file, std::string feature_file )
{
    this->setType( frontend::MONO );
    this->readInPramFile( cam_file, feature_file );
}

FeatureMonoTracker::FeatureMonoTracker( std::string cam_file, std::string err_file, std::string feature_file )
{
    this->setType( frontend::MONO );
    this->readInPramFile( cam_file, err_file, feature_file );
}

void
FeatureMonoTracker::process( bool* is_detect_new_feature )
{
    TicTocPart t_track;

    if ( image_this( ).empty( ) )
        return;

    cv::Mat image;

    if ( isEqualize( ) )
    {
        // cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        cv::Ptr< cv::CLAHE > clahe = cv::createCLAHE( );
        clahe->apply( image_this( ), image );
        ROS_DEBUG( "CLAHE costs: %fms", t_track.toc( ) );
    }
    else
    {
        image = image_this( );
    }

    if ( !ptsPre( ).empty( ) )
        trackFeatures( image );
    ROS_DEBUG( "trackFeatures costs %f", t_track.toc( ) );

    if ( is_detect_new_feature[0] ) // detect new feature
        detectNewFeatures( image );
    ROS_DEBUG( "detectNewFeatures costs %f", t_track.toc( ) );

    copyBack( image );
    ROS_DEBUG( "copyBack costs %f", t_track.toc( ) );

    ROS_DEBUG_STREAM( "---" << camera( )->cameraName( ) << " costs " << t_track.tocEnd( ) );
}

void
FeatureMonoTracker::trackFeatures( const cv::Mat& image )
{
    std::vector< uchar > status;
    std::vector< float > err;

    TicTocPart t_track;

    // track from last image to current image

    ptsThis( ).clear( );

    getSparseFlow( )->calc( image_pre( ), image, ptsPre( ), ptsThis( ), status, err );
    ROS_DEBUG( "SparsePyrLKOpticalFlow costs %f", t_track.toc( ) );

    if ( isRansac( ) && !ptsThis( ).empty( ) && !status.empty( ) )
    {
        rejectWithF( status );

        //  cv::Mat F_fund;
        //  std::vector< uchar > F_status;
        //  if ( ptsPre( ).size( ) != 0 && ptsThis( ).size( ) != 0 )
        //      F_fund = cv::findFundamentalMat( ptsPre( ), ptsThis( ), cv::FM_RANSAC, 0.5,
        //      0.99, F_status );
        //
        //  reduce_vector( ptsPre( ), F_status );
        //  reduce_vector( ptsThis( ), F_status );
        //  reduce_vector( idsPts( ), F_status );
        //  reduce_vector( cntTrack( ), F_status );
    }

    updateCntTrack( );
}

void
FeatureMonoTracker::rejectWithF( const std::vector< uchar > status_track )
{
    if ( ptsPre( ).size( ) >= 8 )
    {
        ROS_DEBUG( "FM ransac begins" );
        TicToc t_f;

        int FOCAL_LENGTH = 460;
        std::vector< cv::Point2f > un_prev_pts( ptsPre( ).size( ) ),
        un_forw_pts( ptsThis( ).size( ) );
        for ( unsigned int i = 0; i < ptsPre( ).size( ); i++ )
        {
            Eigen::Vector3d tmp_p;
            camera( )->liftProjective( Eigen::Vector2d( ptsPre( )[i].x, ptsPre( )[i].y ), tmp_p );
            tmp_p.x( )     = FOCAL_LENGTH * tmp_p.x( ) / tmp_p.z( ) + col( ) / 2.0;
            tmp_p.y( )     = FOCAL_LENGTH * tmp_p.y( ) / tmp_p.z( ) + row( ) / 2.0;
            un_prev_pts[i] = cv::Point2f( tmp_p.x( ), tmp_p.y( ) );

            camera( )->liftProjective( Eigen::Vector2d( ptsThis( )[i].x, ptsThis( )[i].y ), tmp_p );
            tmp_p.x( )     = FOCAL_LENGTH * tmp_p.x( ) / tmp_p.z( ) + col( ) / 2.0;
            tmp_p.y( )     = FOCAL_LENGTH * tmp_p.y( ) / tmp_p.z( ) + row( ) / 2.0;
            un_forw_pts[i] = cv::Point2f( tmp_p.x( ), tmp_p.y( ) );
        }

        std::vector< uchar > status;
        cv::findFundamentalMat( un_forw_pts, un_prev_pts, cv::FM_RANSAC, F_Th( ), 0.99, status );
        int size_a = ptsPre( ).size( );
        reduce_vector( ptsPre( ), status, status_track );
        reduce_vector( ptsThis( ), status, status_track );
        reduce_vector( idsPts( ), status, status_track );
        reduce_vector( cntTrack( ), status, status_track );
        ROS_DEBUG( "FM ransac: %d -> %lu: %f", size_a, ptsPre( ).size( ), 1.0 * ptsPre( ).size( ) / size_a );
        ROS_DEBUG( "FM ransac costs: %fms", t_f.toc( ) );
    }
}

cv::Mat
FeatureMonoTracker::draw( )
{
    if ( !image_this( ).empty( ) //
         && !getPtsPre( ).empty( ) )
    {
        int height = row( );
        int width  = col( );

        cv::Mat output( height, width, CV_8UC3, cv::Scalar( 0, 0, 0 ) );

        cv::cvtColor( image_this( ), output, CV_GRAY2BGR );

        std::vector< cv::Point2f > pts_pre  = getPtsPre( );
        std::vector< cv::Point2f > pts_this = getPtsThis( );
        std::vector< int > track_cnt        = getCntTrack( );

        // previous image to this image
        for ( loop_index_t i = 0; i < loop_index_t( pts_this.size( ) ); i++ )
        {
            cv::Point2f pt_last = pts_pre[i];
            cv::Point2f pt_this = pts_this[i];
            cv::line( output, pt_last, pt_this, cv::Scalar( 255, 0, 0 ), 1, cv::LINE_AA );

            double len = std::min( 1.0, 1.0 * track_cnt[i] / 10 );

            cv::circle( output, pt_this, 1, cv::Scalar( 255 * ( 1 - len ), 0, 255 * len ), 2 );
        }

        return output;
    }
    else
    {
        return cv::Mat( );
    }
}

void
FeatureMonoTracker::readImages( const std::vector< cv::Mat >& image_buf )
{
    readInImage( image_buf[0] );
}

void
FeatureMonoTracker::getPoints( std::vector< std::vector< Eigen::Vector3d > >& points,
                               std::vector< int >& id )
{
    points.push_back( getUndistortedPoints( ) );
    id = getIdsPts( );
}

void
FeatureMonoTracker::getPoints( std::vector< std::vector< Eigen::Vector3d > >& points,
                               std::vector< std::vector< double > >& error_angle,
                               std::vector< int >& id )
{
    points.push_back( getUndistortedPoints( ) );
    error_angle.push_back( getPErrorAnglePoints( ) );
    id = getIdsPts( );
}
