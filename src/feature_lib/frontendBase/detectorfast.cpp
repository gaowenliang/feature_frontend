#include "feature_frontend/feature_lib/frontendBase/detectorfast.h"
#include "feature_frontend/feature_lib/frontendBase/tic_toc.h"

#define DEBUG 1

frontend::DetectorFast::DetectorFast( )
: fastMaxLoopNum( 5 )
, fastMaxThreshold( 100 )
, fastMinThreshold( 50 )
, fastPerThreshold( ( fastMaxThreshold - fastMinThreshold ) / fastMaxLoopNum )
{
    fast = cv::FastFeatureDetector::create( );
}

frontend::DetectorFast::DetectorFast( int minFeatureDist, int maxlooptime, int maxthreshold, int minthreshold )
: m_minFeatureDist( minFeatureDist )
, fastMaxLoopNum( maxlooptime )
, fastMaxThreshold( maxthreshold )
, fastMinThreshold( minthreshold )
, fastPerThreshold( ( fastMaxThreshold - fastMinThreshold ) / fastMaxLoopNum )
{
    fast = cv::FastFeatureDetector::create( 10, true, cv::FastFeatureDetector::TYPE_9_16 );
}

void
frontend::DetectorFast::initMaskWithSquareNum( const cv::Mat& _mask,
                                               const int numWidth,
                                               const int numHeight,
                                               const int _image_width,
                                               const int _image_height )
{
    int image_width;
    int image_height;
    if ( _mask.empty( ) )
    {
        image_width  = _image_width;
        image_height = _image_height;
        mask         = cv::Mat( image_height, image_width, CV_8UC1, cv::Scalar( 255 ) );
    }
    else
    {
        mask         = _mask;
        image_width  = mask.cols;
        image_height = mask.rows;
    }

    maskNumWidth = numWidth;
    maskNumWidth = numHeight;

    maskPerWidth  = std::ceil( image_width / maskNumWidth );
    maskPerHeight = std::ceil( image_height / maskNumWidth );
    std::cout << "mask num  " << maskPerWidth << " " << maskPerHeight << std::endl;

    for ( int index_height = 0; index_height < numHeight; ++index_height )
        for ( int index_width = 0; index_width < numWidth; ++index_width )
        {
            int width = ( index_width * maskPerWidth + maskPerWidth ) >= image_width ?
                        ( image_width - index_width * maskPerWidth ) :
                        maskPerWidth;
            int height = ( index_height * maskPerHeight + maskPerHeight ) >= image_height ?
                         ( image_height - index_height * maskPerHeight ) :
                         maskPerHeight;

            cv::Rect rect( index_width * maskPerWidth, //
                           index_height * maskPerHeight,
                           width,
                           height );
            std::cout << "mask rect size  " << width << " " << height << std::endl;

            maskRect.push_back( rect );
        }
    std::cout << "mask image size  " << mask.cols << " " << mask.rows << std::endl;

    numMask = maskRect.size( );
}

void
frontend::DetectorFast::initMaskWithSquareSize( const cv::Mat& _mask,
                                                const int perWidth,
                                                const int perHeight,
                                                const int _image_width,
                                                const int _image_height )
{
    int image_width;
    int image_height;
    if ( _mask.empty( ) )
    {
        image_width  = _image_width;
        image_height = _image_height;
        mask         = cv::Mat( image_height, image_width, CV_8UC1, cv::Scalar( 255 ) );
    }
    else
    {
        mask         = _mask;
        image_width  = mask.cols;
        image_height = mask.rows;
    }

    maskPerWidth  = perWidth;
    maskPerHeight = perHeight;

    maskNumWidth = std::ceil( image_width / maskPerWidth );
    maskNumWidth = std::ceil( image_height / maskPerHeight );

    for ( int index_height = 0; index_height < maskNumWidth; ++index_height )
        for ( int index_width = 0; index_width < maskNumWidth; ++index_width )
        {
            int width = ( index_width * maskPerWidth + maskPerWidth ) >= image_width ?
                        ( image_width - index_width * maskPerWidth ) :
                        maskPerWidth;
            int height = ( index_height * maskPerHeight + maskPerHeight ) >= image_height ?
                         ( image_height - index_height * maskPerHeight ) :
                         maskPerHeight;
            cv::Rect rect( index_width * maskPerWidth, //
                           index_height * maskPerHeight,
                           width,
                           height );
            maskRect.push_back( rect );
        }

    std::cout << "mask image size  " << mask.cols << " " << mask.rows << std::endl;

    numMask = maskRect.size( );
}

cv::Mat
frontend::DetectorFast::setMask( const std::vector< cv::Point2f >& existPoints )
{
    cv::Mat masknew;
    mask.copyTo( masknew );

    if ( !existPoints.empty( ) )
    {
        for ( auto& pt : existPoints )
        {
            if ( masknew.at< uchar >( pt ) != uchar( 0 ) )
            {
                cv::circle( masknew, pt, m_minFeatureDist / 2.5, 0, -1 );
            }
        }
    }
    return masknew;
}

cv::Mat
frontend::DetectorFast::setMask( const std::vector< cv::KeyPoint >& points )
{
    if ( points.empty( ) )
        return mask;
    else
    {
        cv::Mat masknew = mask;
        for ( auto& pt : points )
        {
            if ( masknew.at< uchar >( pt.pt ) != 0 )
            {
                cv::circle( masknew, pt.pt, m_minFeatureDist / 2.5, 0, -1 );
            }
        }
        return masknew;
    }
}

void
frontend::DetectorFast::setMask( cv::Mat& mask, const std::vector< cv::Point2f >& points )
{
    if ( !points.empty( ) )
    {
        for ( auto& pt : points )
        {
            if ( mask.at< uchar >( pt ) != 0 )
            {
                cv::circle( mask, pt, m_minFeatureDist / 2.5, 0, -1 );
            }
        }
    }
}

void
frontend::DetectorFast::setMask( cv::Mat& mask, const std::vector< cv::KeyPoint >& points )
{
    if ( !points.empty( ) )
    {
        for ( auto& pt : points )
        {
            if ( mask.at< uchar >( pt.pt ) != 0 )
            {
                cv::circle( mask, pt.pt, m_minFeatureDist / 2.5, 0, -1 );
            }
        }
    }
}

void
frontend::DetectorFast::setMask( cv::Mat& mask,
                                 const std::vector< cv::KeyPoint >& points, //
                                 const cv::Rect& mask_rect )
{
    if ( !points.empty( ) )
    {
        for ( auto& pt : points )
        {
            cv::Point2f pt_src = pt.pt + cv::Point2f( mask_rect.x, mask_rect.y );
            if ( mask.at< uchar >( pt_src ) != 0 )
            {
                cv::circle( mask, pt_src, m_minFeatureDist / 2.5, 0, -1 );
            }
        }
    }
}

void
frontend::DetectorFast::setMask( cv::Mat& mask,
                                 const std::vector< cv::KeyPoint >& points,
                                 const std::vector< int >& rect_index,
                                 const std::vector< cv::Rect >& mask_rect )
{
    if ( !points.empty( ) )
    {
        for ( int index = 0; index < int( points.size( ) ); ++index )
        {
            cv::Point2f pt_src = points[index].pt + cv::Point2f( mask_rect[rect_index[index]].x,
                                                                 mask_rect[rect_index[index]].y );
            if ( mask.at< uchar >( pt_src ) != 0 )
            {
                cv::circle( mask, pt_src, m_minFeatureDist / 2.5, 0, -1 );
            }
        }
    }
}

int
frontend::DetectorFast::pointInMask( const cv::Point2f& pt )
{
    int index_width  = std::floor( pt.x / maskPerWidth );
    int index_height = std::floor( pt.y / maskPerHeight );
    return index_width * maskNumWidth + index_height;
}

void
frontend::DetectorFast::detectNewFeatures( const cv::Mat& image_in,
                                           const std::vector< cv::Point2f >& existPoints,
                                           const int _max_new_num,
                                           std::vector< cv::Point2f >& newPoints )
{
    cv::Mat new_masks;

    if ( existPoints.empty( ) )
        mask.copyTo( new_masks );
    else
        new_masks = setMask( existPoints );

    int new_num = _max_new_num / numMask;
    if ( new_num < 2 )
        new_num = 2;
    //    std::cout << "new_num: " << new_num << std::endl; //

    for ( auto& rect : maskRect )
    {
        int size_get_new = 0;
        for ( int index = 0; index < fastMaxLoopNum; ++index )
        {
            int size_need_new = new_num - size_get_new;

            std::vector< cv::KeyPoint > keyPointsTmp;

            fast->setThreshold( calcThresholdLinear( index ) );

            fast->detect( image_in( rect ), keyPointsTmp, new_masks( rect ) );

            size_get_new += keyPointsTmp.size( );

            if ( int( keyPointsTmp.size( ) ) > size_need_new )
                insertionSort( keyPointsTmp );

            if ( int( keyPointsTmp.size( ) ) >= size_need_new )
            {
                for ( int pt_index = 1; pt_index < ( size_need_new - 1 ); ++pt_index )
                    newPoints.push_back(
                    cv::Point2f( keyPointsTmp[size_need_new - pt_index].pt.x + rect.x,
                                 keyPointsTmp[size_need_new - pt_index].pt.y + rect.y ) );

                break;
            }
            else
            {
                setMask( new_masks, keyPointsTmp, rect );
            }

            // ROS_DEBUG_STREAM( "loop_time: " << index //
            //                                 << " setThreshold: "
            //                                 << calcThresholdLinear( index )
            //                                 << " find keyPoints "
            //                                 << keyPointsTmp.size( ) );
        }
    }
}

void
frontend::DetectorFast::insertionSort( std::vector< cv::KeyPoint >& arr )
{
    int num = arr.size( );
    cv::KeyPoint temp;
    int i, j;
    for ( i = 1; i < num; i++ )
    {
        temp = arr[i];
        for ( j = i; j > 0 && arr[j - 1].response > temp.response; j-- )
        {
            arr[j] = arr[j - 1];
        }
        arr[j] = temp;
    }
}
