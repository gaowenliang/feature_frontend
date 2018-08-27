#include "feature_frontend/feature_lib/frontendBase/detectorfast.h"
#include "feature_frontend/feature_lib/frontendBase/tic_toc.h"

#define DEBUG 1
#define FAST 1

frontend::DetectorFast::DetectorFast( )
: fastMaxLoopNum( 5 )
, fastMaxThreshold( 100 )
, fastMinThreshold( 50 )
, fastPerThreshold( ( fastMaxThreshold - fastMinThreshold ) / fastMaxLoopNum )
{
    fast = cv::FastFeatureDetector::create( );
}

frontend::DetectorFast::DetectorFast( int minFeatureDist, int maxlooptime, int maxthreshold, int minthreshold )
: Detector( minFeatureDist )
, fastMaxLoopNum( maxlooptime )
, fastMaxThreshold( maxthreshold )
, fastMinThreshold( minthreshold )
, fastPerThreshold( ( fastMaxThreshold - fastMinThreshold ) / fastMaxLoopNum )
{
    fast = cv::FastFeatureDetector::create( 10, true, cv::FastFeatureDetector::TYPE_9_16 );
}

void
frontend::DetectorFast::detectNewFeatures( const cv::Mat& image_in,
                                           const std::vector< cv::Point2f >& existPoints,
                                           const int _max_new_num,
                                           std::vector< cv::Point2f >& newPoints )
{
    cv::Mat new_masks;
    newPoints.clear( );

    if ( _max_new_num < 3 )
        return;

    if ( existPoints.empty( ) )
        mask.copyTo( new_masks );
    else
        new_masks = setMask( existPoints );

    int new_num = _max_new_num / numMask;
    if ( new_num < 2 )
        new_num = 2;

    for ( auto& rect : maskRect )
    {
        int size_get_new = 0;
        int index_loop   = 0;
        //        for ( int index_loop = 0; index_loop < fastMaxLoopNum; ++index_loop )
        //        //FIXME
        {
            //            int size_need_new = new_num - size_get_new;
            int sizeNeedNewPerGrid = new_num;

#if FAST
            std::vector< cv::KeyPoint > keyPointsTmp;

            fast->setThreshold( calcThresholdLinear( index_loop ) );

            fast->detect( image_in( rect ), keyPointsTmp, new_masks( rect ) );

            size_get_new += keyPointsTmp.size( );

            if ( int( keyPointsTmp.size( ) ) > sizeNeedNewPerGrid )
            {

                insertionSort( keyPointsTmp );

                for ( int pt_index = keyPointsTmp.size( ) - 1;
                      pt_index >= ( keyPointsTmp.size( ) - sizeNeedNewPerGrid );
                      --pt_index )
                {
                    newPoints.push_back( cv::Point2f( keyPointsTmp[pt_index].pt.x + rect.x,
                                                      keyPointsTmp[pt_index].pt.y + rect.y ) );
                }
            }
            else
            {

                for ( int pt_index = 0; pt_index < keyPointsTmp.size( ); ++pt_index )
                {
                    newPoints.push_back( cv::Point2f( keyPointsTmp[pt_index].pt.x + rect.x,
                                                      keyPointsTmp[pt_index].pt.y + rect.y ) );
                }

                //                setMask( new_masks, keyPointsTmp, rect );
            }

#endif

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
    // low response to high response

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
