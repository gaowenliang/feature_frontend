#include <feature_frontend/feature_lib/frontendBase/detectorharris.h>

#define goodFeatures 1

void
frontend::DetectorHarris::detectNewFeatures( const cv::Mat& image_in,
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
        {

#if goodFeatures

            std::vector< cv::Point2f > keyPointsTmp;

            cv::goodFeaturesToTrack( image_in( rect ), //
                                     keyPointsTmp,
                                     new_num,
                                     0.1,
                                     m_minFeatureDist,
                                     cv::Mat( ) /*new_masks( rect ) */ );

            // std::cout << " find keyPoints " << new_num << " -> " << keyPointsTmp.size( )
            // << "\n";

            for ( int pt_index = 0; pt_index < keyPointsTmp.size( ); ++pt_index )
            {
                newPoints.push_back( cv::Point2f( keyPointsTmp[pt_index].x + rect.x,
                                                  keyPointsTmp[pt_index].y + rect.y ) );
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
