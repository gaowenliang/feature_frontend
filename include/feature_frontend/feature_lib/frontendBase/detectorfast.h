#ifndef DETECTORFAST_H
#define DETECTORFAST_H

#include "detector.h"
#include <opencv2/opencv.hpp>

namespace frontend
{
class DetectorFast : public Detector
{
    public:
    DetectorFast( );
    DetectorFast( int minFeatureDist,
                  int maxlooptime  = 5,
                  int maxthreshold = 70, //
                  int minthreshold = 30 );
    ~DetectorFast( ) { maskRect.clear( ); }

    void detectNewFeatures( const cv::Mat& image_in,
                            const std::vector< cv::Point2f >& existPoints, //
                            const int _max_new_num,
                            std::vector< cv::Point2f >& newPoints );

    private:
    void insertionSort( std::vector< cv::KeyPoint >& arr );
    int calcThresholdLinear( int index )
    {
        return fastMaxThreshold - fastPerThreshold * index; // linear
    }
    int calcThresholdLUT( int index )
    {
        // FIXME
        return fastMaxThreshold - fastPerThreshold * index; // linear
    }

    private:
    cv::Ptr< cv::FastFeatureDetector > fast;
    int fastMaxLoopNum;
    int fastMaxThreshold;
    int fastMinThreshold;
    int fastPerThreshold;
};
}
#endif // DETECTORFAST_H
