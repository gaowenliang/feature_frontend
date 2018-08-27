#ifndef DETECTORHARRIS_H
#define DETECTORHARRIS_H

#include "detector.h"
#include <opencv2/opencv.hpp>

namespace frontend
{

class DetectorHarris : public Detector
{
    public:
    DetectorHarris( ) {}
    DetectorHarris( int minFeatureDist )
    : Detector( minFeatureDist )
    {
    }
    ~DetectorHarris( ) { maskRect.clear( ); }

    void detectNewFeatures( const cv::Mat& image_in,
                            const std::vector< cv::Point2f >& existPoints, //
                            const int _max_new_num,
                            std::vector< cv::Point2f >& newPoints );
};
}
#endif // DETECTORHARRIS_H
