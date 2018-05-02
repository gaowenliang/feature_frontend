#ifndef DETECTORFAST_H
#define DETECTORFAST_H

#include <opencv2/opencv.hpp>

namespace frontend
{
class DetectorFast
{
    public:
    DetectorFast( );
    DetectorFast( int minFeatureDist,
                  int maxlooptime  = 5,
                  int maxthreshold = 70, //
                  int minthreshold = 30 );
    ~DetectorFast( ) { maskRect.clear( ); }

    void initMaskWithSquareNum( const cv::Mat& _mask,
                                const int numWidth,
                                const int numHeight, //
                                const int _image_width  = 0,
                                const int _image_height = 0 );

    void initMaskWithSquareSize( const cv::Mat& _mask, //
                                 const int perWidth,
                                 const int perHeight,
                                 const int _image_width  = 0,
                                 const int _image_height = 0 );

    cv::Mat setMask( const std::vector< cv::Point2f >& existPoints );

    cv::Mat setMask( const std::vector< cv::KeyPoint >& points );
    void setMask( cv::Mat& mask, const std::vector< cv::Point2f >& points );
    void setMask( cv::Mat& mask, const std::vector< cv::KeyPoint >& points );
    void setMask( cv::Mat& mask, const std::vector< cv::KeyPoint >& points, const cv::Rect& mask_rect );
    void setMask( cv::Mat& mask,
                  const std::vector< cv::KeyPoint >& points,
                  const std::vector< int >& rect_index,
                  const std::vector< cv::Rect >& mask_rect );
    int pointInMask( const cv::Point2f& pt );

    void detectNewFeatures( const cv::Mat& image_in,
                            const std::vector< cv::Point2f >& existPoints, //
                            const int _max_new_num,
                            std::vector< cv::Point2f >& newPoints );
    void setMinFeatureDist( int minFeatureDist ) { m_minFeatureDist = minFeatureDist; }

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
    int maskPerWidth;
    int maskPerHeight;
    int maskNumWidth; // row majar
    int maskNumHeight;
    cv::Mat mask;
    std::vector< cv::Rect > maskRect;
    int numMask;

    int m_minFeatureDist;
    cv::Ptr< cv::FastFeatureDetector > fast;
    int fastMaxLoopNum;
    int fastMaxThreshold;
    int fastMinThreshold;
    int fastPerThreshold;
};
}
#endif // DETECTORFAST_H
