#ifndef DETECTOR_H
#define DETECTOR_H

#include <opencv2/opencv.hpp>

namespace frontend
{

class Detector
{
    public:
    Detector( )
    : m_minFeatureDist( 0 )
    {
    }
    Detector( int minFeatureDist )
    : m_minFeatureDist( minFeatureDist )
    {
    }

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

    virtual void detectNewFeatures( const cv::Mat& image_in,
                                    const std::vector< cv::Point2f >& existPoints, //
                                    const int _max_new_num,
                                    std::vector< cv::Point2f >& newPoints )
    = 0;

    void setMinFeatureDist( int minFeatureDist ) { m_minFeatureDist = minFeatureDist; }

    public:
    int maskPerWidth;
    int maskPerHeight;
    int maskNumWidth; // row majar
    int maskNumHeight;
    std::vector< cv::Rect > maskRect;
    int numMask;
    cv::Mat mask;
    int m_minFeatureDist;
};
}
#endif // DETECTOR_H
