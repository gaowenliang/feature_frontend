#ifndef PARAMETER_H
#define PARAMETER_H

#include "iostream"
#include <opencv2/opencv.hpp>

namespace frontend
{

enum FrontendType
{
    MONO = 0,
    STEREO
};

class FrontendParameter
{
    public:
    FrontendParameter( )
    : m_freq( 0 )
    , m_min_dist( 0 )
    , m_max_cnt( 0 )
    , m_is_mask( false )
    , m_is_RANSAC( true )
    , m_is_EQUALIZE( false )
    , m_is_SHOW_TRACK( 1 )
    {
    }

    FrontendParameter( std::string cam_name, //
                       int min_dist,
                       int max_cnt,
                       int freq,
                       bool is_RANSAC,
                       int is_mask,
                       int is_EQUALIZE,
                       int is_SHOW_TRACK )
    : m_freq( freq )
    , m_min_dist( min_dist )
    , m_max_cnt( max_cnt )
    , m_is_mask( is_mask )
    , m_is_RANSAC( true )
    , m_is_EQUALIZE( is_EQUALIZE )
    , m_is_SHOW_TRACK( is_SHOW_TRACK )
    , m_cam_name( cam_name )
    {
    }

    bool readFromYamlFile( const std::string& filename )
    {
        cv::FileStorage fs( filename, cv::FileStorage::READ );

        if ( !fs.isOpened( ) )
        {
            std::cerr << "ERROR: Wrong path to settings 'feature_config_file': " << filename << std::endl;
            return false;
        }

        m_max_cnt       = fs["max_cnt"];
        m_min_dist      = fs["min_dist"];
        m_freq          = fs["freq"];
        m_F_THRESHOLD   = fs["F_threshold"];
        m_is_SHOW_TRACK = fs["show_track"];
        m_is_EQUALIZE   = fs["equalize"];
        m_is_mask       = fs["is_mask"];

        if ( m_is_mask )
        {
            std::string mask_path;
            fs["mask_path"] >> mask_path;
            m_mask = cv::imread( mask_path, cv::IMREAD_GRAYSCALE );
        }

        fs.release( );

        if ( m_is_SHOW_TRACK )
        {
            std::cout << "max_cnt     " << m_max_cnt << std::endl;
            std::cout << "min_dist    " << m_min_dist << std::endl;
            std::cout << "freq        " << m_freq << std::endl;
            std::cout << "F_threshold " << m_F_THRESHOLD << std::endl;
        }

        return true;
    }

    int m_freq;
    int m_min_dist;
    int m_max_cnt;
    double m_F_THRESHOLD;
    int m_is_mask;
    cv::Mat m_mask;
    bool m_is_RANSAC;
    int m_is_EQUALIZE;
    int m_is_SHOW_TRACK;
    std::string m_cam_name;
    FrontendType m_type;
};
}
#endif // PARAMETER_H
