#include "feature_frontend/feature_lib/frontendinit.h"

boost::shared_ptr< frontend::FrontendInit > frontend::FrontendInit::m_instance;

frontend::FrontendInit::FrontendInit( ) {}

boost::shared_ptr< frontend::FrontendInit >
frontend::FrontendInit::newFrontend( )
{
    if ( m_instance.get( ) == 0 )
    {
        m_instance.reset( new FrontendInit );
    }

    return m_instance;
}

frontend::FrontendPtr
frontend::FrontendInit::init( frontend::FrontendType type,
                              std::vector< std::string > cam_files,
                              std::vector< std::string > feature_files )
{
    switch ( type )
    {
        case MONO:
        {
            FeatureMonoPtr initial( new FeatureMonoTracker( cam_files[0], feature_files[0] ) );
            return initial;
        }
        case STEREO:
        {
            FeatureStereoPtr initial( new FeatureStereoTracker( cam_files[0],
                                                                cam_files[1], //
                                                                feature_files[0],
                                                                feature_files[1] ) );
            return initial;
        }
        default:
        {
            FeatureMonoPtr initial( new FeatureMonoTracker( cam_files[0], feature_files[0] ) );
            return initial;
        }
    }
    return FrontendPtr( );
}
