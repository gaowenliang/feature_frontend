#ifndef FRONTENDINIT_H
#define FRONTENDINIT_H

#include "frontendBase/featuretracker.h"
#include "frontendMono/frontendmono.h"
#include "frontendStereo/frontendstereo.h"

namespace frontend
{

class FrontendInit
{
    public:
    FrontendInit( );
    static boost::shared_ptr< FrontendInit > newFrontend( void );
    FrontendPtr init( FrontendType type, //
                      std::vector< std::string > cam_files,
                      std::vector< std::string > feature_files );
    FrontendPtr init( FrontendType type, //
                      std::vector< std::string > cam_files,
                      std::vector< std::string > error_files,
                      std::vector< std::string > feature_files );

    private:
    static boost::shared_ptr< FrontendInit > m_instance;
};
}
#endif // FRONTENDINIT_H
