#ifndef FEATURE_UTILS_H
#define FEATURE_UTILS_H

#include <vector>

typedef int index_t;
typedef unsigned int loop_index_t;

template< typename T, typename T_status >
inline void
reduce_vector( std::vector< T >& v, std::vector< T_status > status )
{
    loop_index_t j = 0;
    if ( !status.empty( ) )
    {
        for ( loop_index_t i = 0; i < loop_index_t( v.size( ) ); i++ )
            if ( status[i] )
                v[j++] = v[i];
        v.resize( j );
    }
}

template< typename T, typename T_status >
inline void
reduce_vector( std::vector< T >& v, std::vector< T_status > status, std::vector< T_status > status2 )
{
    loop_index_t j = 0;
    for ( loop_index_t i = 0; i < loop_index_t( v.size( ) ); i++ )
        if ( status[i] && status2[i] )
            v[j++] = v[i];
    v.resize( j );
}

template< typename T, typename T_status >
inline void
reduce_vector( std::vector< T >& v,
               std::vector< T_status > status,
               std::vector< T_status > status2,
               std::vector< T_status > status3 )
{
    loop_index_t j = 0;
    for ( loop_index_t i = 0; i < loop_index_t( v.size( ) ); i++ )
        if ( status[i] && status2[i] && status3[i] )
            v[j++] = v[i];
    v.resize( j );
}

#endif // FEATURE_UTILS_H
