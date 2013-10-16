#include <limits>
#include <tr1/random>
#include <cmath>

#include "HelperFunctions.h"

template<class T>
T clamp(T v, T a, T b)
{
    return std::max(a, std::min(v, b));
}

float randf( float min, float max) {
    return min + (max - min) * ( (float)rand() / RAND_MAX );
}

Vec3 genRandPoint( float bBoxMin, float bBoxMax )
{
    return Vec3( randf( bBoxMin, bBoxMax ), randf( bBoxMin, bBoxMax ), randf( bBoxMin, bBoxMax ) );
}

void genRandPointCloud( unsigned numPoints, float bBoxMin , float bBoxMax, std::vector<Vec3>& o_pointCloud)
{
    o_pointCloud.resize( numPoints );
    for ( std::vector<Vec3>::iterator  it = o_pointCloud.begin(); it != o_pointCloud.end(); ++it  )
    {
        *it = genRandPoint( bBoxMin, bBoxMax );
    }
}

float getNearestPoint( const Vec3& point, const std::vector<Vec3>& pointCloud, Vec3& o_found )
{

    float minSqrDist = std::numeric_limits<float>::max();
    for ( std::vector<Vec3>::const_iterator it = pointCloud.begin(); it != pointCloud.end(); ++it )
    {
        //skip if we've hitted the search point
        if ( &point == &(*it) )
        {
            continue;
        }

        float dist = Vec3::sqrDistance( point, *it );
        if ( dist < minSqrDist )
        {
            minSqrDist = dist;
            o_found = *it;
        }
    }

    return minSqrDist;
}

bool assertClosest( const Vec3& point, const Vec3& found, const std::vector<Vec3>& pointCloud )
{
    float minDist = Vec3::sqrDistance( point, found);
    bool res = true;

    for ( std::vector<Vec3>::const_iterator it = pointCloud.begin(); it != pointCloud.end(); ++it )
    {
        float dist = Vec3::sqrDistance( point, *it );
        if ( dist < minDist )
        {
            res = false;
            break;
        }
    }
    return res;
}
