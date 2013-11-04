#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <vector>
#include "Vec3.h"

template<typename T>
T clamp(T v, T a, T b);

float randf( float min, float max);

Vec3 genRandPoint( float bBoxMin, float bBoxMax );

void genRandPointCloud( unsigned numPoints, float bBoxMin , float bBoxMax, std::vector<Vec3>& o_pointCloud);

//brute force solution of nearest point problem
float getNearestPoint( const Vec3& point, const std::vector<Vec3>& pointCloud, Vec3& o_found );

bool assertClosest( const Vec3& point, const Vec3& found, const std::vector<Vec3>& pointCloud );

#endif // HELPERFUNCTIONS_H
