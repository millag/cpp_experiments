#include <iostream>
#include <vector>
#include <cstdlib>
#include <limits>

#include "workshop1.h"
#include "Vec3.h"
#include "HelperFunctions.h"
#include "Octree.h"

int main()
{
    unsigned pointCnt = 1000000;//1000:1
    std::cout << "Point count: " << pointCnt << std::endl;

    std::vector<Vec3> pointCloud;
    genRandPointCloud( pointCnt, -1.0f, 1.0f, pointCloud );
//    for ( unsigned i = 0; i < pointCloud.size(); ++i ) {
//        pointCloud[i].print();
//    }

    Vec3 searchPoint = genRandPoint( -1.0f, 1.0f );

    Vec3 nearest;
    float minSqrDist = getNearestPoint( searchPoint, pointCloud, nearest );
    nearest.print();
    std::cout << assertClosest( searchPoint , nearest, pointCloud ) << std::endl;
    std::cout << minSqrDist << std::endl;

    Octree octree( pointCloud, 10 );
    std::cout << octree.getDepth() << std::endl;
    Vec3 nearest2;
    float minSqr2Dist = octree.getNearestPoint( searchPoint, nearest2 );
    std::cout <<  minSqr2Dist << " == " << minSqrDist << std::endl;
    nearest.print();
    nearest2.print();

    return EXIT_SUCCESS;
}

