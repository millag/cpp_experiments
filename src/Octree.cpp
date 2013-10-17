#include <limits>
#include <cassert>

#include "Octree.h"


Octree::Octree( const std::vector<Vec3>& pointCloud, unsigned resolution , unsigned maxDepth ):m_resolution(resolution),m_maxDepth(maxDepth)
{
    assert( resolution > 0 && maxDepth > 0 );
    OctreeNode* node = new OctreeNode();
    m_depth = buildNode( pointCloud, 0u, *node );
    m_root = node;
}

unsigned Octree::buildNode( const std::vector<Vec3>& pointCloud, unsigned depth , OctreeNode& o_node)
{
    ++depth;
    // if we hit the bottom
    if ( pointCloud.size() <= m_resolution || depth >= m_maxDepth )
    {
        o_node.m_points.insert( o_node.m_points.begin(), pointCloud.begin(), pointCloud.end() );
        return depth;
    }

    o_node.m_midPoint = getBarycenterForPointCloud( pointCloud );
    o_node.m_children.resize( 8, NULL );

    unsigned d;
    unsigned maxDepth = depth;
    for ( unsigned i = 0; i< o_node.m_children.size(); ++i )
    {
        std::vector<Vec3> subCloud = filterPointsForOctant( pointCloud, o_node.m_midPoint, i );
        if ( !subCloud.size() )
            continue;

        OctreeNode* node = new OctreeNode();
        o_node.m_children[i] = node;
        d = buildNode( subCloud, depth, *node );
        maxDepth = std::max( maxDepth, d );
    }

    return maxDepth;
}


std::vector<Vec3> Octree::filterPointsForOctant( const std::vector<Vec3>& pointCloud, const Vec3& origin, unsigned octantIndex ) const
{
    std::vector<Vec3> result;
    typedef std::vector<Vec3>::const_iterator  Iter;
    for ( Iter it = pointCloud.begin(); it != pointCloud.end(); ++it )
    {
        unsigned index = getOctantForPoint( *it, origin );
        if ( index == octantIndex )
            result.push_back( *it );
    }

    return result;
}


Vec3 Octree::getBarycenterForPointCloud( const std::vector<Vec3>& pointCloud ) const
{
    Vec3 barycenter;
    if ( !pointCloud.size() )
        return barycenter;

    typedef std::vector<Vec3>::const_iterator  Iter;
    for ( Iter it = pointCloud.begin(); it != pointCloud.end(); ++it )
    {
        barycenter += *it;
    }

    barycenter /= pointCloud.size();
    return barycenter;
}

unsigned Octree::getOctantForPoint( const Vec3& point, const Vec3& origin ) const
{
    return ((point.m_x < origin.m_x) << 2) + ((point.m_y < origin.m_y) << 1) + (point.m_z < origin.m_z);
}

float Octree::getNearestPoint( const Vec3& point, Vec3& o_found ) const
{
    return findNearestPoint( point, *m_root, o_found );
}

float Octree::findNearestPoint( const Vec3& point, const OctreeNode& node, Vec3& o_found ) const
{
//    leaf = hit the bottom
    if ( node.isLeaf() )
    {
        return node.getNearestPoint( point, o_found );
    }
    unsigned octIndex = getOctantForPoint( point, node.m_midPoint );
    float minSqrDist = std::numeric_limits<float>::max();
//    find nearest point in search point's octatnt if it has any points in it
    if ( node.m_children[octIndex] != NULL )
    {
        minSqrDist = findNearestPoint( point, *node.m_children[octIndex], o_found );
    }

//    check if we intersect with any of the axis planes
    Vec3 p = (point - node.m_midPoint);
    p.m_x *= p.m_x;
    p.m_y *= p.m_y;
    p.m_z *= p.m_z;
    p -= Vec3( minSqrDist, minSqrDist, minSqrDist );
    unsigned mask = getOctantForPoint( p, Vec3() );

//    no intersection found
    if ( !mask )
    {
        return minSqrDist;
    }
//    we have an intersection => loop through all children and check every octant for intersection
    float dist = minSqrDist;
    Vec3 nearest = o_found;
    for ( unsigned i = 0; i < node.m_children.size(); ++i)
    {
//        no points in octtant i => no need to check it
        if ( node.m_children[i] == NULL )
            continue;

//      check for intersection with current octatnt =>  tricky and unreadable :P
        if ( ((i & mask)^(octIndex & mask)) != 0  && (((i & (~mask))^(octIndex & (~mask))) == 0) )
        {
            dist = findNearestPoint( point, *node.m_children[i], nearest );
            if ( dist < minSqrDist )
            {
                minSqrDist = dist;
                o_found = nearest;
            }
        }
    }

    return minSqrDist;
}




float OctreeNode::getNearestPoint( const Vec3& point, Vec3& o_found ) const
{
    return ::getNearestPoint( point, m_points, o_found );
}
