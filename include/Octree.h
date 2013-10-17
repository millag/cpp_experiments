#ifndef OCTREE_H
#define OCTREE_H

#include <cstdio>
#include <vector>
#include "Vec3.h"
#include "HelperFunctions.h"


//TODO:
//display tree using openGL
//FIX: pls figure out how to hide OctreeNode class? :(


class OctreeNode
{

public:
    Vec3 m_midPoint;
//    list of child nodes of size 0-when leaf, 8-when internal
//    index of the child calculated by: ( x < midPoint.x )<<2 + ( y < midPoint.y )<<1  + ( z < midPoint.z )<<0
    std::vector<OctreeNode*> m_children;
//    empty when node is internal
    std::vector<Vec3> m_points;



    OctreeNode():m_midPoint(),m_children(),m_points() { }

    bool isLeaf() const { return (m_children.size() == 0); }

    OctreeNode* getChildAt( unsigned index )
    {
        index = clamp( index, 0u, (unsigned)m_children.size() );
        return ( ( index == m_children.size() )? NULL : m_children[index] );
    }

    float getNearestPoint( const Vec3& point, Vec3& o_found ) const;
};

class Octree
{

public:
    Octree():m_resolution(0),m_maxDepth(0),m_root(NULL),m_depth(0) { }
    Octree( const std::vector<Vec3>& pointCloud, unsigned resolution = 10, unsigned maxDepth = 5000 );

    ~Octree()
    {
        if ( m_root != NULL )
        {
            delete m_root;
            m_root = NULL;
        }
    }

    float getNearestPoint( const Vec3& point, Vec3& o_found ) const;

    unsigned getDepth() const { return m_depth; }

private:
    const unsigned m_resolution;
    const unsigned m_maxDepth;
    OctreeNode* m_root;
    unsigned m_depth;

    unsigned buildNode( const std::vector<Vec3>& pointCloud, unsigned depth, OctreeNode& o_node);

    float findNearestPoint( const Vec3& point, const OctreeNode& node, Vec3& o_found ) const;

    std::vector<Vec3> filterPointsForOctant( const std::vector<Vec3>& pointCloud, const Vec3& origin, unsigned octantIndex ) const;

    Vec3 getBarycenterForPointCloud( const std::vector<Vec3>& pointCloud ) const;

    unsigned getOctantForPoint( const Vec3& point, const Vec3& origin ) const;
};

#endif // OCTREE_H
