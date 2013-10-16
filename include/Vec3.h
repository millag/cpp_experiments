#ifndef VEC3_H
#define VEC3_H

#include<string>
#include<cmath>
#include<iostream>

class Vec3
{

public:
    float m_x, m_y, m_z;


    Vec3( float x = 0, float y = 0, float z = 0):m_x(x),m_y(y),m_z(z) { }

    Vec3( const Vec3& v):m_x(v.m_x),m_y(v.m_y),m_z(v.m_z) { }

    Vec3& operator+=( const Vec3& rhs )
    {
        m_x += rhs.m_x;
        m_y += rhs.m_y;
        m_z += rhs.m_z;
        return *this;
    }

    Vec3& operator-=( const Vec3& rhs )
    {
        m_x -= rhs.m_x;
        m_y -= rhs.m_y;
        m_z -= rhs.m_z;
        return *this;
    }

    Vec3& operator*=( float d )
    {
        m_x *= d;
        m_y *= d;
        m_z *= d;
        return *this;
    }

    Vec3& operator/=( float d )
    {
        m_x /= d;
        m_y /= d;
        m_z /= d;
        return *this;
    }


    inline void print() const
    {
        std::cout << "( " << m_x << ", " << m_y << ", " << m_z << " )" << std::endl;
    }

    static float sqrDistance(const Vec3& p1, const Vec3& p2)
    {
        return pow( p1.m_x - p2.m_x, 2.0f) + pow( p1.m_y - p2.m_y, 2.0f) + pow( p1.m_z - p2.m_z, 2.0f);
    }
};

inline Vec3 operator+( const Vec3& lhs, const Vec3& rhs)
{
    return Vec3( lhs.m_x + rhs.m_x , lhs.m_y + rhs.m_y , lhs.m_z + rhs.m_z );
}

inline Vec3 operator-( const Vec3& lhs, const Vec3& rhs)
{
    return Vec3( lhs.m_x - rhs.m_x , lhs.m_y - rhs.m_y , lhs.m_z - rhs.m_z );
}

inline Vec3 operator*( const Vec3& lhs, float d )
{
    return Vec3( lhs.m_x * d , lhs.m_y * d , lhs.m_z * d );
}

inline Vec3 operator/( const Vec3& lhs, float d )
{
    return Vec3( lhs.m_x / d , lhs.m_y / d , lhs.m_z / d );
}

inline bool operator==( const Vec3& lhs, const Vec3& rhs)
{
    return ( lhs.m_x == rhs.m_x ) && ( lhs.m_y == rhs.m_y ) && ( lhs.m_z == rhs.m_z );
}

inline bool operator!=(const Vec3& lhs, const Vec3& rhs)
{
    return !operator==(lhs,rhs);
}


#endif // VEC3_H
