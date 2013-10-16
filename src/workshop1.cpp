#include <utility>
#include <cstring>
#include <vector>

#include "workshop1.h"

int reverse( char* str )
{
    char* s = str;
    char* e = str + strlen( str ) - 1;
    while ( s < e ) {
        char tmp = *e;
        *e = *s;
        *s = tmp;
        s++; e--;
    }

    return strlen( str );
}

void printSpiral(int n)
{
//    vector<vector<int>> spiral;
//    int s = 0;
//    int e = n;

//    int j = s;
//    for ( int i = s; i < e; i++) {

//    }
//    for ( k = 0; k < n * n; k++ ) {
//        for ( int i = s )
//        int r = k / n;
//        int c = k % n;
//    }
}
