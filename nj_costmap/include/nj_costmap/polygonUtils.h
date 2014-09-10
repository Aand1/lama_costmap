#ifndef _NJ_COSTMAP_POLYGON_UTILS_H_
#define _NJ_COSTMAP_POLYGON_UTILS_H_

#include <vector>
#include <list>
#include <algorithm>
#include <cmath>

#include "Voronoi.h"

namespace lama{
namespace PolygonUtils {

using std::list;
using std::sqrt;
using Voronoi::Point;

template<typename T>
class TPoint {
	public:
		TPoint(const T xx=0, const T yy=0):x(xx),y(yy) {}
		TPoint(const TPoint<T> &p):x(p.x),y(p.y) {}

		T x,y;
};


template<typename T>
int wind(const TPoint<T> &a, const TPoint<T> &b, const TPoint<T> &c)
{
   T w;
   w = ((a.y - b.y)*(c.x - b.x) - (c.y - b.y)*(a.x - b.x));
   return (w > .0001) ? 1 : ((w < -.0001) ? -1 : 0);
}


template<typename T>
int in_cone (const TPoint<T> &a0,const TPoint<T> &a1,const TPoint<T> &a2,const TPoint<T> &b)
{
   int m = wind(b,a0,a1);
   int p = wind(b,a1,a2);
   if (wind(a0,a1,a2) > 0)
      return ( m >= 0 && p >= 0 ); /* convex at a */
   else
      return ( m >= 0 || p >= 0 ); /* reflex at a */
}


template<typename T>
int inBetween (const TPoint<T> &a,const TPoint<T> &b,const TPoint<T> &c)
{
   if (a.x != b.x)   /* not vertical */
      return (((a.x < c.x) && (c.x < b.x)) || ((b.x < c.x) && (c.x < a.x)));
   else
      return (((a.y < c.y) && (c.y < b.y)) || ((b.y < c.y) && (c.y < a.y)));
}

/* the code is adopted from J.Faigl's cis.cpp from imrh::pathPlanning */
template<typename T>
int intersect(const TPoint<T> &a,const TPoint<T> &b,const TPoint<T> &c,const TPoint<T> &d)
{
   int a_abc;
   int a_abd;
   int a_cda;
   int a_cdb;
   a_abc = wind(a,b,c);
   if ((a_abc == 0) && inBetween (a,b,c)) {
      return 1;
   }
   a_abd = wind(a,b,d);
   if ((a_abd == 0) && inBetween (a,b,d)) {
      return 1;
   }
   a_cda = wind(c,d,a);
   a_cdb = wind(c,d,b);
   // True if c and d are on opposite sides of ab,
   // and a and b are on opposite sides of cd.
   //
   return (((a_abc * a_abd) < 0) && ((a_cda * a_cdb) < 0));
}



// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2 on the line
//            <0 for P2 right of the line
//    See: the January 2001 Algorithm "Area of 2D and 3D Triangles and Polygons"
template<typename T>
T isLeft(const  TPoint<T> &P0, const TPoint<T> &P1, const TPoint<T> &P2 )
{
    return ( (P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y) );
}


//===================================================================

// cn_PnPoly(): crossing number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  0 = outside, 1 = inside
// This code is patterned after [Franklin, 2000]

template<typename T>
int cn_PnPoly(const TPoint<T> &P, const std::vector< TPoint<T> > &V)
{
    int    cn = 0;    // the crossing number counter
	const int n = V.size() -1 ;

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {    // edge from V[i] to V[i+1]
       if (((V[i].y <= P.y) && (V[i+1].y > P.y))    // an upward crossing
        || ((V[i].y > P.y) && (V[i+1].y <= P.y))) { // a downward crossing
            // compute the actual edge-ray intersect x-coordinate
            T vt = (T)1.0*(P.y - V[i].y) / (V[i+1].y - V[i].y);
            if (P.x < V[i].x + vt * (V[i+1].x - V[i].x)) // P.x < intersect
                ++cn;   // a valid crossing of y=P.y right of P.x
        }
    }
    return (cn&1);    // 0 if even (out), and 1 if odd (in)

}



// wn_PnPoly(): winding number test for a point in a polygon
//      Input:   P = a point,
//               V[] = vertex points of a polygon V[n+1] with V[n]=V[0]
//      Return:  wn = the winding number (=0 only if P is outside V[])
template<typename T>
int wn_PnPoly( const TPoint<T> &P, const std::vector< TPoint<T> > &V)
{
    int    wn = 0;    // the winding number counter
	const int n = V.size() -1;

    // loop through all edges of the polygon
    for (int i=0; i<n; i++) {   // edge from V[i] to V[i+1]
        if (V[i].y <= P.y) {         // start y <= P.y
            if (V[i+1].y > P.y)      // an upward crossing
                if (isLeft( V[i], V[i+1], P) > 0)  // P left of edge
                    ++wn;            // have a valid up intersect
        }
        else {                       // start y > P.y (no test needed)
            if (V[i+1].y <= P.y)     // a downward crossing
                if (isLeft( V[i], V[i+1], P) < 0)  // P right of edge
                    --wn;            // have a valid down intersect
        }
    }
    return wn;

}



// return true if robot( polygon a) is not in collision with 
// map (polygon b)
// false     all vertices of a are in b and no intersection
//           between segment of a vs b.
// true      othervise
template<typename T>
bool robotMapCollision(const std::vector< TPoint<T> > &a, const std::vector< TPoint<T> > &b) {

	for(int i=0;i<a.size();i++) {
		if (wn_PnPoly(a[i],b) == 0)
			return true;
	}

	for(int i=0 ; i<a.size()-1 ; i++) {

		for(int j=0 ; j<b.size()-1 ; j++) {

			if (intersect(a[i],a[i+1],b[j],b[j+1])) {
				return true;
			}

		}

	}

	return false;

}

template<typename T>
struct S {
		int idx;
		T r;
		S(const int i, const T rel):idx(i),r(rel) {}
};

template<typename T>
class myLess {
	public:
	bool operator()(const T &a, const T &b) {
		return a.r < b.r;
	}
};

double getRelevance(const Point &pi, const Point &pj, const Point &pk);

double getRelevance(const list< S<double> > &s, 
		typename list< S<double> >::const_iterator i, 
		const std::vector<Point> &pts);

std::vector<Point> filterRelevance(const std::vector<Point> &pts, const double maxR);

template<typename T>
std::vector< TPoint<T> >cutPolygon(const std::vector< TPoint<T> > &pts, const T maxDistance, const TPoint<T> &center = TPoint<T>(0,0)) {
	std::vector< TPoint<T> > result;
	T dx,dy;
	const T d2 = maxDistance * maxDistance;

	for(int i=0;i<pts.size();i++) {
		dx = pts[i].x - center.x;
		dy = pts[i].y - center.y;
		if ( (dx * dx + dy * dy) < d2) {
			result.push_back(pts[i]);
		}
	}

	return result;
}

} // namespace PolygonUtils
} // namesapce lama

#endif // _NJ_COSTMAP_POLYGON_UTILS_H_

