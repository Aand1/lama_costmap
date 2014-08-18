#ifndef _LMAP_LOC_TYPES_H_
#define _LMAP_LOC_TYPES_H_

/* A 2D point
 */
struct SPoint {

	double x;
	double y;
	SPoint():x(0),y(0) {}
	SPoint(const double xx, const double yy):x(xx),y(yy) {}
	SPoint(const SPoint &p):x(p.x),y(p.y){}
	bool operator==(const SPoint &p) {
	    return (p.x == x && p.y == y);
	}

};

/* SFrontier is a line segment through which the robot can go
 * p1 First point
 * p2 Second point, so that angle(r-p1, r-p2) is positive, where r is the laser base
 * width Segment length, i.e. width of free space
 * angle angle of the line (robot, frontier center) with the robot local x-axis
 */
struct SFrontier {

	SFrontier(const SPoint &ip1, const SPoint &ip2, const double iwidth, const double iangle):
		p1(ip1),p2(ip2),width(iwidth),angle(iangle) {}

	SPoint p1, p2;
	double width;
	double angle;
};

// Edge of a Voronoi diagram
struct VDEdge {
	double x1,y1,x2,y2;
	int i1, i2;
	VDEdge(const double _x1, const double _y1, const double _x2, const double _y2,
			const int _i1=0, const int _i2=0):
		x1(_x1),y1(_y1),x2(_x2),y2(_y2),i1(_i1),i2(_i2) {}

};

// Crossing center
struct CenterC {
	double x,y;
	double r;
	CenterC(const double _x, const double _y, const double _r):
		x(_x),y(_y),r(_r) {}
};


#endif // #ifndef _LMAP_LOC_TYPES_H_
