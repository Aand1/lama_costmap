#ifndef _LMAP_LOC_TYPES_H_
#define _LMAP_LOC_TYPES_H_

namespace lama {
namespace nj_costmap {

// Edge of a Voronoi diagram
struct VDEdge {
	double x1;
  double y1;
  double x2;
  double y2;
	int i1;
  int i2;
	VDEdge(const double _x1, const double _y1, const double _x2, const double _y2,
			const int _i1=0, const int _i2=0) :
		x1(_x1), y1(_y1), x2(_x2), y2(_y2), i1(_i1), i2(_i2) {}

};

// Crossing center
struct CenterC
{
	double x;
  double y;
	double r;
	CenterC(const double _x, const double _y, const double _r) :
		x(_x), y(_y), r(_r) {}
};

} // namespace nj_costmap
} // namespace lama

#endif // #ifndef _LMAP_LOC_TYPES_H_
