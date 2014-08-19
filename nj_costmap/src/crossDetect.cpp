/*
 * Utility functions to detect crossing center and radius
 */

#include <cmath>
#include <map>
#include <cstdint>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

#include "nj_costmap/crossDetect.h"
#include "nj_costmap/lmap_loc_types.h"
#include "nj_costmap/polygonUtils.h"
#include "Voronoi.h"
#include "ANN/ANN.h"

//#define DEBUG_CROSSDETECT

#ifdef DEBUG_CROSSDETECT
#include <fstream>
#endif

namespace lama {
namespace Laloc {

// A map to store previously computed distances with ANN
// first 16 bit = x * 100, last 16 bit = y * 100
typedef std::map<uint32_t, double> freeSpace_dict;

// A map to store ray castings
typedef std::map<double, vector<size_t>> raycast_dict;

/* Converts a scan to a list of Voronoi::Points
 */
std::vector<Voronoi::Point> scanToVpoint(const std::vector<double>& scan)
{
	vector<Voronoi::Point> vpts;
	double scan_angle = FAKE_SCAN_START_ANGLE;
	for(unsigned int i = 0; i < scan.size(); ++i)
	{
		double x = scan[i] * std::cos(scan_angle);
		double y = scan[i] * std::sin(scan_angle);
		vpts.push_back(Voronoi::Point(x, y));
		scan_angle += FAKE_SCAN_RESOLUTION;
	}
	return vpts;
}

/* Return true if the map point is occupied
 */
inline bool pointOccupied(const nav_msgs::OccupancyGrid& map, const int index)
{
	// Points with unknown occupancy (-1) are also considered as occupied.
	return (map.data[index] > OCCUPIED_THRESHOLD || map.data[index] == -1);
}

/* Return true if the point lies in the map
 */
inline bool pointInMap(const int row, const int col, const size_t nrow, const size_t ncol)
{
	return ((0 <= col) && (col < ncol) &&
			(0 <= row) && (row < nrow));
}

/* Return the row number from offset for a row-major array
 */
inline size_t rowFromOffset(const size_t offset, const size_t ncol)
{
	return offset / ncol;
}

/* Return the column number from offset for a row-major array
 */
inline size_t colFromOffset(const size_t offset, const size_t ncol)
{
	return offset % ncol;
}

/* Return the offset from row and column number for a row-major array
 */
inline size_t offsetFromRowCol(const size_t row, const size_t col, const size_t ncol)
{
	return (row * ncol) + col;
}

/* Return the angle of the line from map center to the given pixel
 */
inline double pixelAngle(const size_t row, const size_t col, const size_t nrow, const size_t ncol)
{
	return std::atan2((double) row - nrow / 2, (double) col - ncol / 2);
}

/* Return the list of pixel indexes from map center to pixel at map border and given angle
 *
 * The Bresenham algorithm is used for rendering.
 *
 * angle[in] beam angle
 * nrow[in] image height
 * ncol[in] image width
 */
vector<size_t> getRayCastToMapBorder(const double angle, const size_t nrow, const size_t ncol)
{
	vector<size_t> pts;

	// Twice the distance from map center to map corner.
	const double r = std::sqrt((double) nrow * nrow + ncol * ncol);
	// Start point, map center.
	// TODO: the sensor position (map origin)  may not be the map center
	int x0 = ncol / 2;
	int y0 = nrow / 2;
	// End point, outside the map.
	int x1 = (int) std::lround(x0 + r * std::cos(angle)); // Can be negative
	int y1 = (int) std::lround(y0 + r * std::sin(angle));
	int dx = x1 - x0;
	int dy = y1 - y0;
	bool steep = (std::abs(dy) >= std::abs(dx));
	if (steep)
	{
		swap(x0, y0);
		swap(x1, y1);
		// recompute Dx, Dy after swap
		dx = x1 - x0;
		dy = y1 - y0;
	}
	int xstep = 1;
	if (dx < 0)
	{
		xstep = -1;
		dx = -dx;
	}
	int ystep = 1;
	if (dy < 0)
	{
		ystep = -1;
		dy = -dy;
	}
	int twoDy = 2 * dy;
	int twoDyTwoDx = twoDy - 2 * dx; // 2*Dy - 2*Dx
	int e = twoDy - dx; //2*Dy - Dx
	int y = y0;
	int xDraw, yDraw;
	for (int x = x0; x != x1; x += xstep)
	{
		if (steep)
		{
			xDraw = y;
			yDraw = x;
		}
		else
		{
			xDraw = x;
			yDraw = y;
		}
		if (pointInMap(yDraw, xDraw, nrow, ncol))
		{
			pts.push_back(offsetFromRowCol(yDraw, xDraw, ncol));
		}
		else
		{
			// We exit when the first point outside the map is encountered.
			return pts;
		}
		// next
		if (e > 0)
		{
			e += twoDyTwoDx; //E += 2*Dy - 2*Dx;
			y = y + ystep;
		}
		else
		{
			e += twoDy; //E += 2*Dy;
		}
	}
}

/* Return the pixel representation of the nearest ray
 *
 * raycast_lookup[in] map (ray angle --> pixel list)
 * angle[in] laser beam angle
 */
vector<size_t> rayLookup(const raycast_dict& raycast_lookup, const double angle)
{
	double dangle_lower;
	double dangle_upper;

	auto upper_bound = raycast_lookup.upper_bound(angle);
	if (upper_bound == raycast_lookup.begin())
	{
		return upper_bound->second;
	}
	else if (upper_bound == raycast_lookup.end())
	{
		dangle_upper = raycast_lookup.begin()->first - angle + 2 * M_PI;
		upper_bound--;
		dangle_lower = upper_bound->first - angle;
		if (dangle_lower < dangle_upper)
		{
			return upper_bound->second;
		}
		else
		{
			return raycast_lookup.begin()->second;
		}
	}
	else
	{
		dangle_upper = upper_bound->first - angle;
		auto lower_bound = upper_bound;
		lower_bound--;
		dangle_lower = angle - lower_bound->first;
		if (dangle_lower < dangle_upper)
		{
			return lower_bound->second;
		}
		else
		{
			return upper_bound->second;
		}
	}
}

/* Return the pixel list by ray casting from map center to map border
 *
 * angle[in] laser beam angle
 * nrow[in] image height in pixel
 * ncol[in] image width in pixel
 */
vector<size_t> getRayCast(const double angle, const size_t nrow, const size_t ncol)
{
	// Store the ray casting up to the map border into a look-up table. Ray
	// casting exclusively depends on the ray angle.
	static map<double, vector<size_t>> raycast_lookup;
	static bool raycast_lookup_cached;

	if (!raycast_lookup_cached)
	{
		// Get the largest angle, to get one pixel resolution at image corner.
		double max_angle = std::max(pixelAngle(nrow, ncol, nrow, ncol) - pixelAngle(nrow -1, ncol, nrow, ncol),
				pixelAngle(nrow, ncol - 1, nrow, ncol) - pixelAngle(nrow, ncol, nrow, ncol));
		double angle_resolution = max_angle / 4;  // Use a better resolution

		double last_angle = FAKE_SCAN_START_ANGLE + 2 * M_PI;
		for (double a = FAKE_SCAN_START_ANGLE; a <= last_angle; a += angle_resolution)
		{
			raycast_lookup[a] = getRayCastToMapBorder(a, nrow, ncol);
		}
		raycast_lookup_cached = true;
	}
	return rayLookup(raycast_lookup, angle);
}

/* Return the range for one laser ray.
 *
 * Return the range for one laser ray, calculated by raytracing from the map
 * center to the map border with the Bresenham algorithm and stopping at the first
 * encountered occupied point. Returns the distance rmax
 * if no occupied point is encountered before reaching the map
 * borders.
 *
 * map[in] occupancy grid 
 * angle[in] angle of the laser ray
 * rmax[in] max laser range
 */
double getRayRange(const nav_msgs::OccupancyGrid& map, const double angle, const double rmax)
{
	vector<size_t> ray = getRayCast(angle, map.info.height, map.info.width);
	for (auto idx : ray)
	{
		if (pointOccupied(map, idx))
		{
			double xcenter = (map.info.width / 2) * map.info.resolution;
			double ycenter = (map.info.height / 2) * map.info.resolution;
			size_t row = rowFromOffset(idx, map.info.height);
			size_t col = colFromOffset(idx, map.info.width);
			double xobstacle = col * map.info.resolution;
			double yobstacle = row * map.info.resolution;
			double dx = xobstacle - xcenter;
			double dy = yobstacle - ycenter;
			return std::sqrt(dx * dx + dy * dy);
		}
	}
	return rmax;
}

/* Return a simulated 360° ranger output
 */
std::vector<double> mapToScan(const nav_msgs::OccupancyGrid& map)
{
	std::vector<double> scan;
	scan.reserve((size_t) std::ceil(2 * M_PI / FAKE_SCAN_RESOLUTION));
	double halfx = (double) map.info.width / 2;
	double halfy = (double) map.info.height / 2;
	double rmax = std::sqrt(halfx * halfx + halfy * halfy) * map.info.resolution;
	double max_angle = FAKE_SCAN_START_ANGLE + 2 * M_PI;
	for (double angle = FAKE_SCAN_START_ANGLE; angle <= max_angle; angle += FAKE_SCAN_RESOLUTION)
	{
		scan.push_back(getRayRange(map, angle, rmax));
	}
	return scan;
}

/* return frontiers
 *
 * scan laser ranges
 * rt max range for a laser beam to be considered
 * dt min frontier width
 * maxFrontierAngle max. allowed frontier angle in radians (0 means angle
 *                  between frontier and line from robot to frontier middle is
 *                  90 deg).
 * frontiers returned frontiers
 */
void getFrontiers(
	const vector<double> & scan, const double rt, const double dt, const double maxFrontierAngle,
	std::vector<SFrontier> & frontiers)
{
	vector<double> filtScan;
	vector<int> angleNumber;

	// Filter out laser beams longer than rt
	filtScan.reserve(scan.size());
	angleNumber.reserve(scan.size());
	for(int i = 0; i < scan.size(); i++) {
		if (scan[i] < rt) {
			filtScan.push_back(scan[i]);
			angleNumber.push_back(i);
		}
	}


	if (filtScan.size() == 0) {
		ROS_ERROR("All points from scan are above threshold");
		return;
	}

	if (scan.size() < 2)
	{
		ROS_ERROR("laser scan must have at least 2 points");
		return;
	}

	double aAngle, bAngle;
	SPoint a, b;
	aAngle = angleNumber[0] * FAKE_SCAN_RESOLUTION;
	a.x = filtScan[0] * std::cos(FAKE_SCAN_START_ANGLE + aAngle);
	a.y = filtScan[0] * std::sin(FAKE_SCAN_START_ANGLE + aAngle);

	double dist;
	frontiers.clear();

	for(int i = 1; i < filtScan.size() + 1; i++)
	{
		int j = i % filtScan.size();
		bAngle = angleNumber[j] * FAKE_SCAN_RESOLUTION;
		b.x = filtScan[j] * std::cos(FAKE_SCAN_START_ANGLE + bAngle);
		b.y = filtScan[j] * std::sin(FAKE_SCAN_START_ANGLE + bAngle);
		dist = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);

		if (dist > dt*dt)
		{
			double sx = (a.x + b.x) / 2.0;
			double sy = (a.y + b.y) / 2.0;

			double distToFrontierCenter = std::sqrt(sx * sx + sy * sy);
			double dotProductFrontierSxSy = (b.x - a.x) * sx + (b.y - a.y) * sy;
			double frontierAngleWithSxSy = std::acos(dotProductFrontierSxSy / dist / distToFrontierCenter);
			if (fabs(M_PI_2 - frontierAngleWithSxSy) < maxFrontierAngle)
			{
				frontiers.push_back(SFrontier(a, b, std::sqrt(dist), std::atan2(sy, sx)));
			}
		}
		a.x = b.x;
		a.y = b.y;
		aAngle = bAngle;
	}
}


/* Return the index of the first value in an array greater than a value
 *
 * array[in] input array, must be partitioned
 * a[in] value for which a close value in array is searched
 */
int getIndex(const vector<double>& array, const double a)
{
	// Adapted from http://en.cppreference.com/w/cpp/algorithm/upper_bound.
    int count;
    count = array.size();
	int step;
 
	int i;
	int i_smaller = 0;
    while (count > 0)
	{
        i = i_smaller; 
        step = count / 2; 
        i += step;
        if (!(a < array[i]))
		{
            i_smaller = ++i;
            count -= step + 1;
        }
		else
		{
		   	count = step;
		}
    }
	return i;
}

/**
  * remove all edges that are behind scan points
  */
void filterVoronoiEdges(vector<VDEdge>& edges, const vector<Voronoi::Point>& pts)
{

	vector<VDEdge> newEdges;

	vector<double> angles;
	angles.reserve(pts.size());

	double a;  // Point angle
	double dr; // Point distance
	for(auto pt : pts)
	{
		a = std::atan2(pt.y, pt.x);
		angles.push_back(a);
	}

	const int pSize = pts.size();
	
	for(auto edge : edges)
	{
		const double x1 = edge.x1;
		const double y1 = edge.y1;
		const double x2 = edge.x2;
		const double y2 = edge.y2;

		// If some point in edge is far then actual scan in given
		// angle, do not put such edge to vector
		a = atan2(y1, x1);
		dr = x1 * x1 + y1 * y1;

		// We can compare angles because both angles and a are within ]-pi,pi].
		int i = getIndex(angles, a) % pSize;
		bool far = (pts[i].x*pts[i].x + pts[i].y*pts[i].y < dr);
		if (!far)
		{
			i = (i - 1) % pSize;
			far = (pts[i].x*pts[i].x + pts[i].y*pts[i].y < dr);
		}

		if (!far)
		{
			a = std::atan2(y2, x2);
			dr = x2 * x2 + y2 * y2;

			i = getIndex(angles, a) % pSize;
			far = (pts[i].x*pts[i].x + pts[i].y*pts[i].y < dr);
			if (!far)
			{
				i = (i - 1) % pSize;
				far = (pts[i].x*pts[i].x + pts[i].y*pts[i].y < dr);
			}
		}

		if (!far) 
			newEdges.push_back(VDEdge(x1, y1, x2, y2));
	}
	edges.clear();
	edges = newEdges;		
}

/* Return the integer representing a point
 *
 * The first 16 bit are a signed integer equal to the x-coordinate multiplied
 * by 100. The last 16 bit are for y.
 */
void encodePoint(const double x, const double y, uint32_t& code)
{
	if (((x * 100) < (((double) INT16_MIN) + 1)) ||
			((x * 100) > (((double) INT16_MAX) - 1)) ||
			((y * 100) < (((double) INT16_MIN) + 1)) ||
			((y * 100) > (((double) INT16_MAX) - 1)))
	{
		ROS_ERROR("Point (%f, %f) cannot be encoded", x, y);
	}
	code = 0;
	int16_t xint = (int16_t) std::lround(x * 100);
	int16_t yint = (int16_t) std::lround(y * 100);
	code |= (xint << 16);
	code |= (yint & 0x0000FFFF);
}

/* Return the point represented by an integer
 *
 * The first 16 bit of the coded integer are a signed integer equal to the
 * x-coordinate multiplied by 100. The last 16 bit are for y.
 */
void decodePoint(const uint32_t code, double& x, double& y)
{
	int16_t xint = (int16_t) (code >> 16);
	int16_t yint = (int16_t) code;
	x = ((double) xint) / 100;
	y = ((double) yint) / 100;
}

// Return the largest circle (x, y, radius^2) of free space around each vertex.
//
// edges[in] edges, the vertices of which are the circle centers
// pts[in] pts representing obstacles
vector<CenterC> getFreeSpace(const vector<VDEdge>& edges, const vector<Voronoi::Point>& pts)
{
	// build KD-tree from the pts array 
	const int k = 1; // Max. number of nearest neighbors
	ANNidxArray idx = new ANNidx[k];
	ANNdistArray dist = new ANNdist[k];
	ANNpoint query = annAllocPt(2);	

	ANNpointArray ap = annAllocPts(pts.size(), 2);
	for(unsigned i = 0; i < pts.size(); ++i)
	{
		ap[i][0] = pts[i].x;
		ap[i][1] = pts[i].y;
	}

	ANNkd_tree* tree = new ANNkd_tree(ap, pts.size(), 2);

	// freeSpace_d is a map used to saved already computed nearest neighbors,
	// allowing to improve the comparison by computing each node only once.
	freeSpace_dict freeSpace_d;
	unsigned int ann_count = 0;
	for(auto edge : edges)
	{
		uint32_t codedPoint;
		encodePoint(edge.x1, edge.y1, codedPoint);
		freeSpace_dict::const_iterator search = freeSpace_d.find(codedPoint);
		if (search == freeSpace_d.end())
		{
			// Point was not yet calculated.
			query[0] = edge.x1;
			query[1] = edge.y1;
			// TODO: test eps value (last parameter of annkSearch)
			// TODO: try tree->annkFRSearch (same as annkSearch with max. radius)
			tree->annkSearch(query, k, idx, dist, 0.005);
			ann_count++;
			if (dist[0] > 0)
			{
				freeSpace_d[codedPoint] = dist[0];
			}
		}
		encodePoint(edge.x2, edge.y2, codedPoint);
		search = freeSpace_d.find(codedPoint);
		if (search == freeSpace_d.end())
		{
			query[0] = edge.x2;
			query[1] = edge.y2;
			tree->annkSearch(query, k, idx, dist, 0.005);
			++ann_count;
			if (dist[0] > 0)
			{
				freeSpace_d[codedPoint] = dist[0];
			}
		}
	}
	ROS_DEBUG("Number of nearest neighbor searches %u", ann_count);

	vector<CenterC> freeSpace;
	freeSpace.reserve(freeSpace_d.size());
	for (auto pair : freeSpace_d)
	{
		double x, y;
		decodePoint(pair.first, x, y);
		freeSpace.push_back(CenterC(x, y, pair.second));
	}
	ROS_DEBUG("Number of crossing center candidates: %lu", freeSpace.size());

	delete [] idx;
	delete [] dist;
	delete tree;
	annDeallocPt(query);
	annDeallocPts(ap);
	annClose();

	return freeSpace;
}

/**
  * return the crossing center determined by Voronoi diagram.
  * The center is returned as triplet (cx,cy,r) where cx,cy is the center
  * of circle with radius 'r'. if no center is detected, the triplet (0,0,-1)
  * is returned.
  */
void getXingDesc(
		const nav_msgs::OccupancyGrid& map, const double dt, const double maxFrontierAngle,
		double& cx, double& cy, double& radius, vector<SFrontier>& frontiers)
{
	if (map.data.size() == 0)
	{
		frontiers.clear();
		cx = 0;
		cy = 0;
		radius = -1;
		return;
	}

	// Get the Voronoi diagram. The crossing center will be one of the Voronoi nodes.
	double relevanceFilterRadius = 0.15;
	std::vector<double> scan = mapToScan(map);
	std::vector<Voronoi::Point> pts = scanToVpoint(scan);
	std::vector<Voronoi::Point> filt_pts(lama::PolygonUtils::filterRelevance(pts, relevanceFilterRadius));

	ROS_DEBUG("Number of scan points: %zu", pts.size());
	ROS_DEBUG("Number of filtered scan points: %zu", filt_pts.size());
		
	// delta = extra space around the map, so that no point are at the boundary
	// defined below.
	double delta = 2;
	double maxx = (map.info.width / 2) * map.info.resolution;
	double maxy = (map.info.height / 2) * map.info.resolution;
	double maxr = maxx * maxx + maxy * maxy;
	maxx = maxr;
	maxy = maxr;
	double minx = -maxx;
	double miny = -maxy;

	Voronoi::VoronoiDiagramGenerator *vd = new Voronoi::VoronoiDiagramGenerator();
	vd->generateVoronoi(&filt_pts, minx - delta, maxx + delta, miny - delta, maxy + delta, 0);
	vd->resetIterator();

	vector<VDEdge> edges;
 	Voronoi::GraphEdge ge;
	unsigned int edge_count = 0;
	while(vd->getNext(ge))
	{
		if ( ge.x1 > minx && ge.y1 > miny && ge.x1 < maxx && ge.y1 < maxy && 
			   ge.x2 > minx && ge.y2 > miny && ge.x2 < maxx && ge.y2 < maxy) 
			// Add the edge if both vertices are in the bounding box.
			edges.push_back(VDEdge(ge.x1, ge.y1, ge.x2, ge.y2));
		edge_count++;
	}
	ge.next = NULL;

	ROS_DEBUG("Number of graph edges: %u", edge_count);
	ROS_DEBUG("Number of Voronoi edges: %zu", edges.size());

	// remove edges behind scan points
	vector<VDEdge> filt_edges(edges);
	filterVoronoiEdges(filt_edges, pts);

	ROS_DEBUG("Number of Voronoi edges after filtering = %zu", filt_edges.size());

	// Get the max. radius of free space around each Voronoi node.
	vector<CenterC> candidates = getFreeSpace(filt_edges, pts);

#ifdef DEBUG_CROSSDETECT
	// Cf. tests/debug_plots.py
	// "cd /tmp; python $(rospack find nj_costmap)/tests/debug_plots.py"
	std::ofstream ofs_pts("/tmp/pts.dat");
	for (auto pt : pts)
	{
		ofs_pts << pt.x << "," << pt.y << std::endl;
	}
	ofs_pts.close();
	std::ofstream ofs_filt_pts("/tmp/filt_pts.dat");
	for (auto pt : filt_pts)
	{
		ofs_filt_pts << pt.x << "," << pt.y << std::endl;
	}
	ofs_filt_pts.close();

	std::ofstream ofs_edges("/tmp/edges.dat");
	for (auto edge : edges)
	{
		ofs_edges << edge.x1 << "," << edge.y1 << "," << edge.x2 << "," << edge.y2 << std::endl;
	}
	ofs_edges.close();

	std::ofstream ofs_filt_edges("/tmp/filt_edges.dat");
	for (auto edge : filt_edges)
	{
		ofs_filt_edges << edge.x1 << "," << edge.y1 << "," << edge.x2 << "," << edge.y2 << std::endl;
	}
	ofs_filt_edges.close();

	std::ofstream ofs_candidates("/tmp/candidates.dat");
	for (auto c : candidates)
	{
		ofs_candidates << c.x << "," << c.y << "," << c.r << std::endl;
	}
	ofs_candidates.close();
#endif

	// Find the Voronoi vertex with the largest free space around it, this will
	// be the crossing center.
	unsigned int idx_maxr = 0;
	for(unsigned int i = 0; i < candidates.size(); i++)
	{
		if (candidates[i].r > candidates[idx_maxr].r)
		{
			idx_maxr = i;
		}
	}

	if (candidates.size() > 0)
	{
		cx = candidates[idx_maxr].x;
		cy = candidates[idx_maxr].y;
		radius = sqrt(candidates[idx_maxr].r);
	}
	else
	{
		cx = 0;
		cy = 0;
		radius = -1;
	}

	// Get the frontiers.
	getFrontiers(scan, 1.4 * radius, dt, maxFrontierAngle, frontiers);
	ROS_DEBUG("Number of frontiers: %zu", frontiers.size());
	
	delete vd;
}


} // namespace Laloc
} // namespace lama
