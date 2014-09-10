#include <nj_costmap/polygonUtils.h>

namespace lama{
namespace PolygonUtils {

double getRelevance(const Point &pi, const Point &pj, const Point &pk)
{
	double rel, dx,dy;
	dx = pi.x - pj.x;
	dy = pi.y - pj.y;
	rel = sqrt(dx*dx + dy*dy);
	dx = pj.x - pk.x;
	dy = pj.y - pk.y;
	rel += sqrt(dx*dx + dy*dy);
	dx = pi.x - pk.x;
	dy = pi.y - pk.y;
	rel -= dx*dx+ dy*dy;
	return std::fabs(rel);
}

double getRelevance(const list< S<double> > &s, 
		typename list< S<double> >::const_iterator i, 
		const std::vector<Point> &pts)
{
	typename list< S<double> >::const_iterator pred = i;
	typename list< S<double> >::const_iterator succ = i;
	pred--;
	succ++;

	double rel = -1;
	if (i != s.begin() && succ != s.end()) {
		rel = getRelevance(pts[pred->idx],pts[i->idx],pts[succ->idx]);
	} 
	return rel;
}

std::vector<Point> filterRelevance(const std::vector<Point> &pts, const double maxR)
{
	typedef typename list< S<double> >::iterator Iterator;
	typedef typename list< S<double> >::const_iterator Const_iterator;
	if (pts.size() < 3)
		return pts;

	list< S<double> > r;
	double dx,dy,rel;
	double maxRel = -1;

	for(int j=0;j<pts.size();j++) {
		if (j > 0 && j < pts.size()-1) {
			rel = getRelevance(pts[j-1], pts[j], pts[j+1]);
		} else 
			rel = -1;
		r.push_back(S<double>(j,rel));
		if (rel > maxRel || maxRel == -1)
			maxRel = rel;
	}
	r.front().r = 2*maxRel;
	r.back().r = 2*maxRel;

	double minR,rr;
	do {
		Iterator me = std::min_element(r.begin(),r.end(),myLess< S<double> >());
		minR = me->r;
		if (me != r.begin() && me != r.end()) {
			Iterator i = me;
			i--;
			Iterator j = me;
			j++;
			r.erase(me);
			if (i != r.begin() && i != r.end()) {
				rr = getRelevance(r,i,pts);
				if (rr >= 0)
					i->r = rr;
			}
			if (j != r.begin() && j != r.end()) {
				rr = getRelevance(r,j,pts);
				if (rr >= 0)
					j->r = rr;
			}
		}
		if (r.size() <= 3)
			break;
	} while (minR < maxR);
	
	std::vector<Point> res;
	res.reserve(r.size());
	for(Const_iterator i = r.begin(); i != r.end(); i++)
		res.push_back(pts[i->idx]);

	return res;
}

} // namespace PolygonUtils
} // namesapce lama

