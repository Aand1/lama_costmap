from __future__ import print_function, division

from pylab import *
import numpy as np

from matplotlib.collections import LineCollection

def plot_edges(edges, **kwargs):
    segs = []
    for x1, y1, x2, y2 in edges:
        segs.append([[x1, y1], [x2, y2]])
    col = LineCollection(segs, **kwargs)
    ax.add_collection(col)



filt_edges = np.loadtxt('/tmp/filt_edges.dat', delimiter=',')
figure()
ax = gca()
plot_edges(filt_edges)
show()
x = np.hstack((filt_edges[:,0], filt_edges[:,2]))
y = np.hstack((filt_edges[:,1], filt_edges[:,3]))
xy = np.vstack((x, y)).transpose()
for pt in xy:
    print(count_nonzero(np.all(pt == xy, axis=1)))

