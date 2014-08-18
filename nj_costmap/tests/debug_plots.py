
#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function, division

import numpy as np
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.collections import PatchCollection
from matplotlib.patches import Circle

map_width = 4
map_height = 4

def do_plots():
    def plot_edges(edges, **kwargs):
        segs = []
        for x1, y1, x2, y2 in edges:
            segs.append([[x1, y1], [x2, y2]])
        col = LineCollection(segs, **kwargs)
        ax.add_collection(col)

    def plot_candidates(candidates, **kwargs):
        rmax = candidates[:,2].max()
        patches = []
        for x, y, r in candidates:
            patches.append(Circle((x, y), r))
            if r == rmax:
                # Add the best candidate twice, wo that it will be darker
                # if transparency is used
                patches.append(Circle((x, y), r))
        col = PatchCollection(patches, **kwargs)
        ax.add_collection(col)

    pts = np.loadtxt('pts.dat', delimiter=',')
    filt_pts = np.loadtxt('filt_pts.dat', delimiter=',')
    edges = np.loadtxt('edges.dat', delimiter=',')
    filt_edges = np.loadtxt('filt_edges.dat', delimiter=',')
    candidates = np.loadtxt('candidates.dat', delimiter=',')

    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1, aspect='equal')

    x = np.array([1, -1, -1, 1, 1]) * map_width / 2
    y = np.array([1, 1, -1, -1, 1]) * map_height / 2
    ax.plot(x, y, color='k', alpha=0.5)

    if do_plot_pts:
        plt.plot(pts[:, 0], pts[:, 1], 'r.', label='pts')
    if do_plot_filt_pts:
        plt.plot(filt_pts[:, 0], filt_pts[:, 1], 'b.', label='filt_pts')

    if do_plot_edges:
        plot_edges(edges, color='b', label='edges')
    if do_plot_filt_edges:
        plot_edges(filt_edges, color='g', linewidth=1.5, label='filt_edges')

    if do_plot_candidates:
        plot_candidates(candidates, color=[0, 0, 0.9], alpha=0.1, label='candidates')

    ax.legend()
    ax.grid(True)

    plt.show()

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 2:
        do_plot_pts = 'pts' in sys.argv
        do_plot_filt_pts = 'filt_pts' in sys.argv
        do_plot_edges = 'edges' in sys.argv
        do_plot_filt_edges = 'filt_edges' in sys.argv
        do_plot_candidates = 'candidates' in sys.argv
    else:
        do_plot_pts = False
        do_plot_filt_pts = True
        do_plot_edges = False
        do_plot_filt_edges = True
        do_plot_candidates = False
    do_plots()
