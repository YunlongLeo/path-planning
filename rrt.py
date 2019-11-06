__author__ = 'Jacky Baltes <jacky@cs.umanitoba.ca>'

import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import numpy as np
import math
import random
import copy
from pathplanning import PathPlanningProblem, Rectangle

def ExploreDomain( domain, initial,blockNO,goals ):
    pos = np.array(initial)
    log=pos
    end = np.array(goals)[0]
    dd = 1
    endlog=end
    while((abs(pos-end)>dd).any()):
        diff = (end - pos)
        delta = 0
        while (True):
            cut=random.uniform(delta/math.pi*180,-delta/math.pi*180)
            theta = np.arctan2(diff[1], diff[0])
            if(cut>0):
                cut+=90
            else:
                cut-=90
            if(delta!=0):
                theta+=cut
            newpos = pos + dd * np.array([dd * math.cos(theta), dd * math.sin(theta)])
            r = Rectangle(newpos[0], newpos[1], dd, dd)
            if ( newpos[0] >= 0.0 ) and ( newpos[0] < domain.width ) and ( newpos[1] >= 0.0 ) and ( newpos[1] < domain.height ):
                if ( not domain.CheckOverlap( r ) ):
                    pos = newpos
                    break
            if(delta<math.pi/2):
                delta+=0.1*math.pi/2
        delta = 0
        while (True):
            cut = random.uniform(delta / math.pi * 180, -delta / math.pi * 180)
            theta = np.arctan2(-diff[1], -diff[0])
            if (cut > 0):
                cut += 90
            else:
                cut -= 90
            if (delta != 0):
                theta += cut
            newend = end + dd * np.array([dd * math.cos(theta), dd * math.sin(theta)])
            r = Rectangle(newend[0], newend[1], dd, dd)
            if (newend[0] >= 0.0) and (newend[0] < domain.width) and (newend[1] >= 0.0) and (
                    newend[1] < domain.height):
                if (not domain.CheckOverlap(r)):
                    end = newend
                    break
            if (delta < math.pi / 2):
                delta += 0.1 * math.pi / 2

        endlog=np.vstack((end,endlog))
        log=np.vstack((log,pos))
    return np.vstack((log,endlog))

def main( argv = None ):
    if ( argv == None ):
        argv = sys.argv[1:]

    width = 100.0
    height = 100.0
    blocknumber = 10
    pp = PathPlanningProblem( width, height, blocknumber, 50.0, 50.0)
    #pp.obstacles = [ Obstacle(0.0, 0.0, pp.width, pp.height / 2.2, '#555555' ) ]
    initial, goals = pp.CreateProblemInstance()

    fig = plt.figure()
    ax = fig.add_subplot(1,2,1, aspect='equal')
    ax.set_xlim(0.0, width)
    ax.set_ylim(0.0, height)

    for o in pp.obstacles:
        ax.add_patch(copy.copy(o.patch) )
    ip = plt.Rectangle((initial[0],initial[1]), 1.0, 1.0, facecolor='#ff0000')
    ax.add_patch(ip)

    for g in goals:
        g = plt.Rectangle((g[0],g[1]), 1.0, 1.0, facecolor='#00ff00')
        ax.add_patch(g)
    path = ExploreDomain( pp, initial, blocknumber, goals)
    ax.set_title('Vacuuming Domain')
    plt.plot(path[:,0], path[:,1], 'r-')
    ax = fig.add_subplot(1,2,2)

    heatmap, x, y = np.histogram2d(path[:,0], path[:,1], bins = 50, range=[[0.0, pp.width], [0.0, pp.height]])
    coverage = float( np.count_nonzero(heatmap) ) / float( len(heatmap) * len(heatmap[0]))
    extent = [ x[0], x[-1], y[0], y[-1]]
    ax.set_title('Random Walk\nCoverage {0}'.format(coverage))
    plt.imshow(np.rot90(heatmap))
    plt.colorbar()

    plt.show()

if ( __name__ == '__main__' ):
    main()

