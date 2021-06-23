from math import radians
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
from QuaternionsOrientation import *

# A function found online (https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to)
# Used to scale the plot axes equally
def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def approximateTargetPoint(targetPoint, baseRotationAngle, segmentsRotationAngles, lastSegmentAngle):
    global previousBaseRotationAngle, previousSegmentsRotationAngles, currentBaseRotationAngle, currentSegmentsRotationAngles

    currentBaseRotationAngle, currentSegmentsRotationAngles, points, baseRotationAxis, score = calculateOrientation(basePoint, targetPoint, segmentLengths, baseRotationAngle, segmentsRotationAngles, lastSegmentAngle, x0)

    ###  Results plot ###
    plt.ion()
    fig = plt.figure()
    plt.show()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(points[...,0], points[...,1], points[...,2], 'orange', linewidth=3)
    ax.scatter(points[1:,0], points[1:,1], points[1:,2], marker='o', s=20, c='r', facecolors='r')
    ax.scatter(basePoint[0], basePoint[1], basePoint[2], marker='o', s=30, c='k', facecolors='k')
    ax.scatter(targetPoint[0], targetPoint[1], targetPoint[2], marker='x', s=50, c='c', facecolors='c', linewidth=4)
    scaledBaseRotationAxis = np.sqrt(np.sum(targetPoint ** 2)) / 10 * baseRotationAxis
    p = plt.Circle((0, 0), np.sqrt(np.sum(scaledBaseRotationAxis ** 2)), fc='orange', ec='orange', alpha = 0.4)
    ax.add_patch(p)
    art3d.pathpatch_2d_to_3d(p, z=basePoint[2], zdir="z")
    ax.plot3D([basePoint[0],scaledBaseRotationAxis[0]], [basePoint[1],scaledBaseRotationAxis[1]], [basePoint[2],basePoint[2]], 'black', linewidth=3)

    props = dict(boxstyle='round', facecolor='white')
    for i in range(len(segmentLengths)+1):
        point_coordinates_text = '(%g, %g, %g)' % (points[i,0], points[i,1], points[i,2])
        ax.text3D(points[i,0], points[i,1], points[i,2], point_coordinates_text, fontsize=8, bbox=props)

    end_point_coordinates_text = '(%g, %g, %g)' % (targetPoint[0], targetPoint[1], targetPoint[2])
    ax.text3D(targetPoint[0], targetPoint[1], targetPoint[2], end_point_coordinates_text, fontsize=8, bbox=props)
    err_text = 'Target point: (%g,%g,%g)\nEnd point: (%g,%g,%g)\n\nRMS Error:\n%g' % (targetPoint[0], targetPoint[1], targetPoint[2], points[-1,0], points[-1,1], points[-1,2], np.sqrt(score))
    props = dict(boxstyle='round', facecolor='red', alpha=0.5)
    ax.text2D(0.7, 1.1, err_text, fontsize=10, ha = 'left', va = 'top', transform=ax.transAxes, bbox=props) 

    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    set_axes_equal(ax)
    plt.pause(1)

    baseAngleDiff = (currentBaseRotationAngle - previousBaseRotationAngle) % 360
    segmentAnglesDiff = (currentSegmentsRotationAngles - previousSegmentsRotationAngles) % 360
    print(f'Base rotation angle: {str(baseAngleDiff)} deg.')
    print(f'Segments rotation angles: {str(segmentAnglesDiff)} deg.')
    previousBaseRotationAngle = currentBaseRotationAngle
    previousSegmentsRotationAngles = currentSegmentsRotationAngles

    return baseRotationAngle, segmentsRotationAngles


### Global variables and initializations ###
x0 = np.array([1.4, 2])
basePoint = np.array([0, 0, 50])
segmentLengths = np.array([100, 100, 50])
previousBaseRotationAngle = 0
previousSegmentsRotationAngles = np.array([0, 0])

targetPoint = np.array([20, 130, 50])
approximateTargetPoint(targetPoint, previousBaseRotationAngle, previousSegmentsRotationAngles, 180)

targetPoint = np.array([-20, -130, 50])
approximateTargetPoint(targetPoint, previousBaseRotationAngle, previousSegmentsRotationAngles, 180)

while True:
    if plt.waitforbuttonpress():
        plt.close('all')
        break
