# The program optimizes the sum of given segments (as 3d vectors) to approximate given target point, using quaternions.
# The segments orientation is limited by Jil's machine - all the segments must rotate around the same axis, determined by the target point.

import numpy as np
from scipy.optimize import minimize, LinearConstraint


def multiplyQuaternions(p, q):
    res = np.array([p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
                    p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
                    p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
                    p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0]])
    return res

def norm(p):
    return np.sqrt(np.sum(p ** 2))

def inverseQuaternion(p):
    return np.array([p[0], -p[1], -p[2], -p[3]]) / norm(p) ** 2

def crossProduct(u, v):
    return np.array([u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]])

# Optimization cost function
def func(x):
    th1 = x[0]
    th2 = x[1]
    q1 = np.array([np.cos(th1), targetBaseRotationAxis[0] * np.sin(th1), targetBaseRotationAxis[1] * np.sin(th1), targetBaseRotationAxis[2] * np.sin(th1)])
    q2 = np.array([np.cos(th2), targetBaseRotationAxis[0] * np.sin(th2), targetBaseRotationAxis[1] * np.sin(th2), targetBaseRotationAxis[2] * np.sin(th2)])
    q3 = np.array([np.cos(th3), targetBaseRotationAxis[0] * np.sin(th3), targetBaseRotationAxis[1] * np.sin(th3), targetBaseRotationAxis[2] * np.sin(th3)])
    p = np.insert(basePoint, 0, 0)
    segmentsSum = p + multiplyQuaternions(multiplyQuaternions(q1, segments[0]), inverseQuaternion(q1)) + \
                multiplyQuaternions(multiplyQuaternions(q2, segments[1]), inverseQuaternion(q2)) + \
                multiplyQuaternions(multiplyQuaternions(q3, segments[2]), inverseQuaternion(q3))

    # if (p[3] + multiplyQuaternions(multiplyQuaternions(q1, segments[0]), inverseQuaternion(q1))[3] <= 0 or
    #     p[3] + multiplyQuaternions(multiplyQuaternions(q1, segments[0]), inverseQuaternion(q1))[3] + multiplyQuaternions(multiplyQuaternions(q2, segments[1]), inverseQuaternion(q2))[3] <= 0):
    #     err = np.inf
    # else:
    err = np.sum((segmentsSum[1:] - targetPoint) ** 2)
    return err


### Global variables and initializations ###
basePoint = np.zeros((3,), dtype = float)
targetPoint = np.zeros((3,), dtype = float)
startingBaseRotationAxis = np.zeros((3,), dtype = float)
targetBaseRotationAxis = np.zeros((3,), dtype = float)
th3 = 0
segments = np.zeros((3,4), dtype = float)

def calculateOrientation(base, target, segmentLengths, startingBaseRotationAngle, startingSegmentsRotationAngles, lastSegmentAngle, x0):
    global basePoint, targetPoint, startingBaseRotationAxis, targetBaseRotationAxis, th3, segments

    phi_starting = np.radians(startingBaseRotationAngle)
    th3 = np.radians(lastSegmentAngle) / 2
    basePoint = base
    targetPoint = target
    startingBaseRotationAxis = np.array([np.cos(phi_starting), np.sin(phi_starting), 0])
    targetBaseRotationAxis = -crossProduct(targetPoint - basePoint, np.array([0, 0, 1]))      # always contained in xy plane
    targetBaseRotationAxis = targetBaseRotationAxis / norm(targetBaseRotationAxis)
    if np.isnan(targetBaseRotationAxis).any():
        targetBaseRotationAxis = np.array([-1, 0, 0])
    segments[0,3] = segmentLengths[0]
    segments[1,3] = segmentLengths[1]
    segments[2,3] = segmentLengths[2]
    
    '''
    ### Optimization method 1 ###
    linear_constraint = LinearConstraint([2, 0], [-np.radians(40)], [np.radians(110)])
    res = minimize(func, x0, method='trust-constr', constraints=[linear_constraint],
                    options={'verbose': 1, 'xtol': 1e-10})
    ###
    '''

    ### Optimization method 2 ###
    ineq_cons  = {'type': 'ineq',
                  'fun' : lambda x: np.array([2 * x[0] + np.radians(40), np.radians(110) - 2 * x[0]])}

    res = minimize(func, x0, method='SLSQP',
                    constraints=[ineq_cons],
                    options={'ftol': 1e-30, 'maxiter': 1000})
    ###
    

    segmentAngles = np.degrees(res.x * 2) % 360
    # segmentAngles = np.append(segmentAngles, lastSegmentAngle)
    # print(f'Segments rotation angles: {str(segmentAngles)}')
    baseAngle = np.degrees(np.arctan2(targetBaseRotationAxis[1], targetBaseRotationAxis[0]))
    # print(f'Base rotation angle: {str(baseAngle)}')

    p = basePoint
    points = np.zeros((len(segmentLengths)+1, 3), dtype = float)
    points[0] = p
    for i in range(len(segmentLengths)):
        if i < 2:
            q = np.array([np.cos(res.x[i]), targetBaseRotationAxis[0] * np.sin(res.x[i]), targetBaseRotationAxis[1] * np.sin(res.x[i]), targetBaseRotationAxis[2] * np.sin(res.x[i])])
        else:
            q = np.array([np.cos(th3), targetBaseRotationAxis[0] * np.sin(th3), targetBaseRotationAxis[1] * np.sin(th3), targetBaseRotationAxis[2] * np.sin(th3)])
        vec = multiplyQuaternions(multiplyQuaternions(q, segments[i]), inverseQuaternion(q))
        p = p + vec[1:]
        points[i+1] = p

    # print(points)

    ###  Segments length verification ###
    # for i in range(len(segmentLengths)):
    #     d = np.sqrt(np.sum((points[i+1] - points[i]) ** 2))
    #     print(d)

    score = func(res.x)
    return baseAngle, segmentAngles, points, targetBaseRotationAxis, score
