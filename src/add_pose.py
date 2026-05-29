import math
import numpy as np
import gtsam
from gtsam.symbol_shorthand import L, X

PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))  # (x, y, theta)
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.1]))  # (dx, dy, dtheta)
MEASUREMENT_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.1]))  # (bearing, range)

def add_pose(graph, initial_estimate):
    dx = 2.0 * np.cos(np.pi / 4.0)
    dy = 2.0 * np.sin(np.pi / 4.0)
    dtheta = np.pi / 2.0
    
    odometry = gtsam.Pose2(dx, dy, dtheta)


    graph.add(gtsam.BetweenFactorPose2(X(3), X(4), odometry, ODOMETRY_NOISE))

    pose4Ideal = gtsam.Pose2(4.0 + np.sqrt(2), np.sqrt(2), np.pi / 2.0)
    
    if initial_estimate.exists(X(4)):
        initial_estimate.update(X(4), pose4Ideal)
    else:
        initial_estimate.insert(X(4), pose4Ideal)
    
    return graph, initial_estimate