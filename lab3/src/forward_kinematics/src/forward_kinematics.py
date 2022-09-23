#!/usr/bin/env python

import numpy as np
import scipy as sp
import kin_func_skeleton as kfs

def baxter_forward_kinematics_from_angles(joint_angles):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    angles in radians.

    Parameters
    ----------
    joint_angles ((7x) np.ndarray): 7 joint angles (s0, s1, e0, e1, w0, w1, w2)

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """

    qs = np.ndarray((3,8)) # points on each joint axis in the zero configuration
    ws = np.ndarray((3,7)) # axis vector of each joint axis

    # Assign the q values
    qs[0:3,0] = [0.0635, 0.2598, 0.1188]
    qs[0:3,1] = [0.1106, 0.3116, 0.3885]
    qs[0:3,2] = [0.1827, 0.3838, 0.3881]
    qs[0:3,3] = [0.3682, 0.5684, 0.3181]
    qs[0:3,4] = [0.4417, 0.6420, 0.3177]
    qs[0:3,5] = [0.6332, 0.8337, 0.3067]
    qs[0:3,6] = [0.7152, 0.9158, 0.3063]
    qs[0:3,7] = [0.7957, 0.9965, 0.3058]

    # Assign the w values
    ws[0:3,0] = [-0.0059,  0.0113,  0.9999]
    ws[0:3,1] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,3] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,5] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                  [-0.7040, 0.7102, -0.0053],
                  [0.7102, 0.7040, 0.0055]]).T # rotation matrix of zero config

    # YOUR CODE HERE (Task 1)
    vs = np.ndarray((3,7))
    for i in range(7):
        # print("ws: ", np.expand_dims(ws[:,i], axis=0), "\tshape: ", np.expand_dims(ws[:,i], axis=0).shape)
        # print("qs: ", np.expand_dims(qs[:,i], axis=0), "\tshape: ", np.expand_dims(qs[:,i], axis=0).shape)
        vs[:,i] = np.squeeze(np.transpose(np.cross(-np.expand_dims(ws[:,i], axis=0), np.expand_dims(qs[:,i], axis=0))), axis=1)

    xis = np.vstack((vs, ws))
    assert xis.shape[0] == 6 and xis.shape[1] == 7, \
        print(f"Got it wrong: # Rows for xis is {xis.shape[0]} and # Cols for xis is {xis.shape[1]}.")

    g_st = kfs.prod_exp(xis, joint_angles)

    g_0 = np.eye(4)
    g_0[0:3,:] = np.hstack((R, np.transpose([[0.7957, 0.9965, 0.3058]])))
    
    g_st = g_st @ g_0
    return g_st

def baxter_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of Baxter robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    # YOUR CODE HERE (Task 2)

    # s0, s1, e0, e1, w0, w1, w2
    jsp = joint_state.position
    angles = np.array([jsp[4], jsp[5], jsp[2], jsp[3], jsp[6], jsp[7], jsp[8]])

    # END YOUR CODE HERE
    print(baxter_forward_kinematics_from_angles(angles))