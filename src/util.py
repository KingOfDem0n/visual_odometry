import math
import numpy as np
import cv2 as cv

def onePointHistogram(P1, P2, K, threshold=1):
    assert P1.shape == P2.shape, "P1 shape ({}) does not match P2 shape ({})".format(P1.shape, P2.shape)

    _P1 = P1.copy()
    _P2 = P2.copy()

    if _P1.shape[1] == 3:
        _P1 = _P1.T

    if _P2.shape[1] == 3:
        _P2 = _P2.T

    # Find suitable theta prime ======================================================================
    thetas = -2*np.arctan((_P1[1]*_P2[2] - _P2[1]*_P1[2])/(_P1[0]*_P2[2] + _P2[0]*_P1[2]))
    hist, bin_edges = np.histogram(thetas, bins=50000, range=(-np.pi/2, np.pi/2))  # bin size of 0.5 deg
    theta_p = bin_edges[np.argmax(hist)]
    deg = round(theta_p*180/np.pi, 3)
    print(deg)
    print("======================================")

    # # Create transform and inverse transform matrix ==================================================
    # R = np.array([[np.cos(theta_p), -np.sin(theta_p), 0],
    #               [np.sin(theta_p), np.cos(theta_p), 0],
    #               [0, 0, 1]])
    # t = np.array([[np.cos(theta_p/2)],
    #               [np.sin(theta_p/2)],
    #               [0]])
    # R_inv = R.T
    # t_inv = np.dot(-R_inv, t)
    #
    # T = np.hstack((R, t))
    # T_inv = np.hstack((R_inv, t_inv))
    #
    # # Calculate reprojection error (forward and backward) ==================================================
    # P_homo = np.vstack((P2, np.ones((1, _P.shape[1]))))
    # prev_hat = K.dot(T_inv.dot(P_homo))
    # prev_hat = (prev_hat / prev_hat[2, :])[:2, :]
    #
    # # cur_hat = K.dot(T.dot(P_homo))
    # # cur_hat = (cur_hat / cur_hat[2, :])[:2, :]
    #
    # reproj_error = np.sqrt(np.sum((prev_hat - _prev_p) ** 2, axis=0))
    # print("Minimum reprojection error: {}".format(np.min(reproj_error)))
    #
    # # reproj_error = np.sum((cur_hat - _cur_p) ** 2, axis=0)
    # # print("Minimum reprojection error: {}".format(np.min(reproj_error)))
    #
    # inliers = (reproj_error <= threshold).astype(np.bool_)
    #
    # print("=================================")
    #
    # return inliers, _yaw
