import math
import numpy as np

def onePointHistogram(P1, P2, K, threshold=1):
    assert P1.shape == P2.shape, "P1 shape ({}) does not match P2 shape ({})".format(P1.shape, P2.shape)

    _P1 = P1.copy()
    _P2 = P2.copy()

    if _P1.shape[1] == 3:
        _P1 = _P1.T

    if _P2.shape[1] == 3:
        _P2 = _P2.T

    P1_norm = _P1 / _P1[2, :]
    P2_norm = _P2 / _P2[2, :]

    # Find suitable theta prime ======================================================================
    thetas = -2*np.arctan2(P2_norm[1]*P1_norm[2] - P1_norm[1]*P2_norm[2], P2_norm[0]*P1_norm[2] + P1_norm[0]*P2_norm[2])
    hist, bin_edges = np.histogram(thetas, bins=360, range=(-math.pi/2.0, math.pi/2.0))  # bin size of 0.5 deg
    theta_p = bin_edges[np.argmax(hist)]

    # Create transform and inverse transform matrix ==================================================
    R = np.array([[np.cos(theta_p), -np.sin(theta_p), 0],
                  [np.sin(theta_p), np.cos(theta_p), 0],
                  [0, 0, 1]])
    t = np.array([[np.cos(theta_p/2)],
                  [np.sin(theta_p/2)],
                  [0]])
    R_inv = R.T
    t_inv = np.dot(-R_inv, t)

    T = np.hstack((R, t))
    T_inv = np.hstack((R_inv, t_inv))

    # Calculate reprojection error (forward and backward) ==================================================
    P1_homo = np.vstack((_P1, np.ones((1, _P1.shape[1]))))
    P2_homo = np.vstack((_P2, np.ones((1, _P2.shape[1]))))

    P2_homo_hat = K.dot(T.dot(P1_homo))
    P2_norm_hat = (P2_homo_hat / P2_homo_hat[2, :])[:2, :]

    P1_homo_hat = K.dot(T_inv.dot(P2_homo))
    P1_norm_hat = (P1_homo_hat / P1_homo_hat[2, :])[:2, :]

    forward_reproj_error = np.sqrt(np.sum((P2_norm_hat - P2_norm) ** 2, axis=0))
    backward_reproj_error = np.sqrt(np.sum((P1_norm_hat - P1_norm) ** 2, axis=0))

    forward_count = forward_reproj_error <= threshold
    backward_count = backward_reproj_error <= threshold

    inliers = (forward_count*backward_count).astype(np.bool_)

    return R, t, inliers
