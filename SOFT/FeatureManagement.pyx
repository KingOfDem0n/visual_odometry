#!/usr/bin/env python

import numpy as np
cimport numpy as np
import cv2 as cv

def removeDuplicate(queryPoints, refPoints, radius=5):
    # remove duplicate points from new query points,
    for i in range(len(queryPoints)):
        query = queryPoints[i]
        xliml, xlimh = query[0] - radius, query[0] + radius
        yliml, ylimh = query[1] - radius, query[1] + radius
        inside_x_lim_mask = (refPoints[:, 0] > xliml) & (refPoints[:, 0] < xlimh)
        # print(np.array(inside_x_lim_mask).shape)
        # print(refPoints.shape)
        # print("=========")
        curr_kps_in_x_lim = refPoints[inside_x_lim_mask]

        if curr_kps_in_x_lim.shape[0] != 0:
            inside_y_lim_mask = (curr_kps_in_x_lim[:, 1] > yliml) & (curr_kps_in_x_lim[:, 1] < ylimh)
            curr_kps_in_x_lim_and_y_lim = curr_kps_in_x_lim[inside_y_lim_mask, :]
            if curr_kps_in_x_lim_and_y_lim.shape[0] != 0:
                queryPoints[i] = np.array([0, 0])
    return (queryPoints[:, 0] != 0)

def extract_keypoints_surf(img1, img2, K, baseline, refPoints=None, method=None, grid=(5,5)):
    # Reference: https://github.com/snmnmin12/VisualOdometry/blob/master/python/odometry3D.py
    # h, w = img1.shape[:2]
    # incrementH = h // grid[0]
    # incrementW = w // grid[1]
    # kp1, kp2 = [], []
    # desc1, desc2 = None, None

    detector = cv.xfeatures2d.SURF_create(400)
    # detector = cv.xfeatures2d.SIFT_create()
    # detector = cv.ORB_create(nfeatures=1000)
    # detector = cv.FastFeatureDetector_create()
    kp1, desc1 = detector.detectAndCompute(img1, None)
    kp2, desc2 = detector.detectAndCompute(img2, None)
    # for i in range(grid[0]):
    #     for j in range(grid[1]):
    #         mask = np.zeros_like(img1)
    #         rowTop = i * incrementH
    #         rowBot = (i + 1) * incrementH
    #         colTop = j * incrementW
    #         colBot = (j + 1) * incrementW
    #         mask[rowTop:rowBot, colTop:colBot] = 1
    #         kp1_, desc1_ = detector.detectAndCompute(img1, mask)
    #         kp2_, desc2_ = detector.detectAndCompute(img2, mask)
    #         if len(kp1_) > 0 and len(kp2_) > 0:
    #             kp1 += kp1_
    #             kp2 += kp2_
    #             if desc1 is None:
    #                 desc1 = desc1_.copy()
    #                 desc2 = desc2_.copy()
    #             else:
    #                 desc1 = np.vstack((desc1, desc1_))
    #                 desc2 = np.vstack((desc2, desc2_))

    # FLANN parameters
    FLANN_INDEX_KDTREE = 1 # Norm_L2
    FLANN_INDEX_LSH = 6 # HAMMING_L2
    flann_params = dict(algorithm = FLANN_INDEX_KDTREE,
                        trees=5,
                       table_number = 6, # 12
                       key_size = 12,     # 20
                       multi_probe_level = 1) #2
    search_params = dict()  # or pass empty dictionary
    flann = cv.FlannBasedMatcher(flann_params, search_params)
    matches = flann.knnMatch(desc1, desc2, k=2)

    # ratio test as per Lowe's paper
    match_points1, match_points2 = [], []

    for i, (m, n) in enumerate(matches):
        if m.distance < 0.7 * n.distance:
            match_points1.append(kp1[m.queryIdx].pt)
            match_points2.append(kp2[m.trainIdx].pt)

    p1 = np.array(match_points1).astype(float)
    p2 = np.array(match_points2).astype(float)

    if refPoints is not None:
        mask = removeDuplicate(p1, refPoints)
        p1 = p1[mask, :]
        p2 = p2[mask, :]

    if method is None:
        M_left = K.dot(np.hstack((np.eye(3), np.zeros((3, 1)))))
        M_rght = K.dot(np.hstack((np.eye(3), np.array([[-baseline, 0, 0]]).T)))

        p1_flip = np.vstack((p1.T, np.ones((1, p1.shape[0]))))
        p2_flip = np.vstack((p2.T, np.ones((1, p2.shape[0]))))

        P = cv.triangulatePoints(M_left, M_rght, p1_flip[:2], p2_flip[:2])
        P = P / P[3]
        land_points = P[:3]
    else:
        land_points = []
        for _p1, _p2 in zip(p1, p2):
            P, _ = method(_p1, _p2)
            land_points.append(P)
        land_points = np.array(land_points).squeeze().T

    return land_points.T, p1

cdef sort_key(x):
    return x[0]

# A more efficient implement exist: https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=1699659
cpdef efficientNMS(np.ndarray img, int r=30):
    fail_flag = False
    cdef int i, j, i2, j2, mi, mj
    cdef np.ndarray upper_bound1, upper_bound2, lower_bound1, lower_bound2
    cdef list pts = []
    for i in range(r, img.shape[0]-r, 2*r+1 - r):
        for j in range(r, img.shape[1]-r, 2*r+1 - r):
            fail_flag = False
            mi, mj = i, j
            upper_bound1 = np.array(range(i, i+r)).astype(np.int16)
            lower_bound1 = np.array(range(j, j+r)).astype(np.int16)
            for (i2,j2) in zip(upper_bound1,lower_bound1):
                if img[i2, j2] > img[mi, mj]:
                    mi, mj = i2, j2
            upper_bound2 = np.array(range(mi-r, mi+r)).astype(np.int16)
            lower_bound2 = np.array(range(mj-r, mj+r)).astype(np.int16)
            upper_bound2 = upper_bound2[~np.isin(upper_bound2,upper_bound1)]
            lower_bound2 = lower_bound2[~np.isin(lower_bound2,lower_bound1)]
            upper_bound2 = upper_bound2[upper_bound2 < img.shape[0]]
            lower_bound2 = lower_bound2[lower_bound2 < img.shape[1]]
            for (i2,j2) in zip(upper_bound2, lower_bound2):
                if img[i2, j2] > img[mi, mj]:
                    fail_flag = True
                    break
            if not fail_flag:
                pts.append((img[mi, mj], mj, mi))

    return sorted(pts, reverse=True, key=sort_key)

cpdef SAD(np.ndarray img, int r, tuple pt1, tuple pt2):
    cdef np.ndarray m_filtered = cv.copyMakeBorder(img.copy(), r, r, r, r, cv.BORDER_CONSTANT)
    cdef np.ndarray patch1 = m_filtered[pt1[1]:pt1[1]+2*r+1, pt1[0]:pt1[0]+2*r+1]
    cdef np.ndarray patch2 = m_filtered[pt2[1]:pt2[1]+2*r+1, pt2[0]:pt2[0]+2*r+1]
    cdef float result = np.sum(patch1-patch2).astype(np.float32)
    return result

cpdef circularMatching(np.ndarray imgL_prev, np.ndarray imgR_prev, np.ndarray imgL_cur, np.ndarray imgR_cur, tuple pts):
    pass

# def bucketing(frame, sup_r, nfeature, grid=(5,5)):
#     sub_nfeatures = nfeature // (grid[0] * grid[1])
#     kp_corner = []
#     kp_blob = []
#     h, w = frame.shape
#     incrementH = h // grid[0]
#     incrementW = w // grid[1]
#     R_blob = cv.filter2D(frame, -1, blob_kernel)
#     R_corner = cv.filter2D(frame, -1, blob_kernel)
#     for i in range(grid[0]):
#         for j in range(grid[1]):
#             mask = np.zeros_like(frame)
#             _R_blob = R_blob.copy()
#             _R_corner = R_corner.copy()
#             rowTop = i * incrementH
#             rowBot = (i + 1) * incrementH
#             colTop = j * incrementW
#             colBot = (j + 1) * incrementW
#             mask[rowTop:rowBot, colTop:colBot] = 1
#             _R_blob *= mask
#             _R_corner *= mask
#             _pts_blob = nonMaximumSuppression(_R_blob, sup_r, sub_nfeatures)
#             _pts_corner = nonMaximumSuppression(_R_corner, sup_r, sub_nfeatures)
#             if len(_pts_blob) > 0:
#                 kp_blob += _pts_blob
#             if len(_pts_corner) > 0:
#                 kp_corner += _pts_corner
#
#     return kp_blob, kp_corner


# path = os.path.join(os.getcwd(), ".." , "dataset", "sequences", "00")
# grayL = cv.imread(os.path.join(path, "image_0", "{:06d}.png".format(0)), 0)
#
# blob_mask = cv.filter2D(grayL, -1, blob_kernel)
# corner_mask = cv.filter2D(grayL, -1, corner_kernel)
#
# begin = time.time()
# blob_pts = efficientNMS(blob_mask)
# corner_pts = efficientNMS(corner_mask)
# end = time.time()
# print("Time: {} sec".format(end-begin))
