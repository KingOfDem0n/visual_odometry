#!/usr/bin/env python

import os
import numpy as np
import cv2 as cv
import time
import FeatureManagement as FM

blob_kernel = np.array([[-1, -1, -1, -1, -1],
                        [-1, 1, 1, 1, -1],
                        [-1, 1, 8, 1, -1],
                        [-1, 1, 1, 1, -1],
                        [-1, -1, -1, -1, -1]])

corner_kernel = np.array([[-1, -1, 0, 1, 1],
                         [-1, -1, 0, 1, 1],
                         [0, 0, 0, 0, 0],
                         [1, 1, 0, -1, -1],
                         [1, 1, 0, -1, -1]])

def efficientNMS(img, r=30):
    fail_flag = False
    pts = []
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

    return sorted(pts, reverse=True, key=lambda x: x[0])

path = os.path.join(os.getcwd(), ".." , "dataset", "sequences", "00")
grayL = cv.imread(os.path.join(path, "image_0", "{:06d}.png".format(0)), 0)

blob_mask = cv.filter2D(grayL, -1, blob_kernel)
corner_mask = cv.filter2D(grayL, -1, corner_kernel)

begin = time.time()
blob_pts = efficientNMS(blob_mask, 5)
corner_pts = efficientNMS(corner_mask, 5)
end = time.time()
python = end-begin
print("Python Time: {} sec".format(end-begin))

begin = time.time()
blob_pts = FM.efficientNMS(blob_mask, 5)
corner_pts = FM.efficientNMS(corner_mask, 5)
end = time.time()
cython = end-begin
print("Cython Time: {} sec".format(end-begin))

print("Improved by {}".format(cython/python))
