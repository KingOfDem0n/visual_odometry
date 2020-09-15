import numpy as np
from math import sin, cos
import math
import cv2 as cv
import matplotlib.pyplot as plt
from copy import deepcopy

# ROS imports
import rospy
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from visual_odometry.srv import getImage

lk_params = dict(winSize=(21, 21),
                 maxLevel=3,
                 criteria=(cv.TERM_CRITERIA_EPS |
                           cv.TERM_CRITERIA_COUNT, 20, 0.03)) # Change 20 to 100

ransacPnP_params = dict(useExtrinsicGuess=True,
                        iterationsCount=250,
                        reprojectionError=1,
                        confidence=0.999,
                        flags=cv.SOLVEPNP_ITERATIVE) # SOLVEPNP_ITERATIVE, SOLVEPNP_P3P, SOLVEPNP_EPNP, SOLVEPNP_DLS

detection_method = "SURF"

def getImages():
    # rospy.loginfo("Waiting for imageGrabber server...")
    rospy.wait_for_service('imageGrabber')
    try:
        server = rospy.ServiceProxy('imageGrabber', getImage)
        return server()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

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
                pts.append((mj, mi))

    return np.array(pts)

def extract_keypoints_ros(method="SURF", frame=None, justFrame=False):
    assert method in ["SURF", "SOFT"], "Incorrect keypoint extraction method passed in"

    data = None
    gray = None
    point2D = None
    point3D = None

    if frame is None:
        bridge = CvBridge()
        data = getImages()
        while len(data.rgb.data) == 0 or len(data.point.data) == 0:
            data = getImages()
        rgb = bridge.imgmsg_to_cv2(data.rgb, desired_encoding='bgr8')
        gray = cv.cvtColor(rgb, cv.COLOR_BGR2GRAY)
    else:
        gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)

    if not justFrame:
        if method == "SURF":
            detector = cv.xfeatures2d.SURF_create(400)
            kps = detector.detect(gray, None)
            point2D = [(int(round(x.pt[0])), int(round(x.pt[1]))) for x in kps]
        elif method == "SOFT":
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

            blob_mask = cv.filter2D(gray, -1, blob_kernel)
            corner_mask = cv.filter2D(gray, -1, corner_kernel)

            blob_pts = efficientNMS(blob_mask, 30)
            corner_pts = efficientNMS(corner_mask, 30)

            point2D = np.vstack((blob_pts, corner_pts)).tolist()

        point3D = np.array(list(pc2.read_points(data.point, uvs=point2D)))
        point2D = np.array(point2D)

    return point3D, point2D, gray, data.point

class Stereo(object):
    def __init__(self):
        self.K = np.array([[570.3405151367188, 0.0, 314.5],
                           [0.0, 570.3405151367188, 235.5],
                           [0.0, 0.0, 1.0]], dtype=np.float32)

        self.keyPoint = {"2D": np.array([]),
                         "3D": np.array([])}
        self.prevInvTransform = np.hstack((np.eye(3), np.ones((3, 1))))
        self.prevFrameL = None
        self.prevPointCloud = None
        self.prevState = {"2D": np.array([]),
                          "3D": np.array([])}

    def initialize(self, frame=None):
        point3D, point2D, grayL, pointCloud = extract_keypoints_ros(detection_method, frame=frame)
        self.prevState["2D"] = point2D.copy()
        self.prevState["3D"] = point3D.copy()
        self.prevFrameL = grayL.copy()
        self.prevPointCloud = pointCloud

        return point3D, point2D

    def saveNewKeyPoints(self, frame=None):
        point3D, point2D, _ = extract_keypoints_ros(detection_method, frame=frame)
        self.keyPoint["2D"] = point2D.copy()
        self.keyPoint["3D"] = point3D.copy()

    def getNewKeyPoints(self, method="SURF"):
        assert method in ["SURF", "SOFT"], "Incorrect keypoint extraction method passed in"

        point2D = None
        point3D = None

        if method == "SURF":
            detector = cv.xfeatures2d.SURF_create(400)
            kps = detector.detect(self.prevFrameL, None)
            point2D = [(int(round(x.pt[0])), int(round(x.pt[1]))) for x in kps]
        elif method == "SOFT":
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

            blob_mask = cv.filter2D(self.prevFrameL, -1, blob_kernel)
            corner_mask = cv.filter2D(self.prevFrameL, -1, corner_kernel)

            blob_pts = efficientNMS(blob_mask, 30)
            corner_pts = efficientNMS(corner_mask, 30)

            point2D = np.vstack((blob_pts, corner_pts)).tolist()

        point3D = np.array(list(pc2.read_points(self.prevPointCloud, uvs=point2D)))
        point2D = np.array(point2D)

        return point2D, point3D

    def featureTracking(self, curFrame, p_prev, P):
        p_cur, status, err = cv.calcOpticalFlowPyrLK(self.prevFrameL, curFrame, p_prev, None, **lk_params)
        p_cur = p_cur[status.ravel() == 1]
        p_prev = p_prev[status.ravel() == 1]
        P = P[status.ravel() == 1]
        p_prev_r, _, _ = cv.calcOpticalFlowPyrLK(curFrame, self.prevFrameL, p_cur, None, **lk_params)

        # Filter out occluded point
        d = abs(p_prev - p_prev_r).reshape(-1, 2).max(-1)
        P = P[d < 1]
        p_cur = p_cur[d < 1]
        p_prev = p_prev[d < 1]

        return p_prev, p_cur, P

    def nextFrame(self, frame=None):
        _, _, curFrame, pointCloud = extract_keypoints_ros(detection_method, frame=frame, justFrame=True)

        p_prev = self.prevState["2D"]
        P = self.prevState["3D"]

        p_prev = p_prev.astype(np.float32)

        #p_prev, p_cur, P = self.featureTracking(curFrame, p_prev, P)

        p_cur, status, err = cv.calcOpticalFlowPyrLK(self.prevFrameL, curFrame, p_prev, None, **lk_params)
        p_cur = p_cur[status.ravel() == 1]
        p_prev = p_prev[status.ravel() == 1]
        P = P[status.ravel() == 1]
        p_prev_r, _, _ = cv.calcOpticalFlowPyrLK(curFrame, self.prevFrameL, p_cur, None, **lk_params)

        # Filter out occluded point
        d = abs(p_prev - p_prev_r).reshape(-1, 2).max(-1)
        P = P[d < 1]
        p_cur = p_cur[d < 1]
        p_prev = p_prev[d < 1]

        if p_cur.shape[0] < 50:
            # P = self.keyPoint["3D"].copy()
            # p_prev = self.keyPoint["2D"].copy()
            p_prev, P = self.getNewKeyPoints(method=detection_method)
            P = np.vstack((P.T, np.ones((1, P.shape[0]))))
            P = self.prevInvTransform.dot(P).T
            p_prev = p_prev.astype(np.float32)

            # p_prev, p_cur, P = self.featureTracking(curFrame, p_prev, P)

            p_cur, status, err = cv.calcOpticalFlowPyrLK(self.prevFrameL, curFrame, p_prev, None, **lk_params)
            p_cur = p_cur[status.ravel() == 1]
            p_prev = p_prev[status.ravel() == 1]
            P = P[status.ravel() == 1]
            p_prev_r, _, _ = cv.calcOpticalFlowPyrLK(curFrame, self.prevFrameL, p_cur, None, **lk_params)

            # Filter out occluded point
            d = abs(p_prev - p_prev_r).reshape(-1, 2).max(-1)
            P = P[d < 1]
            p_cur = p_cur[d < 1]
            p_prev = p_prev[d < 1]

        if p_cur.shape[0] > 10:
            _, _R, _t, inliers = cv.solvePnPRansac(P, p_cur, self.K, None, **ransacPnP_params)
            R, _ = cv.Rodrigues(_R)
            t = -R.T.dot(_t)
            inv_transform = np.hstack((R.T, t.reshape((3, 1))))

            if inliers is not None:
                inliers = inliers.squeeze()
                p_prev = p_prev[inliers]
                p_cur = p_cur[inliers]
                P = P[inliers]
            else:
                print("No inliers passes")
                inliers = np.array([])
                p_prev = np.array([])
                p_cur = np.array([])
                P = np.array([])
        else:
            R = self.prevInvTransform[:,:3].copy()
            t = self.prevInvTransform[:,3].copy()
            inv_transform = self.prevInvTransform.copy()
            print("Skipped frame")

        # Update previous frame and new state
        self.prevInvTransform = inv_transform.copy()
        self.prevFrameL = curFrame.copy()
        self.prevPointCloud = pointCloud
        self.prevState["2D"] = p_cur.copy()
        self.prevState["3D"] = P.copy()

        # self.saveNewKeyPoints(frame=curFrame)

        return R.T, t, curFrame
