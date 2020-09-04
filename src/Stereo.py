import numpy as np
from math import sin, cos
import cv2 as cv
import matplotlib.pyplot as plt
from copy import deepcopy

# ROS imports
import rospy
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
from visual_odometry.srv import getImage

def getImages():
    # rospy.loginfo("Waiting for imageGrabber server...")
    rospy.wait_for_service('imageGrabber')
    try:
        server = rospy.ServiceProxy('imageGrabber', getImage)
        return server()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

lk_params = dict(winSize=(21, 21),
                 maxLevel=3,
                 criteria=(cv.TERM_CRITERIA_EPS |
                           cv.TERM_CRITERIA_COUNT, 20, 0.03))

def extract_keypoints_ros():
    bridge = CvBridge()
    data = getImages()
    rgb = bridge.imgmsg_to_cv2(data.rgb, desired_encoding='bgr8')
    gray = cv.cvtColor(rgb, cv.COLOR_BGR2GRAY)
    detector = cv.xfeatures2d.SURF_create(400)
    kps = detector.detect(gray, None)
    point2D = [(int(round(x.pt[0])), int(round(x.pt[1]))) for x in kps]
    point3D = np.array(list(pc2.read_points(data.point, uvs=point2D)))
    point2D = np.array(point2D)

    return point3D, point2D, gray

# TODO: Make a featureTracking function to track feature points. This will make it looks significantly cleaner

class Stereo(object):
    def __init__(self):
        self.K = np.array([[570.3405151367188, 0.0, 314.5],
                           [0.0, 570.3405151367188, 235.5],
                           [0.0, 0.0, 1.0]], dtype=np.float32)

        self.keyPoint = {"2D": np.array([]),
                         "3D": np.array([])}
        self.prevInvTransform = np.hstack((np.eye(3), np.ones((3, 1))))
        self.prevFrameL = None
        self.prevState = {"2D": np.array([]),
                          "3D": np.array([])}

    def initialize(self):
        point3D, point2D, grayL = extract_keypoints_ros()
        self.prevState["2D"] = point2D.copy()
        self.prevState["3D"] = point3D.copy()
        self.prevFrameL = grayL.copy()

        return  point3D, point2D

    def saveNewKeyPoints(self):
        point3D, point2D, _ = extract_keypoints_ros()
        self.keyPoint["2D"] = point2D.copy()
        self.keyPoint["3D"] = point3D.copy()

    def nextFrame(self):
        _, _, curFrame = extract_keypoints_ros()
        p_prev = self.prevState["2D"]
        P = self.prevState["3D"]

        p_prev = p_prev.astype(np.float32)

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
            P = self.keyPoint["3D"].copy()
            p_prev = self.keyPoint["2D"].copy()
            P = np.vstack((P.T, np.ones((1, P.shape[0]))))
            P = self.prevInvTransform.dot(P).T
            p_prev = p_prev.astype(np.float32)

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

        if p_cur.shape[0] > 30:
            _, _R, _t, inliers = cv.solvePnPRansac(P, p_cur, self.K, None, flags=cv.SOLVEPNP_EPNP)
            R, _ = cv.Rodrigues(_R)
            t = -R.T.dot(_t)
            inv_transform = np.hstack((R.T, t.reshape((3, 1))))

            if inliers is not None:
                inliers = inliers.squeeze()
                p_prev = p_prev[inliers]
                p_cur = p_cur[inliers]
                P = P[inliers]
            else:
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
        self.prevState["2D"] = p_cur.copy()
        self.prevState["3D"] = P.copy()

        self.saveNewKeyPoints()

        return R, t, curFrame
