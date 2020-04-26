#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
cv2.setUseOptimized(True)

bridge = CvBridge()

# Attempt to filter disparity map with WLS Filter (too slow for Pi currently)
def filter_image(imageL, imageR):
    left = bridge.imgmsg_to_cv2(imageL, desired_encoding='mono8')
    right = bridge.imgmsg_to_cv2(imageR, desired_encoding='mono8')

    left_matcher = cv2.StereoBM_create(numDisparities=16, blockSize=9)

    left_matcher.setPreFilterCap(61)
    left_matcher.setPreFilterSize(5)
    left_matcher.setMinDisparity(0)
    left_matcher.setTextureThreshold(507)
    left_matcher.setUniquenessRatio(0)
    left_matcher.setSpeckleWindowSize(10)
    left_matcher.setSpeckleRange(8)
    left_matcher.setDisp12MaxDiff(10)
    """
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
    right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
    left_disparities = left_matcher.compute(left, right)
    right_disparities = right_matcher.compute(right, left)
    wls_filter.setLambda(4000.0)
    wls_filter.setSigmaColor(1.0)
    filtered_disp = wls_filter.filter(left_disparities, left, disparity_map_right=right_disparities)
    filtered_disp = cv2.normalize(src=filtered_disp, dst=filtered_disp, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX)
    filtered_disp = np.uint8(filtered_disp)
    """
    filtered_disp = left_matcher.compute(left, right).astype('float32')
    filtered_disp *= (255 / np.max(filtered_disp))
    cv2.imshow("filtered_disparities", filtered_disp)
    cv2.waitKey(10)


def listener():
    rospy.init_node('disparity_listener', anonymous=True)
    left_sub = Subscriber("/stereo/left/image_rect_color", Image)
    right_sub = Subscriber("/stereo/right/image_rect_color", Image)
    ats = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.1)
    ats.registerCallback(filter_image)
    #rospy.Subscriber("/control_robot/image", Image, filter_image)
    rospy.spin()


if __name__ == '__main__':
    listener()
