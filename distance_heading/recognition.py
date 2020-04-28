import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

cv.setUseOptimized(True)

bridge = CvBridge()
from common import draw_str, StatValue

sensor_width = 0.00367
sensor_height = 0.00276
face_height = 0.0205
body_height = 1.8

f = 0.00304
body_classifier = cv.CascadeClassifier('haarcascade_frontalface_default.xml')

def recognition(frame):
    left = bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8')
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    bodies = body_classifier.detectMultiScale(gray, 1.1, 3)
    distance_x = 0
    distance_y = 0
    (x, y, w, h) = bodies[1,:]
    cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
    image_height, image_width = frame.shape[:2]

    distance_x = distance_calculation(w, image_width, sensor_width, face_height)
    distance_y = distance_calculation(h, image_height, sensor_height, face_height)

    pub = rospy.Publisher("/distance_heading/distance_angle", Point, queue_size=1)
    rospy.init_node('sender', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("Sending: ", distance_y)
        msg = Point()
        msg.x = distance_y
        pub.publish(msg)
        rate.sleep()

def distance_calculation(box_dim, img_dim, sensor_dim, face_dim):
    real_dim = face_dim * (img_dim/box_dim)

    sensor_side = np.sqrt((sensor_dim/2)*(sensor_dim/2) + f*f)

    pic_side = (real_dim/sensor_dim)*sensor_side

    distance = np.sqrt((pic_side*pic_side)-((real_dim/2)*(real_dim/2)))

    return distance

# def angle_calculation(box_coor, box_dim, img_dim, sensor_dim, face_dim, distance):
#     center -


def listener():
    rospy.init_node('distance_angle', anonymous=True)
    rospy.Subscriber("/stereo/left/image_rect_color", Image, recognition)
    rospy.spin()


if __name__ == '__main__':
    listener()