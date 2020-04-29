import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

cv.setUseOptimized(True)

bridge = CvBridge()
#from common import draw_str, StatValue

sensor_width = 0.00367
sensor_height = 0.00276
face_height = 0.0205
body_height = 1.8

f = 0.00304
body_classifier = cv.CascadeClassifier('haarcascade_frontalface_default.xml')

pub = None

def recognition(frame):
    left = bridge.imgmsg_to_cv2(frame, desired_encoding='bgr8')
    gray = cv.cvtColor(left, cv.COLOR_BGR2GRAY)
    bodies = body_classifier.detectMultiScale(gray, 1.1, 3)
    distance_x = 0
    distance_y = 0
    (x, y, w, h) = bodies[0,:]
    cv.rectangle(left, (x, y), (x + w, y + h), (0, 255, 255), 2)
    image_height, image_width = left.shape[:2]

    distance_x = distance_calculation(w, image_width, sensor_width, face_height)
    distance_y = distance_calculation(h, image_height, sensor_height, face_height)
    angle = angle_calculation(x, w, image_width, sensor_width)

    pub = rospy.Publisher("/distance_heading/distance_angle", Point, queue_size=1)
    msg = Point()
    print("Distance: ", distance_y)
    print("Angle: ", angle)
    msg.x = distance_y
    msg.y = angle
    pub.publish(msg)


def distance_calculation(box_dim, img_dim, sensor_dim, face_dim):
    real_dim = face_dim * (img_dim/box_dim)

    sensor_side = np.sqrt((sensor_dim/2)*(sensor_dim/2) + f*f)

    pic_side = (real_dim/sensor_dim)*sensor_side

    distance = np.sqrt((pic_side*pic_side)-((real_dim/2)*(real_dim/2)))

    return distance

def angle_calculation(box_coor, box_dim, img_dim, sensor_dim):
    right = True
    center = img_dim / 2
    diff_pix = center - (box_coor + (box_dim/2))
    print("center: ", center)
    print("box_coor: ", box_coor)
    print("box_dim: ", box_dim)
    print("diff_pix: ", diff_pix)
    if diff_pix > 0:
        right = False
    diff_pix = abs(diff_pix)
    diff_real = (float(diff_pix) / img_dim) * sensor_dim
    print("img_dim: ", img_dim)
    print("sensor_dim: ", sensor_dim)
    print("diff_real: ", diff_real)
    heading = np.arcsin(diff_real / f)
    print("heading: ", heading)
    if not right:
        heading = -1 * heading

    return heading


def listener():
    rospy.init_node('distance_angle', anonymous=True)
    rospy.Subscriber("/stereo/raspicam_node/image", Image, recognition)
    rospy.spin()


if __name__ == '__main__':
    pub = rospy.Publisher("/distance_heading/distance_angle", Point, queue_size=1)
    listener()
