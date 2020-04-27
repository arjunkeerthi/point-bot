#!/usr/bin/env python

import rospy
import threading
import time
from geometry_msgs.msg import Point

distance = 10


def change_distance():
    global distance
    time.sleep(5)
    distance = 0


def send_distances():
    pub = rospy.Publisher("/control_robot/distance_angle", Point, queue_size=1)
    rospy.init_node('sender', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print("Sending: ", distance)
        msg = Point()
        msg.x = distance
        msg.y = 0.52
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    t = threading.Thread(target=change_distance)
    t.start()
    try:
        send_distances()
    except rospy.ROSInterruptException:
        t.join()

