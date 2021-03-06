#!/usr/bin/env python

import rospy
#import cv2
import numpy as np
import RPi.GPIO as GPIO
import time
import threading
#from cv_bridge import CvBridge
from geometry_msgs.msg import Point
#from sensor_msgs.msg import Image
#from message_filters import ApproximateTimeSynchronizer, Subscriber

#cv2.setUseOptimized(True)

#bridge = CvBridge()

# Setup
GPIO.setmode(GPIO.BOARD)
IN1 = 15
IN2 = 16
IN3 = 18
IN4 = 22
ENR = 33
ENL = 32
ENCL = 11
ENCR = 13

# Setup pins for motors (1,2 -> right, 3,4 -> left)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)
GPIO.setup(ENR, GPIO.OUT)
GPIO.setup(ENL, GPIO.OUT)

# Setup pins for encoders
GPIO.setup(ENCL, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(ENCR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Setup pins for pwm on enable lines
motorR = GPIO.PWM(ENR, 100)
motorR.start(0)
motorL = GPIO.PWM(ENL, 100)
motorL.start(0)

# Store ticks overall and during each interval
left_ticks = 0
right_ticks = 0
curr_left_ticks = 0
curr_right_ticks = 0

# Robot parameters
diameter = 0.067 # meters
ticks_per_rev = 20
drive_radius = 0.3 # meters
wheel_base = 0.131 # meters
speed_scale_factor = 10
depth_scale_factor = 0.1

# Initially, want robot at rest
target_left_ticks = 0
target_right_ticks = 0

# PID constants
kp = 0.5
kd = 0.2
ki = 0.1

# Interval between correction updates
interval = 0.8 # seconds

# Estimate for depth straight ahead from robot (middle of image from webcam)
# Initialize to some high value
middle_depth = 10000

# Flag to stop robot
stop = False

# Handle for thread
move_thread = None

# Turned
turn = False

# Calculate the left and right wheel velocities in order to turn a given angle
# with a specified radius for the arc driven and wheel base
# Reference: http://robotsforroboticists.com/drive-kinematics
def get_left_velocity(angle):
    return angle * (drive_radius + wheel_base / 2)


def get_right_velocity(angle):
    return angle * (drive_radius - wheel_base / 2)


def velocity_to_ticks(vel):
    return vel * ticks_per_rev * interval / diameter


def calc_speed(ticks, time):
    return (diameter * ticks) / (ticks_per_rev * time)


# Callbacks to record every tick encoder reads from wheels
def left_encoder_callback(channel):
    global left_ticks, curr_left_ticks
    left_ticks += 1
    curr_left_ticks += 1


def right_encoder_callback(channel):
    global right_ticks, curr_right_ticks
    right_ticks += 1
    curr_right_ticks += 1


def move():
    global curr_left_ticks, curr_right_ticks
    GPIO.add_event_detect(ENCL, GPIO.RISING, callback=left_encoder_callback, bouncetime=10)
    GPIO.add_event_detect(ENCR, GPIO.RISING, callback=right_encoder_callback, bouncetime=10)

    # Start the robot at rest
    left_duty = 0
    right_duty = 0
    left_prev_error = 0
    right_prev_error = 0
    left_sum_error = 0
    right_sum_error = 0
    left_end_ticks = 0
    right_end_ticks = 0
    left_adj = 0
    total_time = 0

    #while total_time < 8:
    while stop is False:
        motorL.ChangeDutyCycle(left_duty)
        motorR.ChangeDutyCycle(right_duty)
        curr_left_ticks = 0
        curr_right_ticks = 0

        # Right (forward: 1=low, 2=high)
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        GPIO.output(ENR, GPIO.HIGH)

        # Left (forward: 3=low, 4=high)
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
        GPIO.output(ENL, GPIO.HIGH)
        time.sleep(0.8)

        # PID correction

        # Compute error
        left_error = target_left_ticks - curr_left_ticks
        right_error = target_right_ticks - curr_right_ticks
        print("curr_left_ticks: ", curr_left_ticks)
        print("curr_right_ticks: ", curr_right_ticks)
        print("curr_left_speed: ", calc_speed(curr_left_ticks, 0.8))
        print("curr_right_speed: ", calc_speed(curr_right_ticks, 0.8))
        print("left_error: ", kp*left_error)
        print("right_error: ", kp*right_error)

        # Add correction
        left_duty = min(max(left_duty + kp*left_error + kd*left_prev_error + ki*left_sum_error, 0), 100)
        right_duty = min(max(right_duty + kp*right_error + kd*right_prev_error + ki*right_sum_error, 0), 100)
        print("left_duty: ", left_duty)
        print("right_duty: ", right_duty)
        print()

        # Update errors for derivative and integral terms
        left_prev_error = left_error
        right_prev_error = right_error
        left_sum_error += left_error
        right_sum_error += right_error
        total_time += interval
        #if total_time > 6:
        #    left_end_ticks += curr_left_ticks
        #    right_end_ticks += curr_right_ticks
        #if total_time > 1:
        #    left_adj = 0

    # Stop
    #print("error between wheels: ", abs(left_end_ticks - right_end_ticks))
    print("Total time: ", total_time);
    GPIO.output(ENR, GPIO.LOW)
    GPIO.output(ENL, GPIO.LOW)
    motorR.stop()
    motorL.stop()
    GPIO.cleanup()
    print("Left ticks: ", left_ticks)
    print("Right ticks: ", right_ticks)

callback_count = 0

def motor_callback(point):
    global target_left_ticks, target_right_ticks, drive_radius, stop, callback_count, turn

    callback_count += 1
    distance = point.x
    print("Distance received: ", distance)
    angle = point.y if not turn else 0
    print("Angle received: ", angle)

    if callback_count > 5:
        turn = True

    # If we want to continue going straight (angle ~ 0), then just set
    # vel_left = vel_right
    if abs(angle) < 0.008 and distance > 0.02:
        vel_left = 0.2
        vel_right = 0.2
    elif angle < 0:
        vel_right = get_left_velocity(abs(angle))
        vel_left = get_right_velocity(abs(angle))
    else:
        vel_left = get_left_velocity(angle)
        vel_right = get_right_velocity(angle)
    print("Initial vel_left: ", vel_left)
    # If distance to destination is less than 0.02 meters (1 in), stop
    # If distance is greater than 1 meter, cut off to 1 for speed purposes
    distance = min(distance, 1)
    if distance < 0.005:
        stop = True
        distance = 0
        move_thread.join()
    speed_scale = distance * speed_scale_factor

    # Get target velocity adjusted for how far away destination is
    # (i.e. larger distance corresponds to higher speed and vice versa)
    vel_left *= speed_scale
    vel_right *= speed_scale
    print("vel_left after distance scaling: ", vel_left)
    # Adjust for estimated depth ahead of robot, but give smaller weight
    # due to inaccuracy of depth estimation
    scaled_depth = (100 / middle_depth) * depth_scale_factor
    vel_left -= scaled_depth
    vel_right -= scaled_depth

    # Get target ticks for left and right wheels in interval
    target_left_ticks = velocity_to_ticks(vel_left)+15
    target_right_ticks = velocity_to_ticks(vel_right)
    print("target_vel_left: ", vel_left)
    print("target_vel_right: ", vel_right)
    print("target_left_ticks: ", target_left_ticks)
    print("target_right_ticks: ", target_right_ticks)


def depth_callback(point):
    global middle_depth
    middle_depth = point.x
    print("Depth received: ", middle_depth)


def dummy(point):
    print("In dummy: ", point.x)


def listener():
    rospy.init_node('control_motor_listener', anonymous=True)
    rospy.Subscriber("/distance_heading/distance_angle", Point, motor_callback)
    rospy.Subscriber("/control_robot/middle_depth", Point, depth_callback)
    rospy.spin()


if __name__ == "__main__":
    move_thread = threading.Thread(target=move)
    move_thread.start()
    listener()
    stop = True
    move_thread.join()
    #GPIO.output(ENR, GPIO.LOW)
    #GPIO.output(ENL, GPIO.LOW)
    #motorR.stop()
    #motorL.stop()
    #GPIO.cleanup()
