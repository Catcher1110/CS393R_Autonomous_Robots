#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('amrl_msgs')
from amrl_msgs.msg import AckermannCurvatureDriveMsg
import pynput
from pynput.keyboard import Key, Listener

pub = rospy.Publisher('ackermann_curvature_drive', AckermannCurvatureDriveMsg, queue_size=10) # "key" is the publisher name
rospy.init_node('keypress',anonymous=True)

msg = AckermannCurvatureDriveMsg()
velocity = 0;
curvature = 0;

VEL = 1
LEFT_CURV = 1
RIGHT_CURV = -1
def on_press(key):
    if hasattr(key, "char") and key.char == "w":
        msg.velocity = VEL
    elif hasattr(key, "char") and key.char == "s":
        msg.velocity = -1
    elif hasattr(key, "char") and key.char == "a":
        msg.curvature = LEFT_CURV
    elif hasattr(key, "char") and key.char == "d":
        msg.curvature = RIGHT_CURV
    return True

def on_release(key):
    if hasattr(key, "char") and key.char == "w":
        msg.velocity = 0
    elif hasattr(key, "char") and key.char == "a":
        msg.curvature = 0
    elif hasattr(key, "char") and key.char == "d":
        msg.curvature = 0
    elif hasattr(key, "char") and key.char == "s":
        msg.curvature = 0        
    if key == Key.esc:
        return False

if __name__ == "__main__":
    msg.velocity = 0
    msg.curvature = 0
    rate = rospy.Rate(20)

    listener = pynput.keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
