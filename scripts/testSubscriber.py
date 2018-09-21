#!/usr/bin/env python
import rospy, time
import rospy
from dynamixel_msgs.msg import JointState

claw_upperLeft_offset = 0.0
claw_upperRight_offset = 0.0


def callback(data, OffsetName):

    global offsetName = data.goal_pos


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('multiturnOffsets', anonymous=True)
    rospy.Subscriber('/claw_upperLeft_controller/state', JointState, callback)
    rospy.Subscriber('/claw_upperRight_controller/state', JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
