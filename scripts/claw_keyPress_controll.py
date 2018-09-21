#!/usr/bin/env python

# The line above is found in all Python ROS nodes:
# ensures that the script is executed as a python script.

# Useful website:
# https://pythonhosted.org/pynput/keyboard.html

# rospy Publisher info & example:
# https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

# Make sure that dynamixel motors are in wheel mode for belt-gripper.

from pynput.keyboard import Key, Listener, Controller

import rospy, time
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState
import pprint




#############
# ROS STUFF #
#############


class ClawMotor:
    def __init__(self, name, sign):
        self.name = name
        self.load = 0.0             # float value representing calculated load from I-draw.
        self.overloadValue = 0.3    # Motor must not exceed this load-value, or will raise overload error exceprion and become unresponseive until power is cycled.
        self.sign = sign            # used to define the rotation-orientation to close claw.
        self.initSubscriber()       # Initializes subscription to motor's load value.
        self.initPublisher()        # Initializes publication to motor's joint angle (goal_pos).

        self.jointValue = 0.0       # Value used to control joint position.
        self.jointIncrement = 0.1

    def initSubscriber(self):
        rospy.Subscriber('/' + self.name + "_controller/state", JointState, self.loadValue_callBack)

    def loadValue_callBack(self, msg):
        self.load = msg.load
        #rospy.loginfo(rospy.get_caller_id() + " load value: %f", self.load)
    
    def initPublisher(self):
        self.pub = rospy.Publisher('/' + self.name + '_controller/command', Float64, queue_size=1)  # Creates a new publisher for every claw-motor instance.
        rospy.init_node('claw_commander', anonymous=True)                                           # Initialize a node for the publisher.
        self.pub.publish(0.0)                                                                       # Initialize motors to their home position.

    def isOverload(self):
        #return (True if (self.load >= self.overloadValue) or (self.load <= -1*self.overloadValue) else False)
        
        if self.load >= self.overloadValue or self.load <= -self.overloadValue:
            # In overloaded state: get outta there!
            return True

        return False



    # returns true if joint-value is within the premitted joint range.
    def inRange(self):
        return (True if (self.jointValue > 0.0 and self.jointValue < 3.7) else False)




    def openClaw(self, openValue = 0.1):

        if self.jointValue < 3.7:   
            self.jointValue += self.sign * openValue



    def closeClaw(self, openValue = 0.1):

        if self.jointValue > openValue: 
            self.jointValue -= self.sign * openValue
        
        # Default to 0.0 if below joint value.
        if self.jointValue < openValue:
            self.jointValue = 0.0



    # Method that makes motor attempt to avoid overload conditions.
    def control_motor(self, key):

        
        if key == Key.up:
            self.closeClaw(0.05)
        if key == Key.down:
            self.openClaw(0.05)

        self.pub.publish(self.jointValue)




#######################
# INSTANTIATE OBJECTS #
#######################

# Initializing gripper ClawMotor objects.
claw_motor_71 = ClawMotor('claw_motor_71', 1)
claw_motor_72 = ClawMotor('claw_motor_72', 1)
claw_motor_73 = ClawMotor('claw_motor_73', 1)




###################
# KEYLOGGER STUFF #
###################

keyboard = Controller()


def on_press(key):

    if key == Key.esc:

        #show final counter value.
        #print('\n\n Final count_fwd value: ' + str(counter_fwd) + '\n')

        #Open gripper
        claw_motor_71.pub.publish(0.0)
        claw_motor_72.pub.publish(0.0)
        claw_motor_73.pub.publish(0.0)

        #Make sure motor isn't overloaded when in resting position (caused by gear backlash)
        # claw_motor_71.control_motor(key)
        # claw_motor_72.control_motor(key)
        # claw_motor_73.control_motor(key)

        #shutdown node
        rospy.is_shutdown()
        # Stop listener
        return False



    claw_motor_71.control_motor(key)
    claw_motor_72.control_motor(key)
    claw_motor_73.control_motor(key)



    time.sleep(0.001)

    return True

# Collect events until released
with Listener(on_press=on_press) as listener:
    listener.join()