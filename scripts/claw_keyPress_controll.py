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
    def __init__(self, name, sign, min_angle, max_angle):
        self.name = name
        self.load = 0.0             # float value representing calculated load from I-draw.
        self.overloadValue = 0.3    # Motor must not exceed this load-value, or will raise overload error exceprion and become unresponseive until power is cycled.
        self.sign = sign            # used to define the rotation-orientation to close claw.
        self.maxAngle = max_angle#4.2         # Radians.
        self.minAngle = min_angle#0.5          # --||--


        self.initSubscriber()       # Initializes subscription to motor's load value.
        self.initPublisher()        # Initializes publication to motor's joint angle (goal_pos).

        self.jointValue = 0.0       # Value used to control joint position.

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
        return (True if (self.jointValue > self.minValue and self.jointValue < self.maxAngle) else False)



    # Positive motor range: [minAngle, maxAngle]
    # Negative motor range: [maxAngle, 2*maxAngle - minAngle]

    def openClaw(self, openValue = 0.1):

        # if below, increment joint value.
        if (self.jointValue < self.maxAngle and self.sign == 1) or (self.jointValue < 2*self.maxAngle and self.sign == -1):
            self.jointValue += openValue

        # If max jointValue threshold exceeded, limit to max threshold.
        else:
            if self.sign == -1:
                self.jointValue = 2*self.maxAngle - self.minAngle
            else:
                self.jointValue = self.maxAngle


    def closeClaw(self, openValue = 0.1):

        # if above, decrement joint value.
        if (self.jointValue > self.minAngle and self.sign == 1) or (self.jointValue > self.maxAngle and self.sign == -1):
            self.jointValue -= openValue

        # if joint value less than lower threshold value, limit to lower threshold value.
        else:
            if self.sign == -1:
                self.jointValue = self.maxAngle
            else:
                self.jointValue = self.minAngle


    # Method that makes motor attempt to avoid overload conditions.
    def control_motor(self, key, mag = 1.0):


        if key == Key.up:

            if self.sign == 1:
                self.closeClaw(mag)
            else:
                self.openClaw(mag)

        elif key == Key.down:
            
            if self.sign == 1:
                self.openClaw(mag)
            else:
                self.closeClaw(mag)

        self.pub.publish(self.jointValue)

    # Method for stopping wheel-mode motors immediately.
    def wheelModeStop(self):
        self.jointValue = 0.0
        self.pub.publish(self.jointValue)


#######################
# INSTANTIATE OBJECTS #
#######################
                                                        # BUG: need to prevent joint-mode motor range from going < 0 (otherwise managers crashes).
# Initializing gripper ClawMotor objects.
claw_motor_71 = ClawMotor('claw_motor_71', 1, 0.5, 4.2)
claw_motor_72 = ClawMotor('claw_motor_72', -1, 0.5, 4.2)
claw_motor_73 = ClawMotor('claw_motor_73', 1, 0.5, 4.2)

belt_motor_61 = ClawMotor('belt_motor_61', 1, -5, 5)

claw_motor_71.jointValue = claw_motor_71.maxAngle
claw_motor_72.jointValue = claw_motor_72.maxAngle
claw_motor_73.jointValue = claw_motor_73.maxAngle

#Open gripper
claw_motor_71.pub.publish(claw_motor_71.jointValue)
claw_motor_72.pub.publish(claw_motor_72.jointValue)
claw_motor_73.pub.publish(claw_motor_73.jointValue)

###################
# KEYLOGGER STUFF #
###################

keyboard = Controller()


def on_press(key):

    if key == Key.esc:

        #show final counter value.
        #print('\n\n Final count_fwd value: ' + str(counter_fwd) + '\n')

        #Open gripper
        claw_motor_71.pub.publish(claw_motor_71.maxAngle)
        claw_motor_72.pub.publish(claw_motor_72.maxAngle)
        claw_motor_73.pub.publish(claw_motor_73.maxAngle)
        belt_motor_61.wheelModeStop()
        #Make sure motor isn't overloaded when in resting position (caused by gear backlash)
        # claw_motor_71.control_motor(key)
        # claw_motor_72.control_motor(key)
        # claw_motor_73.control_motor(key)

        #shutdown node
        rospy.is_shutdown()
        # Stop listener
        return False


    elif key == Key.space:
        belt_motor_61.wheelModeStop()   # Stops belt motor if space-bar is pressed.

    else:


        claw_motor_71.control_motor(key, 0.05)
        claw_motor_72.control_motor(key, 0.05)
        claw_motor_73.control_motor(key, 0.05)
        belt_motor_61.control_motor(key, 0.05)
        pprint.pprint(claw_motor_72.jointValue)

    #FOR DEBUG: #
    #rospy.loginfo(belt_motor_61.jointValue)

    time.sleep(0.001)
    return True

# Collect events until released
with Listener(on_press=on_press) as listener:
    listener.join()
