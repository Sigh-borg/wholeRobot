#!/usr/bin/env python
# This specifies exactly which python interpreter will be used to run the file

#This script subscribes to joystick publisher and then creates 2 publishers. 1 
# for notDT and 1 for wheels, notDT.cpp and listenerMotor.cpp are linked with this file

# Imports a pure Python client library for ROS
import rospy

from geometry_msgs.msg import Twist

# This expresses velocity in free space broken into its linear and angular parts.
from std_msgs.msg import Int8

# Reports the state of a joysticks axes and buttons. (Controller package)
from sensor_msgs.msg import Joy

# Class for NotDT inputs
class JoystickPublisher:
    def __init__(self, Publisher):
        self.pub = Publisher

    def callback(self, message):
        # CONFIRMED NOTDRIVETRAIN PUB
        
        # message.buttons/axes[int] == 1 just checks to see if it is being pressed
        if(message.buttons[6] == 1 and message.buttons[7] == 1 and message.axes[7] == 1.0):
                if(message.buttons[0] == 1):
                        Int8 = 3 #Deposit
                        self.pub.publish(Int8)
                elif(message.buttons[1] == 1):
                        Int8 = 2 #Dig
                        self.pub.publish(Int8)
                elif(message.buttons[3] == 1):
                        Int8 = 1 #DriveMode
                        self.pub.publish(Int8)
                elif(message.buttons[4] == 1):
                        Int8 = 4 #Zero
                        self.pub.publish(Int8)
                elif(message.buttons[12] == 1):
                        Int8 = 5 #config
                        self.pub.publish(Int8)
        if(message.axes[7] == -1):
                Int8 = 0 #kill any function running
                self.pub.publish(Int8)
        if(message.axes[6] == 1): # right or left dpad
                rospy.set_param("/manualMode", True)
                rospy.loginfo("Setting /manualMode to True ")
        if(message.axes[6] == -1): # right or left dpad
                rospy.set_param("/manualMode", False)
                rospy.loginfo("Setting /manualMode to false ")

        # a = message.buttons[0]
        # b = message.buttons[1]
        # x = message.buttons[3]
        # y = message.buttons[4]
        # D^ = message.axes[7]
        # RB = message.buttons[6]
        # LB = message.buttons[7]
        # menu = message.buttons[10]
        # start = message.buttons[11]
        # Xbox = message.buttons[12]

        # for now we are using temporary parameters to check for dig and deposit completion
        # once trencherClass is made into a ros class and can publish to robot_process, uncomment this:
        #if message.data == 26 and not rospy.get_param("/manualMode"):
        #        self.pub.publish(17)
        #if message.data == 27 and not rospy.get_param("/manualMode"):
        #        self.pub.publish(18)

        # checks to see if parameter that is set to true at end of dig and deposit function and if auto is on,
        # if so, it will send a nav goal for move_base. Be aware that right before param is changed, dig and 
        # deposit function automatically go back to driveMode before enabling ongoingDigPhase, so no need to worry about robot
        # killing itself by moving while it is deep inside arena regolith
        if rospy.get_param("/ongoingDigPhase") and not rospy.get_param("/manualMode"): # if dig function ended and auto is on
                rospy.set_param("/ongoingDigPhase", False)  # turn off "notification" that dig function ended
                rospy.loginfo("Publishing Int8 %d (nav to deposit) to robot_process topic: ", 17)
                self.pub.publish(17) # send 2d nav goal to deposit area

        if rospy.get_param("/ongoingDepositPhase") and not rospy.get_param("/manualMode"): # if deposit function ended and auto is on
                rospy.set_param("/ongoingDepositPhase", False) # turn off "notification" that deposit function ended
                rospy.loginfo("Publishing Int8 %d (nav to dig) to robot_process topic: ", 18)
                self.pub.publish(18) # send 2d nav goal to dig area

# Class for wheel inputs
class JoystickPublisherWheel:
    def __init__(self, Publisher):
        self.pub = Publisher

    def combineLTRT(self, message):
        ''' 
        LT is left trigger, message.axes[5] (from Joy package) 
        takes input from left trigger of controller from value -1 to 1.
        Without pressing down on the trigger, you get a default value of -1
        so you need to add 1.0 in the parenthesis to get a default value of 0
        when you are not pressing the trigger. Divide by 2 because 
        motorPercentOutput can accept a maximum value of 1. RT is right trigger.
        '''
        LT = -(message.axes[5] + 1.0) / 2
        RT = -(message.axes[4] + 1.0) / 2
        return (RT - LT)

        # Function that keeps getting called on by publisher
    def callbackWheel(self, message):
        # CONFIRMED DRIVETRAIN PUB

	# twist() now can take in linear and angular values set by twist
        twist = Twist()

	# Linear speed is controlled by triggers using combineLTRT function
        twist.linear.x = self.combineLTRT(message)

	# Angular speed is controlled by left joystick's horizontal axis (-1 to 1)
        twist.angular.z = message.axes[0]
        self.pub.publish(twist)


# Intializes everything
def start():
    # Name of node
    rospy.init_node('talker')

    # Publishes to robot_process topic using twist messages. The matching subscriber is notDT.cpp
    pub = rospy.Publisher('robot_process', Int8, queue_size=5)
    joystick = JoystickPublisher(pub)

    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, joystick.callback)

    # Publishes to chatter topic using twist messages. The matching subscriber is listenerMotor.cpp
    pubWheels = rospy.Publisher('manual_inputs', Twist, queue_size=5)
    joystickWheel = JoystickPublisherWheel(pubWheels)

# subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, joystickWheel.callbackWheel)

    # starts the node
    rospy.spin()

    
if __name__ == '__main__':
        #start the initialize controller script
       start()