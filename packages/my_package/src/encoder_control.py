#!/usr/bin/env python


# Imports
import os
import rospy
from duckietown import DTROS
# Messages imports
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header, String, Int32, Float32
from my_package.msg import encoderTicksStamped

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
	
        # Construct publisher & subscriber
        #self.pub = rospy.Publisher('islduckie44/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.pub = rospy.Publisher('wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.sub_encoders = rospy.Subscriber("encoder_ticks", encoderTicksStamped, self.cbUpdateTicks)
        self.sub_wheels = rospy.Subscriber("wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, self.cbUpdateWheels)
	

	# DECLARE CLASS VARIABLES ----------------------------------------------------------

	# Encoder ticks variables
	# Current encoder ticks
	self.curr_left_encoder_ticks = 0
	self.curr_right_encoder_ticks = 0
	# Last encoder tick measurement
	self.last_left_encoder_ticks = 0 
	self.last_right_encoder_ticks = 0
	# Forward/backward number of ticks (includes additions/subtractions) 
	self.left_encoder_ticks_traversed = 0	
	self.right_encoder_ticks_traversed = 0
	self.last_encoder_time = rospy.get_time() 
	self.curr_encoder_time = rospy.get_time()
	self.ticks_per_revolution = 125 # Approximate placeholder value - waiting from DT org for real value

	# Wheel Speed Variables
	self.PI = 3.1415926535897 # Const
	self.wheel_diameter = 66 # [mm] - Approximate placeholder value - Const
	# Wheel speed as deg / sec --> can convert to rad/sec if desired
	# Angular speed (as different from vel_left and vel_right) is determined according to the following equation:
		# 			change ticks (L or R) 
		# omega (L or R) =  ---------------------------
		#			change time
	# To be calculated during operation
	# Associated with the rpm angular speed, not pwm wheel command speed
	self.left_wheel_speed_ang  = 0 # [deg/sec]
	self.right_wheel_speed_ang = 0 
	self.left_wheel_speed_lin  = 0 # [mm/sec] Instantaneous
	self.right_wheel_speed_lin = 0
	self.wheel_circumference = self.PI * self.wheel_diameter
	# Can be calculated from left/right encoder ticks related to distance via wheel circumference
	self.left_distance_traversed  = 0 # [mm]
	self.right_distance_traversed = 0 # [mm]
	
	# Wheel direction of rotation
	# Used to calculate how to handle the new encoder ticks
	# Assume wheels are initially driving forward - associated with positive wheel velocities
	self.left_wheel_forward  = True
	self.right_wheel_forward = True

	# END DECLARE CLASS VARIABLES ----------------------------------------------------------

	# Define Control Parameters
	# Note: you may wish to include any missing parameters as required for different control paradigms
	# Please decide on either velocity or position control as appropriate
	# The PID control paradigm can be implemented in the self.control function.
	#
	# Control gains (PID)
	# P - proportional
	# I - integral
	# D - derivative
	self.Kp = 0
	self.Ki = 0
	self.Kd = 0


	# End of init function
	rospy.loginfo("%s has started!" % node_name) # Diagnostic




    def cbUpdateTicks(self, msg):
	# Updates internal variables including:
	#	encoder ticks
	#	encoder time
	#	angular and linear velocity
	# ^ Differences of the above variables
	#
	#	angular and linear velocity
	#

        #rospy.loginfo("I heard something:" ) # Diagnostic
	#rospy.loginfo("Sub: Left ticks: %d \t Right ticks: %d" % (msg.left_ticks, msg.right_ticks)) # Diagnostic

	# Set current values --> old values
	self.last_left_encoder_ticks = self.curr_left_encoder_ticks
	self.last_right_encoder_ticks = self.curr_right_encoder_ticks
	self.last_encoder_time = self.curr_encoder_time

	# Update current values from message
	self.curr_left_encoder_ticks = msg.left_ticks
	self.curr_right_encoder_ticks = msg.right_ticks
	self.curr_encoder_time = msg.header.stamp

	# Update number of encoder ticks traversed
	self.calculateTraversed()

	# Update angular rate
	# [deg / sec] 
	#rospy.loginfo("Curr encoder time type:"); rospy.loginfo(type(self.curr_encoder_time)) # Diagnostic
	#rospy.loginfo("Last encoder time type:"); rospy.loginfo(type(self.last_encoder_time)) # Diagnostic
	self.time_since_last_update = (self.curr_encoder_time - self.last_encoder_time)
	#rospy.loginfo("Time since last update"); rospy.loginfo(type(self.time_since_last_update)) # Diagnostic
	#rospy.loginfo("Time diff"); rospy.loginfo(self.time_since_last_update) # Diagnostic
	comp_time = float(self.time_since_last_update.secs + (self.time_since_last_update.nsecs * 1.0e-9))
	#if (self.time_since_last_update.secs !=0): comp_time = float(self.time_since_last_update.secs + (self.time_since_last_update.nsecs * 1.0e-9))
	#else: comp_time = float(self.time_since_last_update.nsecs * 1.0e-9) # Diagnostic
	#rospy.loginfo("Comp time"); rospy.loginfo(comp_time)
	self.left_wheel_speed_ang  = (float(self.curr_left_encoder_ticks  - self.last_left_encoder_ticks)/  comp_time) * (1.0/self.ticks_per_revolution)
	self.right_wheel_speed_ang = (float(self.curr_right_encoder_ticks - self.last_right_encoder_ticks)/ comp_time) * (1.0/self.ticks_per_revolution)

	#rospy.loginfo("v_L_ang %f \t v_R_ang %f" %(self.left_wheel_speed_ang, self.right_wheel_speed_ang)) # Diagnostic

	# Update instantaneous linear wheel rate
	# [mm/sec]
	self.left_wheel_speed_lin  = self.left_wheel_speed_ang  * (self.wheel_circumference / 360) # [mm/s]=[deg/s]*[dist/360deg]
	self.right_wheel_speed_lin = self.right_wheel_speed_ang * (self.wheel_circumference / 360) 

	#rospy.loginfo("v_L_lin %f \t v_R_lin %f" %(self.left_wheel_speed_lin, self.right_wheel_speed_lin)) # Diagnostic


    def cbUpdateWheels(self, msg):
	if(msg.vel_left > 0): 	self.left_wheel_forward = True
	elif(msg.vel_left < 0): self.left_wheel_forward = False
	# else --> no change
	if(msg.vel_right > 0): 	self.right_wheel_forward = True
	elif(msg.vel_right < 0): self.right_wheel_forward = False



	
    def calculateTraversed(self):
	# Update the encoder_ticks_traversed and the wheel-distance traversed

#	# Left
#	if(self.left_wheel_forward == True):
#            self.left_encoder_ticks_traversed += (self.curr_left_encoder_ticks - self.last_left_encoder_ticks)
#	elif(self.left_wheel_forward == False):
#            self.left_encoder_ticks_traversed -= (self.curr_left_encoder_ticks - self.last_left_encoder_ticks)
#	
#	# Right
#	if(self.right_wheel_forward == True):
#            self.right_encoder_ticks_traversed += (self.curr_right_encoder_ticks - self.last_right_encoder_ticks)
#	elif(self.right_wheel_forward == False):
#            self.right_encoder_ticks_traversed -= (self.curr_right_encoder_ticks - self.last_right_encoder_ticks)


	# More succinct way to represent the above conditional statements:
	# If wheel_forward == False, statement gets multiplied by -1 and subtracts difference. 
	# If wheel_forward == true, (-1)**0 --> 1 and adds the difference
	self.left_encoder_ticks_traversed  += ((-1)**(not self.left_wheel_forward))  * (self.curr_left_encoder_ticks  - self.last_left_encoder_ticks)
	self.right_encoder_ticks_traversed += ((-1)**(not self.right_wheel_forward)) * (self.curr_right_encoder_ticks - self.last_right_encoder_ticks)

	# Update the left/right wheel distance travelled
	self.left_distance_traversed  += ((-1)**(not self.left_wheel_forward))  * (self.curr_left_encoder_ticks  - self.last_left_encoder_ticks) * (self.wheel_circumference /self.ticks_per_revolution)
	self.right_distance_traversed += ((-1)**(not self.right_wheel_forward)) * (self.curr_right_encoder_ticks - self.last_right_encoder_ticks) * (self.wheel_circumference / self.ticks_per_revolution)




    def control(self):
	# YOUR CONTROL CODE HERE
	print("Add code here") # Nothing is calling this function yet.


	
    def run(self):
	# YOUR PROGRAM FLOW HERE
	#
	# Define your program flow here
	rate = rospy.Rate(1) # Hz -- Adjust this as appropriate

	# Pseudo code: If you wish to run once
	# while(condition):
	#	write objective in terms of encoder ticks (position) or rate (linear / angular)
	#	Construct a ROS message that achieves your objective, based on the template runDefault function below
	#	Monitor control inputs via the control function
	#
	#
	# If you have multiple conditions, I recommend multiple while loops.
	# 	E.g. 	1. Drive straight for some speed or distance(ticks)
	#		2. Rotate by X degrees
	#			Keeping in mind changes in X-Y displacement while doing so
	#		3. Drive ...
	#
	# If you wish to run continuously, wrap the contents of the function in...
	#	while not rospy.is_shutdown():
			
		

	
	


    def runDefault(self):
	# This is the original default run function that writes 100% wheel speed 
	# for both motors for 30 seconds then shuts down by publishing wheel
	# command velocity (cmd_vel) messages at 1 Hz.

        # publish a message corresponding to the rate
        rate = rospy.Rate(20) # Hz

	now = rospy.get_rostime()
	secs = now.secs
	wait_time = 10


	# Run for the number of secs defined above
	while((rospy.get_rostime().secs - secs) < wait_time):
	    # Construct the wheel command message
            msg = WheelsCmdStamped()
		# WheelsCmdStamped has the format:
		#
		#	Header: seq, stamp (time), frame_id
		#	int32 vel_left
		#	int32 vel_right
		#
		# Header info can be accessed via header.___ --> e.g. header.seq
            msg.vel_left = 1 	# These values can be between -1 and 1.
            msg.vel_right = 1
            msg.header.stamp = rospy.get_rostime() # It's important to add the message time so the rotation rate can be calculated.

            # Diagnostic
            #rospy.loginfo("Publishing message: \n'%s'" % msg) # Diagnostic to show message
	    rospy.loginfo("Current loop time %f" % (rospy.get_rostime().secs - secs)) # Diagnostic to see timing. Onlys runs at rate frequency specified above

            # Publish the message
            self.pub.publish(msg)
            rate.sleep() # Synchronize the running rate for the loop


	# Outside the while loop - initiate shutdown
	rospy.loginfo("Shutting down after %d seconds" % wait_time)
	msg = WheelsCmdStamped()
       	msg.vel_left = 0
       	msg.vel_right = 0
       	msg.header.stamp = rospy.get_rostime()
       	#rospy.loginfo("Publishing message: \n'%s'" % msg) # Diagnostic
        self.pub.publish(msg) # This shuts off the wheels at the end of the script
        #rate.sleep() # No. - about to exit
	return 0


if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='encoder_control_node')

    #node.run() ## UNCOMMENT WHEN READY TO TEST

    # run node
    node.runDefault() # Can be used for an example

    # keep spinning
    #rospy.spin() # This will continue to run the node after the commands have finished


