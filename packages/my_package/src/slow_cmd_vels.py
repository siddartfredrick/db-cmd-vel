#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import String

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        # construct publisher
        self.pub = rospy.Publisher('islduckie44/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz

	now = rospy.get_rostime()
	secs = now.secs
	wait_time = 30

	while((rospy.get_rostime().secs - secs) < wait_time):
            msg = WheelsCmdStamped()
            msg.vel_left = 0.1
            msg.vel_right = 0.1
            msg.header.stamp = rospy.get_rostime()
            #rospy.loginfo("Publishing message: \n'%s'" % msg) # Diagnostic to show message
	    rospy.loginfo("Current loop time %f" % (rospy.get_rostime().secs - secs))
            self.pub.publish(msg)
            rate.sleep()

	# Outside the while loop
	rospy.loginfo("Shutting down after %d seconds" % wait_time)
	msg = WheelsCmdStamped()
       	msg.vel_left = 0
       	msg.vel_right = 0
       	msg.header.stamp = rospy.get_rostime()
       	#rospy.loginfo("Publishing message: \n'%s'" % msg) # Diagnostic
        self.pub.publish(msg) # This shuts off the wheels at the end of the script
        #rate.sleep() # No.
	return 0

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    #rospy.spin() # This will continue to run the node after the commands have finished
