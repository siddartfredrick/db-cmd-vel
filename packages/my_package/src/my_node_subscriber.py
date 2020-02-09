#!/usr/bin/env python

# Example from https://docs.duckietown.org/daffy/duckietown-robotics-development/out/dt_infrastructure.html#sub:ros-sub-duckiebot

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        # construct publisher
        self.sub = rospy.Subscriber("chatter", String, self.callback)

    def callback(self, data):
        rospy.loginfo("I heard %s", data.data)

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node_subscriber')
    # keep spinning
    rospy.spin()
