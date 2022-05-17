#!/usr/bin/python3.8

import time
import sys
import rospy
import typing
import traceback
from typing import List, Dict, Tuple
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
from spectra_wiz.srv import RequestWrench, RequestWrenchResponse
from geometry_msgs.msg import WrenchStamped
from rospy.numpy_msg import numpy_msg
import numpy as np
import copy
# Author: Nathaniel Hanson
# Date: 06/30/21
# Purpose: Listen and record to the wrench and finger data coming from the UR3e

class WrenchListener():
    def __init__(self):
        # Initialize empty store to aggregate spectral readings
        self.wrench_store = []
        self.finger_store = []
        self.collection_on = False
        self.msg_count = 0
        # Initialize Services
        self.service_collect = rospy.Service('request_wrench_collect', RequestWrench, self.send_current_collection)
        # Subscribe to start/stop topics
        rospy.Subscriber('/wrench_data/start', String, self.start_collect)
        rospy.Subscriber('/wrench_data/end', String, self.end_collect)
        # Subscribed to rate limited wrench topic
        rospy.Subscriber('/wrench_limited', WrenchStamped, self.get_wrench_callback)
        rospy.Subscriber('/gripper_position', JointState, self.get_finger_callback)


    def send_current_collection(self, _) -> RequestWrenchResponse:
        toSend = RequestWrenchResponse()
        toSend.wrench_data = self.wrench_store
        toSend.finger_data = self.finger_store
        return toSend

    def start_collect(self, _):
        self.wrench_store = []
        self.finger_store = []
        self.msg_count = 0
        self.collection_on = True
        rospy.loginfo('Starting Wrench Collection!')

    def end_collect(self, _):
        self.wrench_store = []
        self.finger_store = []
        self.msg_count = 0
        self.collection_on = False
        rospy.loginfo('Ending Wrench Collection!')

    def get_wrench_callback(self, msg):
        self.wrench_store.append(msg)
        
    def get_finger_callback(self, msg):
        self.finger_store.append(msg)

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('wrench_listener', anonymous=True)
    controller = WrenchListener()
    rospy.spin()
        
