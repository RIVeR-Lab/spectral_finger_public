#!/usr/bin/env python3

# This line must be set to python 3.6 or the scripts will not work!
import time
import sys
import rospy
import typing
import traceback
from typing import List, Dict, Tuple
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from spectra_wiz.msg import Floats, Spectra, SpectraArray
from spectra_wiz.srv import RequestOnce, RequestCollect, RequestOnceResponse, RequestCollectResponse, RequestWrench, RequestWrenchResponse
from rospy.numpy_msg import numpy_msg
from spectral_finger_planner.srv import Grasp, GraspResponse
import numpy as np
import pandas as pd
import copy
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Conveyor spectrometer driver
# Author: Nathaniel Hanson
# Date: 06/30/21
# Purpose: ROS node to handle command line interface to save (blocking)
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '/path/to/application/app/folder')
class CliInterface():
    def __init__(self):
        # Initialize publishers
        self.pub_start = rospy.Publisher('/spectral_data/start', String, queue_size=10)
        self.pub_end = rospy.Publisher('/spectral_data/end', String, queue_size=10)
        self.pub_save = rospy.Publisher('/current_save', String, queue_size=10)
        self.wrench_start = rospy.Publisher('/wrench_data/start', String, queue_size=10)
        self.wrench_end = rospy.Publisher('/wrench_data/end', String, queue_size=10)
        # Subscribe to images from realsense
        self.bridge = CvBridge()
        self.cv_image = None
        # Setup connection to webcam
        self.cap = cv2.VideoCapture(2)
        # Setup connection to endoscope (finger cam)
        self.finger_cap = cv2.VideoCapture(0)
        # Initialize service
        rospy.wait_for_service('request_collect')
        self.grab_data = rospy.ServiceProxy('request_collect', RequestCollect)
        self.grasp = rospy.ServiceProxy('grasp', Grasp)
        self.grab_wrench = rospy.ServiceProxy('request_wrench_collect', RequestWrench)

    def run(self):
        while True:
            # Collect and publish data
            print('===========================')
            material = input('Please input a unique identifier for this material e.g. wool001:  ')
            if len(material) < 2:
                # Prevent short file names
                continue
            if input('Start collection? (y or n) ') == 'y':
                # Publish a command to start collection
                self.pub_start.publish(String(''))
            else:
                continue
            # Trigger collection service
            _, fullFrame = copy.deepcopy(self.cap.read())
            self.wrench_start.publish(String(''))
            self.grasp('pregrasp')
            # Capture picture of item in pregrasp position
            _, fingerFrame = copy.deepcopy(self.finger_cap.read())
            self.grasp('grasp')
            # Send robot to grasp and lift positions
            data = self.grab_data('')
            wrench_data = self.grab_wrench('')
            self.pub_end.publish(String(''))
            self.wrench_end.publish(String(''))
            # Return robot home
            self.grasp('release')
            # End data collection
            # format data in CSV format
            print(f'Collected {len(data.data)} spectral samples')
            print(f'Collected {len(wrench_data.wrench_data)} wrench samples')
            print(f'Collected {len(wrench_data.finger_data)} finger position samples')
            if input('Save data? (y or n) ') == 'y':
                self.spec_to_csv(data, material)
                self.wrench_to_csv(wrench_data, material)
                self.position_to_csv(wrench_data, material)
                self.save_images(fingerFrame, fullFrame, material)
            else:
                continue

    # Custom shutdown behavior
    def shutdown(self):
        pass
    
    def _convert_ros_time(self, data):
        return data.header.stamp.secs + data.header.stamp.nsecs * 1e-9

    def save_images(self, image_close, image_full, material):
        cv2.imwrite(f'/home/river/ros_cross/data_collection/img_close/{material}.png', image_close)
        cv2.imwrite(f'/home/river/ros_cross/data_collection/img_full/{material}.png', image_full)

    # Create and save csv file
    def spec_to_csv(self, data, material):
        columns = None
        startTime = None
        toSave = []
        for msg in data.data:
            # Create the columns
            if columns == None:
                tData = np.array(msg.data).reshape(msg.channels, msg.width)
                columns = list(tData[:,0])
                columns.insert(0, 'time')
                startTime = self._convert_ros_time(msg)
            # Insert data
            tData = np.array(msg.data).reshape(msg.channels, msg.width)
            temp = list(tData[:,1])
            elapsed_time = self._convert_ros_time(msg)
            temp.insert(0, elapsed_time)
            toSave.append(temp)
        try:
            df = pd.DataFrame(data = toSave, columns = columns)
            df.to_csv(f'/home/river/ros_cross/data_collection/spectral/{material}.csv')
        except Exception as e:
            print(f'Error saving data - please run again!')

    def shutdown():
        self.cap.release()

    # Create and save csv file
    def wrench_to_csv(self, data, material):
        columns = None
        toSave = []
        for msg in data.wrench_data:
            temp = np.array([self._convert_ros_time(msg), msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
            toSave.append(temp)
        try:
            df = pd.DataFrame(data = toSave, columns = ['time', 'x', 'y', 'z'])
            df.to_csv(f'/home/river/ros_cross/data_collection/torque/{material}.csv')
        except Exception as e:
            print(f'Error saving data - please run again!')

    # Create and save csv file
    def position_to_csv(self, data, material):
        columns = None
        toSave = []
        for msg in data.finger_data:
            temp = np.array([self._convert_ros_time(msg), float(msg.position[0]), float(msg.position[1])])
            toSave.append(temp)
        try:
            df = pd.DataFrame(data = toSave, columns = ['time', 'pos', 'force'])
            df.to_csv(f'/home/river/ros_cross/data_collection/finger_pos/{material}.csv')
        except Exception as e:
            print(f'Error saving data - please run again!')

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('cli_interface', anonymous=True)
    try:
        controller = CliInterface()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()
