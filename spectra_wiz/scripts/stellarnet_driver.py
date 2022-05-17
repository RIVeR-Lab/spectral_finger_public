#!/home/river/.pyenv/versions/3.6.7/bin/python

# This line must be set to python 3.6 or the script will not work!
import time
import sys
from xxlimited import new
import rospy
import typing
import traceback
from datetime import datetime
from typing import List, Dict, Tuple
from std_msgs.msg import String, Header
from spectra_wiz.msg import Floats, Spectra, SpectraArray
from rospy.numpy_msg import numpy_msg
from spectra_wiz.srv import RequestOnce, RequestCollect, RequestOnceResponse, RequestCollectResponse
import numpy as np
# import the manufacturer usb driver
import stellarnet_driver3 as sn
import multiprocessing as mp
import copy
# Conveyor spectrometer driver
# Author: Nathaniel Hanson
# Date: 06/30/21
# Purpose: ROS node VNIR sectrometer using manufacturer SDK

class SpectrometerDriver():
    def __init__(self):
        # Initialize empty store to aggregate spectral readings
        self.store = []
        self.collection_on = False
        self.msg_count = 0
        # Initialize spectrometer with parameters
        self._spectrometer, self._wav = sn.array_get_spec(0)  
        self._inttime = rospy.get_param('integration_time', 100)
        self._scansavg = rospy.get_param('scan_averaging', 1)
        self._smooth = rospy.get_param('smoothing_factor', 1)
        self.setup_spec()
        self.last = np.array([])
        # Initialize publishers
        self.pub = rospy.Publisher('/spectral_data', Spectra, queue_size=10)
        self.pub_collect = rospy.Publisher('/spectral_data/collected', SpectraArray, queue_size=10)
        # Initialize Services
        self.service_collect = rospy.Service('request_collect', RequestCollect, self.send_current_collection)
        self.service_once = rospy.Service('request_sample', RequestOnce, self.realtime_read)
        # Define shutdown behavior
        rospy.on_shutdown(self.shutdown)
        # Subscribe to start/stop topics
        rospy.Subscriber("/spectral_data/start", String, self.start_collect)
        rospy.Subscriber("/spectral_data/end", String, self.end_collect)
        rospy.Subscriber("/spectral_data/get_current_collection", String, self.get_collection)

    def realtime_read(self, _) -> RequestOnceResponse:
        data = self.last.copy()
        now = datetime.now() # current date and time
        fileName = now.strftime("%m_%d_%Y__%H_%M_%S")
        saveDir = '/home/river/block_collection/' + fileName + '.txt'
        np.savetxt(saveDir, data)
        print("Saved Data!")
        toSend = self.spectra_message_construction(data)
        return RequestOnceResponse(toSend)

    def send_current_collection(self, _) -> RequestCollectResponse:
        toSend = RequestCollectResponse()
        toSend.data = self.store
        toSend.count = len(self.store)
        return toSend

    def setup_spec(self):
        self._spectrometer['device'].set_config(int_time=self._inttime, scans_to_avg=self._scansavg, x_smooth=self._smooth)

    def start_collect(self, _):
        self.store = []
        self.msg_count = 0
        self.collection_on = True
        rospy.loginfo('Starting Spectrometer Collection!')

    def end_collect(self, _):
        toSend = SpectraArray()
        h = Header()
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        toSend.header = h
        toSend.data = self.store
        toSend.count = self.msg_count
        self.pub_collect.publish(toSend)
        self.store = []
        self.msg_count = 0
        self.collection_on = False
        rospy.loginfo('Ending Spectrometer Collection!')

    def get_collection(self, _):
        self.pub_collect.publish(self.store)

    def get_spectrum(self):
        try:
            rospy.logdebug('requesting spectrum')
            spectrum = sn.array_spectrum(self._spectrometer, self._wav)
            rospy.logdebug('recieved spectrum')
            return spectrum
        except Exception as e:
            rospy.logerr(f'Error retrieving spectrum: {str(e)}')
            rospy.logerr(f'Stack trace: {traceback.print_exc()}')
            return []
    
    def spectra_message_construction(self, newData: np.array) -> Spectra:
        toSend = Spectra()
        h = Header()
        d_shape = newData.shape
        data = newData.reshape(d_shape[0]*d_shape[1],1)
        h.stamp = rospy.Time.now() # Note you need to call rospy.init_node() before this will work
        toSend.header = h
        toSend.data = data
        toSend.channels = newData.shape[0]
        toSend.width = newData.shape[1]
        return toSend

    def run(self):
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            # Collect and publish data
            try:
                newData = self.get_spectrum()
                # Publish all the general data in spectra format
                if len(newData) > 0:
                    if self.collection_on == True:
                        self.store.append(self.spectra_message_construction(newData))
                        self.msg_count += 1
                    self.pub.publish(self.spectra_message_construction(newData))
                    self.last = newData
                r.sleep()
            except Exception as e:
                rospy.logerr(f'Error in main spectrometer loop: {str(e)}')

    # Custom shutdown behavior
    def shutdown(self):
        pass

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('spectrometer_driver', anonymous=True)
    try:
        controller = SpectrometerDriver()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()
