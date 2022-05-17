#!/usr/bin/env python3

# This line must be set to python 3.6 or the scripts will not work!
import time
import sys
import rospy
import typing
import traceback
from typing import List, Dict, Tuple
from std_msgs.msg import String, Header
from spectra_wiz.msg import Floats, Spectra, SpectraArray
from rospy.numpy_msg import numpy_msg
from spectra_wiz.srv import RequestOnce, RequestCollect, RequestOnceResponse, RequestCollectResponse
import numpy as np
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

# Conveyor spectrometer driver
# Author: Nathaniel Hanson
# Date: 06/30/21
# Purpose: ROS collection tool for experimental setup

class CrossCollect():
    def __init__(self):
        # Initialize publishers
        self.pub_start = rospy.Publisher('/spectral_data/start', String, queue_size=10)
        self.pub_end = rospy.Publisher('/spectral_data/end', String, queue_size=10)
        # Subscribe to collected data topic
        rospy.Subscriber('/spectral_data/collected', SpectraArray, self.process_collect)
        rospy.Subscriber('/spectral_data', Spectra, self.process_single)
        self.pred = []
        self.setup_plot()
        plt.show()
        # Define shutdown behavior
        rospy.on_shutdown(self.shutdown)

    def process_single(self, data: Spectra):
        z = np.array(data.data)
        z = z.reshape(data.channels,data.width)
        self.xdata = z[:,0]
        self.ydata = z[:,1]

    def process_collect(self, data):
        # Recreate correct array
        pass

    # Custom shutdown behavior
    def shutdown(self):
        self.ani.event_source.stop()
        plt.close('all')

    def setup_plot(self):
        '''
        Initialize plot to visualize ball position in world 
        '''
        self.fig = plt.figure()
        self.ax = plt.axes(xlim=(300, 1200), ylim=(0, 20000))
        self.ax.set_xlabel('Wavelength (nm)')
        self.ax.set_ylabel('Counts')
        self.ax.set_title('Spectrometer Live View')
        self.line, = self.ax.plot([], [], lw=3)
        self.xdata, self.ydata = [], []
        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, blit=True)
        plt.show(block=False) 

    def init_plot(self):
        self.line.set_data(self.xdata, self.ydata)
        return self.line,

    def update_plot(self, i):
        '''
        Animate graph with current data
        '''
        self.line.set_data(self.xdata,self.ydata)
        return self.line,

# Main functionality
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Collection_Wrapper', anonymous=True)
    try:
        controller = CrossCollect()
        controller.run()
    except rospy.ROSInterruptException:
        pass
