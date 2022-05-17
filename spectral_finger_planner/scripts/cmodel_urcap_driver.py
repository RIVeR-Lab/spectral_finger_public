#!/usr/bin/env python3
import os
import sys
import socket
import rospy
from cmodel_urcap import RobotiqCModelURCap
# from ace_ppe_manipulation.msg import _Robotiq2FGripper_robot_input as inputMsg
# from ace_ppe_manipulation.msg import _Robotiq2FGripper_robot_output as outputMsg

def mainLoop(ur_address):
  # Gripper is a C-Model that is connected to a UR controller with the Robotiq URCap installed. 
  # Commands are published to port 63352 as ASCII strings.
  gripper = RobotiqCModelURCap(ur_address)
  # The Gripper status
  pub = rospy.Publisher('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, queue_size=3)
  # The Gripper command
  rospy.Subscriber('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, gripper.sendCommand)
  
  while not rospy.is_shutdown():
    # Get and publish the Gripper status
    status = gripper.getStatus()
    pub.publish(status)
    # Wait a little
    rospy.sleep(0.1)

if __name__ == '__main__':
  rospy.init_node('cmodel_urcap_driver')

  robot_ip = rospy.get_param('~robot_ip', '192.168.1.1')

  try:
    mainLoop(robot_ip)
  except rospy.ROSInterruptException: pass