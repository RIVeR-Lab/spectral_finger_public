#!/usr/bin/python3.8

import rospy
import typing
import traceback
import PyKDL as kdl
import kdl_parser_py.urdf as kdl_parser
import actionlib
from std_msgs.msg import Float64, String, Header
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from point_cloud_proc.srv import TabletopClustering, TabletopClusteringRequest
from cmodel_urcap import RobotiqCModelURCap
from spectral_finger_planner.srv import Move, MoveResponse
import numpy as np

class UR3EPlanner:
    def __init__(self):
        self.base_link = 'base_link'
        self.ee_link = 'tool0'

        flag, self.tree = kdl_parser.treeFromParam('/robot_description')
        self.chain = self.tree.getChain(self.base_link, self.ee_link)
        self.num_joints = self.tree.getNrOfJoints()
        self.pos_ik_solver = kdl.ChainIkSolverPos_LMA(self.chain)
        self.pos_fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.close_notifier = rospy.Publisher('/joint_done', String, queue_size=10)
        self.move_service = rospy.Service('ur3e_move_point', Move, self.go_point)
        # Connect gripper to the robot
        robot_ip = rospy.get_param('/ur_hardware_interface/robot_ip')
        self.gripper = RobotiqCModelURCap(address=robot_ip)
        rospy.loginfo('Connected to UR at {}.'.format(robot_ip))
        # TODO: this doesn't work, still requires manual activation
        if not self.gripper.is_active():
            rospy.loginfo('Activating gripper...')
            self.gripper.activate(auto_calibrate=False)


        self.home_joint_angles = [-3.20, -1.41, 0.83, -0.96, -1.56, 0.0]
        self.fixed_eef_rot = [3.14, 0, 1.57]
        self.fixed_place_eef_pos = [-0.22, -0.40, 0.20]
        self.arm_joints = kdl.JntArrayVel(self.num_joints)
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        rospy.init_node('ur3e_planner')
        rospy.Subscriber('/joint_states', JointState, self.arm_joint_state_cb)
        self.speed_scaling_pub = rospy.Publisher('/speed_scaling_factor', Float64, queue_size=10)
        self.arm_pos_cli = actionlib.SimpleActionClient(
            '/scaled_pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        self.clustering_cli = rospy.ServiceProxy('/object_clustering/cluster_objects', TabletopClustering)
        self.arm_pos_cli.wait_for_server()
        self.clustering_cli.wait_for_service()

    def arm_joint_state_cb(self, msg):
        self.arm_joints.q[0] = msg.position[2]
        self.arm_joints.q[1] = msg.position[1]
        self.arm_joints.q[2] = msg.position[0]
        self.arm_joints.q[3] = msg.position[3]
        self.arm_joints.q[4] = msg.position[4]
        self.arm_joints.q[5] = msg.position[5]

        self.arm_joints.qdot[0] = msg.velocity[2]
        self.arm_joints.qdot[1] = msg.velocity[1]
        self.arm_joints.qdot[2] = msg.velocity[0]
        self.arm_joints.qdot[3] = msg.velocity[3]
        self.arm_joints.qdot[4] = msg.velocity[4]
        self.arm_joints.qdot[5] = msg.velocity[5]

    def ik(self, eef_pos, eef_rot):
        print(eef_pos)
        eef_pose = kdl.Frame(
            kdl.Rotation.RPY(*eef_rot),
            kdl.Vector(*eef_pos)
        )
        q_init = self.arm_joints.q
        q_sol = kdl.JntArray(self.num_joints)
        result = self.pos_ik_solver.CartToJnt(q_init, eef_pose, q_sol)
        rospy.loginfo('ik solver result: {}'.format(result))
        return list(q_sol)

    def fk(self):
        eef_pose = kdl.Frame()
        self.pos_fk_solver.JntToCart(self.arm_joints.q, eef_pose)
        return eef_pose

    def send_arm_traj(self, q, time=2):
        traj_goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        traj_point = JointTrajectoryPoint()
        traj_point.positions = q
        traj_point.velocities = [0.0] * self.num_joints
        traj_point.time_from_start = rospy.Time(time)

        traj.points = [traj_point]
        traj_goal.trajectory = traj
        
        self.arm_pos_cli.send_goal(traj_goal)
        self.arm_pos_cli.wait_for_result()

    def plan_grasp(self):
        req = TabletopClusteringRequest()
        res = self.clustering_cli.call(req)
        if not res.success:
            return False
        object_pos = res.objects[0].pose.position
        grasp_pos = np.zeros(3)
        grasp_pos[0] = object_pos.x
        grasp_pos[1] = object_pos.y
        grasp_pos[2] = object_pos.z + 0.15
        grasp_rot = self.fixed_eef_rot
        return grasp_pos, grasp_rot

    def grasp(self):
        grasp_pose = self.plan_grasp()
        if not grasp_pose:
            return False
        else:
            grasp_pos, grasp_rot = grasp_pose[0], grasp_pose[1]
        grasp_pos[2] += 0.10
        q = self.ik(grasp_pos, grasp_rot)
        self.send_arm_traj(q)
        grasp_pos[2] -= 0.10
        q = self.ik(grasp_pos, grasp_rot)
        self.send_arm_traj(q)
        self.close_gripper()
        rospy.sleep(0.5)
        grasp_pos[2] += 0.15
        q = self.ik(grasp_pos, grasp_rot)
        self.send_arm_traj(q)
        return True

    def close_gripper(self):
        self.gripper.move_and_wait_for_pos(255, 100, 5)

    def open_gripper(self):
        self.gripper.move_and_wait_for_pos(10, 100, 5)

    def go_point(self, msg):
        '''
        Move the arm to the commanded pose

        RR
        '''
        toSend = MoveResponse()
        print(toSend)
        try:
            print(msg)
            
            grasp_pos = np.zeros(3)
            grasp_pos[0] = msg.request.x
            grasp_pos[1] = msg.request.y
            grasp_pos[2] = msg.request.z
            grasp_rot = self.fixed_eef_rot
            q = self.ik(grasp_pos, grasp_rot)
            self.send_arm_traj(q)
            toSend.response = True
            return toSend
        except Exception as e:
            traceback.print_exc()
            toSend.response = False
            return toSend

    def go_home(self, msg):
        planner.send_arm_traj(planner.home_joint_angles)
        return True


if __name__ == '__main__':
    planner = UR3EPlanner()
    planner.open_gripper()
    run  = True
    while run:
        planner.send_arm_traj(planner.home_joint_angles)
        run = planner.grasp()
        q = planner.ik(planner.fixed_place_eef_pos, planner.fixed_eef_rot)
        planner.send_arm_traj(q)
        planner.open_gripper()