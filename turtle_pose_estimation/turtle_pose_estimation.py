#! /usr/bin/env python3

import time, sys, math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.initial_pose = Pose()
        self.initial_pose_covariance = []
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.amcl_pose = PoseWithCovarianceStamped()
        self.declare_parameter('estimated_pose_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('estimated_pose_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('estimated_pose_angle', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('variance_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('variance_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('variance_angle', rclpy.Parameter.Type.DOUBLE)
        
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos) 
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose',10)
        self.initial_pose_received = False

    def setInitialPose(self, initial_pose, covariance):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self.initial_pose_covariance = covariance
        self._setInitialPose()


    def euler_to_quaternion(self, roll, pitch, yaw):

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]


    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        self.info('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.info('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.info('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.amcl_pose = msg
        self.get_logger().info('recieved message on /amcl_pose')
        self.initial_pose_received = True
        return

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose = self.initial_pose
        msg.pose.covariance = self.initial_pose_covariance
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return 
    
    def info(self, msg):
        self.get_logger().info(msg)
        return

def main(argv=sys.argv[1:]):
    rclpy.init()
    navigator = BasicNavigator()

    initial_pose = Pose()
    initial_pose.position.x = navigator.get_parameter('estimated_pose_x').get_parameter_value().double_value
    initial_pose.position.y = navigator.get_parameter('estimated_pose_y').get_parameter_value().double_value
    initial_pose.position.z = 0.0
    x, y, z, w = navigator.euler_to_quaternion(0.0, 0.0, navigator.get_parameter('estimated_pose_angle').get_parameter_value().double_value)
    initial_pose.orientation.x = x
    initial_pose.orientation.y = y
    initial_pose.orientation.z = z
    initial_pose.orientation.w = w
    var_x = navigator.get_parameter('variance_x').get_parameter_value().double_value
    var_y = navigator.get_parameter('variance_y').get_parameter_value().double_value
    var_angle = navigator.get_parameter('variance_angle').get_parameter_value().double_value
    initial_pose_covariance = [var_x, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, var_y, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, var_angle]

    print('Sending initial pose...')
    navigator.setInitialPose(initial_pose, initial_pose_covariance)
    navigator.waitUntilNav2Active()

    exit(0)

if __name__ == '__main__':
    main()