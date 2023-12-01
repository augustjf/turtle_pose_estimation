#! /usr/bin/env python3

import time, sys

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from tf2_ros import Duration


import rclpy
from rclpy.action import ActionClient
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


    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return
    
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
        # Waits for the node within the tester namespace to become active
        # This code is usually executed at the beninning of the script
        # it is used to control that all the nodes are initiallized correctly 
        # to avoid the code to fail during the execution
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        # Se the initial pose
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        # Save if the initial pose is been received
        self.amcl_pose = msg
        self.get_logger().info('recieved message on /amcl_pose')
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        # local copy of the feedback callback for future use
        self.feedback = msg.feedback
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

    # Logger Functions:
    # They are used to write string to the logger in a more easier way.
    # Message print using logger are saved for multiple purpose (e.x debuggin in case of LARGE projects)
    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def main(argv=sys.argv[1:]):
    rclpy.init()
    # initialize the class of basic navigator
    navigator = BasicNavigator()

    # Set initial pose of the robot with respect to map reference frame!
    # !! In the PROJECT !! 
    # The localization should be performed AUTONOUSLY by the robot
    # HINT: The message PoseWithCovarianceStamped contain a covariance that indicate the confidence of the localization.

    initial_pose = Pose()
    initial_pose.position.x = -1.0
    initial_pose.position.y = 3.0
    initial_pose.position.z = 0.0
    initial_pose.orientation.x = -2.0
    initial_pose.orientation.y = -1.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0
    initial_pose_covariance = [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 2.0]

    print('Sending initial pose...')
    navigator.setInitialPose(initial_pose, initial_pose_covariance)
    navigator.waitUntilNav2Active()

    exit(0)

if __name__ == '__main__':
    main()