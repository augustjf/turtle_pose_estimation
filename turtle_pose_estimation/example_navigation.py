#! /usr/bin/env python3

'''
Create a class that can handle basic navigation task. 
The class will connect to the navigate_to_pose action server and will have method to enable the sent of navigation goal

'''

import time, sys

# Import of function necessary for the task.

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
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
        
        amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # Create the subscriver to 'amcl_pose' topic usign the QoSProfile specified
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
        # Create the publisher to 'initialpose' for the initialization
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose',10)
        self.initial_pose_received = False
        
        # definition of action client connected to 'navigate to pose' and 'navigate_through_poses'
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')


    def setInitialPose(self, initial_pose, covariance):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self.initial_pose_covariance = covariance
        self._setInitialPose()


    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")


        # Create a new action goal
        # Add the target pose and send the goal in async mode
        # Use the rclpy spin to wait until action server completion
        # Get the result at the end.
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)

        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        self.result_future = self.goal_handle.get_result_async()

        return True


    # def goThroughPoses(self, poses):
    #     # Sends a `NavToPose` action request and waits for completion
    #     # Using the same function create for goToPose, now send the command to the alternative action server
    #     self.debug("Waiting for 'NavigateToPose' action server")
    #     while not self.nav_through_poses_client.wait_for_server(timeout_sec=1.0):
    #         self.info("'NavigateToPose' action server not available, waiting...")
        
    #     goal_msg = NavigateThroughPoses.Goal()
    #     goal_msg.poses = poses

    #     send_goal_future = self.nav_through_poses_client.send_goal_async(goal_msg, self._feedbackCallback)
    #     rclpy.spin_until_future_complete(self, send_goal_future)
    #     self.goal_handle = send_goal_future.result()

    #     self.result_future = self.goal_handle.get_result_async()
    #     return True



    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            # Use goal_handle to cancel the goal 
            # Add spin_until_future_complete to wait for request completion
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isNavComplete(self):
        # Check if the task was cancelled or completed
        if not self.result_future:
            return True
        
        # use rclpy to spin until the furue is completed, add a timeout of 0.10 
        # after that check what is been obtained
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)

        # Now control if we have obtained a result
        if self.result_future.result():
            # Get the statu from the results and compare to GoalStatus.STATUS_SUCCEEDED
            # We can use GoalStatus to check the different state of the Action server goal.
            # To know which additional status it may have use (ctrl + left click) on the import to open where is been defined
            # Return True if the goal succeded correctly else False
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True

        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True


    # 'Get function' : used to return information outside the class to the user
    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        # Method used to call all the 'wait function'
        # When all are completed the system is ready to operate
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
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        # local copy of the feedback callback for future use
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose = self.initial_pose
        initial_pose.pose.covariance = self.initial_pose_covariance
        
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(initial_pose)
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
    initial_pose.position.x = 0.0
    initial_pose.position.y = 0.0
    initial_pose.position.z = 0.0
    initial_pose.orientation.x = 0.0
    initial_pose.orientation.y = 0.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 0.0
    initial_pose_covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                               0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, 1.0]

    print('Sending initial pose...')
    # call class setInitialPose with the specified initial pose
    navigator.setInitialPose(initial_pose, initial_pose_covariance)
    input('Pose ok ,enter to continue')

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    input('Navigation2 ok, enter to continue')


    # Go to the demo first goal pose
    # Initialize the message. You can do it manually or read it from a txt/yaml file
    # Specify the frame_id, stamp and position
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -2.0
    goal_pose.pose.position.y = -0.5
    goal_pose.pose.orientation.w = 1.0
    navigator.goToPose(goal_pose)

    
    # Call class function to move to target pose
    ...

    print('Navigator go to pose')
    i = 0
    while not navigator.isNavComplete():
        # While the robot is executing the task of navigate to pose,
        # We can executed other part of the script-
        # For example we can check for the sanification of the environment....

        # Get the feedback from the navigator and print it
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')            
            
            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelNav()

    print('goal concluded')
    # Do something depending on the return code
    result = navigator.getResult()
    if result == GoalStatus.STATUS_SUCCEEDED:
        print('Goal succeeded!')
    elif result == GoalStatus.STATUS_CANCELED:
        print('Goal was canceled!')
    elif result == GoalStatus.STATUS_ABORTED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    exit(0)

if __name__ == '__main__':
    main()