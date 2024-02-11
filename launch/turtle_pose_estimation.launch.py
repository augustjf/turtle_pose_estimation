import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pose_x_val = LaunchConfiguration('x_pose', default='0.0')
    pose_y_val = LaunchConfiguration('y_pose', default='0.0')
    pose_ang_val = LaunchConfiguration('angle_pose', default='0.0')
    var_x_val = LaunchConfiguration('x_var', default='1.0')
    var_y_val = LaunchConfiguration('y_var', default='1.0')
    var_ang_val = LaunchConfiguration('angle_var', default='0.5')

    pose_estimation = launch_ros.actions.Node(
        package=    'turtle_pose_estimation',
        executable= 'turtle_pose_estimation',
        name='pose_estimation',
        parameters=[
            {'estimated_pose_x':pose_x_val},
            {'estimated_pose_y':pose_y_val},
            {'estimated_pose_angle':pose_ang_val},
            {'variance_x':var_x_val},
            {'variance_y':var_y_val},
            {'variance_angle':var_ang_val},
        ]
    )


    return launch.LaunchDescription([
        pose_estimation
    ])