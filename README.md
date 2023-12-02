# turtle_pose_estimation
Small node used to do initial localization in premade map. Sends estimated initial pose with variance. Terminates after localization is set.

## Launch
To launch 
```bash
ros2 launch turtle_pose_estimation turtle_pose_estimation.launch.py
```
Can be followed by launch arguments. Available arguments with default values are:

```
x_pose = 0.0
y_pose = 0.0
angle_pose = 0.0
x_var = 1.0
y_var = 1.0
angle_var = 0.5
```
giving the estimated valies for pose and the variances.

Example:
```bash
ros2 launch turtle_pose_estimation turtle_pose_estimation.launch.py x_pose:=2.0 y_pose:=-1.0 angle_var:=0.2
```


