
# easy_handeye2: automated, hardware-independent Hand-Eye Calibration for ROS2

<img src="docs/img/eye_on_base_ndi_pic.png" width="345"/> <img src="docs/img/05_calibrated_rviz.png" width="475"/> 


This package provides functionality and a GUI to: 
- **sample** the robot position and tracking system output via `tf`,
- **compute** the eye-on-base or eye-in-hand calibration matrix through the OpenCV library's hand-eye calibration algorithms (e.g. Tsai-Lenz),
- **store** the result of the calibration,
- **publish** the result of the calibration procedure as a `tf` transform at each subsequent system startup,
- **evaluate** the accuracy of the resulting calibration matrix,
- (optional) automatically **move** a robot around a starting pose via `MoveIt!` to acquire the samples. 

The intended result is to make it easy and straightforward to perform the calibration, and to keep it up-to-date throughout the system. 
Two launch files are provided to be run, respectively to perform the calibration and check its result. 
A further launch file can be integrated into your own launch files, to make use of the result of the calibration in a transparent way: 
if the calibration is performed again, the updated result will be used without further action required.    

You can try out this software in a simulator, through the 
[easy_handeye2_demo package](https://github.com/marcoesposito1988/easy_handeye2_demo). This package also serves as an 
example for integrating `easy_handeye2` into your own launch scripts.

This is a port of [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) to ROS2.

## News
- version 0.5.0
    - port to ROS2
    - addition of rqt evaluator script
 

## Use Cases

If you are unfamiliar with Tsai's hand-eye calibration [1], it can be used in two ways:

- **eye-in-hand** to compute the static transform between the reference frames of
  a robot's hand effector and that of a tracking system, e.g. the optical frame
  of an RGB camera used to track AR markers. In this case, the camera is
  mounted on the end-effector, and you place the visual target so that it is
  fixed relative to the base of the robot; for example, you can place an AR marker on a table.
- **eye-on-base** to compute the static transform from a robot's base to a tracking system, e.g. the
  optical frame of a camera standing on a tripod next to the robot. In this case you can attach a marker,
  e.g. an AR marker, to the end-effector of the robot.
  
A relevant example of an eye-on-base calibration is finding the position of an RGBD camera with respect to a robot for object collision avoidance, e.g. [with MoveIt!](http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/perception_configuration.html): an [example launch file](docs/example_launch/ur5_kinect_calibration.launch) is provided to perform this common task between an Universal Robot and a Kinect through aruco. eye-on-hand can be used for [vision-guided tasks](https://youtu.be/nBTflbxYGkI?t=24s).

The (arguably) best part is, that you do not have to care about the placement of the auxiliary marker
(the one on the table in the eye-in-hand case, or on the robot in the eye-on-base case). The algorithm
will "erase" that transformation out, and only return the transformation you are interested in.


eye-on-base             |  eye-on-hand
:-------------------------:|:-------------------------:
![](docs/img/eye_on_base_aruco_pic.png)  |  ![](docs/img/eye_on_hand_aruco_pic.png)

## Getting started

- clone this repository into your ROS 2 workspace:
```
cd ~/easy_handeye2_ws/src  # replace with path to your workspace
git clone https://github.com/marcoesposito1988/easy_handeye2
```

- satisfy dependencies
```
cd ..  # now we are inside ~/easy_handeye2_ws
rosdep install -iyr --from-paths src
```

- build
```
colcon build
```

- source the workspace before running the launch files or CLI tools:
```
source install/setup.bash
```

## Usage

Two launch files, one for computing and one for publishing the calibration respectively,
are provided to be included in your own. The default arguments should be
overridden to specify the correct tf reference frames, and to avoid conflicts when using
multiple calibrations at once.

The suggested integration is:
- create a new `handeye_calibrate.launch.py` file, which includes the robot's and tracking system's launch files, as well as
  `easy_handeye2`'s `calibrate.launch.py`
- in each of your launch files where you need the result of the calibration, include
  `easy_handeye2`'s `publish.launch.py`

### Calibration

For both use cases, you can either launch the `calibrate.launch.py`
launch file, or you can include it in another launchfile as shown below. Either
way, the launch file starts `handeye_server` plus the RQT calibrator GUI. Samples are taken and managed from the GUI, and the resulting calibration can be saved to the package's calibration storage.

#### eye-in-hand

Run the calibrator directly:

```bash
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_in_hand \
  name:=my_eih_calib \
  robot_base_frame:=/base_link \
  robot_effector_frame:=/ee_link \
  tracking_base_frame:=/optical_origin \
  tracking_marker_frame:=/optical_target \
  move_group_namespace:=/ \
  move_group:=manipulator
```

#### eye-on-base

```bash
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_on_base \
  name:=my_eob_calib \
  robot_base_frame:=/base_link \
  robot_effector_frame:=/ee_link \
  tracking_base_frame:=/optical_origin \
  tracking_marker_frame:=/optical_target \
  move_group_namespace:=/ \
  move_group:=manipulator
```

If you want to include `easy_handeye2` from your own ROS 2 Python launch file, use `IncludeLaunchDescription` and pass the launch arguments there. For example:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("easy_handeye2"), "launch", "calibrate.launch.py"])
    ),
    launch_arguments={
        "calibration_type": "eye_in_hand",
        "name": "my_eih_calib",
        "robot_base_frame": "/base_link",
        "robot_effector_frame": "/ee_link",
        "tracking_base_frame": "/optical_origin",
        "tracking_marker_frame": "/optical_target",
        "move_group_namespace": "/",
        "move_group": "manipulator",
    }.items(),
)
```

If `move_group` is left empty, calibration samples contain only the TF transforms required for hand-eye solving. If `move_group` is set and can be resolved through MoveIt, `easy_handeye2` also records the current joint names and positions for that group into each `.samples` entry. This does not change the calibration math itself; it adds per-sample robot-state context that can be reused by downstream tooling.

#### Postprocess Multiple Sample Files

If you collected multiple `.samples` files for the same setup, you can recompute calibrations offline without rerunning the GUI.

Process each file independently with all supported OpenCV algorithms:

```bash
process_handeye_samples \
  sample_set_a.samples \
  sample_set_b.samples \
  --output-dir ./postprocessed
```

This writes one `.calib` file per input sample file and per algorithm, for example:

- `sample_set_a_tsai-lenz.calib`
- `sample_set_a_park.calib`
- `sample_set_b_horaud.calib`

If you want to merge multiple sample files into one larger sample set first, use `--merge`:

```bash
process_handeye_samples \
  sample_set_a.samples \
  sample_set_b.samples \
  sample_set_c.samples \
  --merge \
  --name merged_wrist_camera_calibration \
  --output-dir ./postprocessed
```

This produces one merged calibration set per algorithm, for example:

- `merged_wrist_camera_calibration_tsai-lenz.calib`
- `merged_wrist_camera_calibration_park.calib`
- `merged_wrist_camera_calibration_horaud.calib`

You can also restrict the algorithms explicitly:

```bash
process_handeye_samples \
  sample_set_a.samples \
  sample_set_b.samples \
  --merge \
  --algorithms Tsai-Lenz Park Horaud
```

If the `.samples` files do not contain complete calibration metadata, provide it on the command line:

```bash
process_handeye_samples \
  sample_set_a.samples \
  sample_set_b.samples \
  --merge \
  --calibration-type eye_in_hand \
  --robot-base-frame chassis \
  --robot-effector-frame green_gripper_frame_link \
  --tracking-base-frame left_arm_camera_optical \
  --tracking-marker-frame tag_42
```

`process_handeye_samples` is installed as a console entry point when the workspace is built and sourced. You can also invoke the module directly if needed:

```bash
python3 -m easy_handeye2.process_samples ...
```


#### Moving the robot

**WARNING**: this will only be available for Iron, and is work-in-progress

Automatic robot movement support remains work-in-progress. The current ROS 2 package includes the calibrator and evaluator RQT tools, but the movement helper path is not the primary workflow here.

This is optional, and can be disabled in both aforementioned cases with:
```bash
ros2 launch easy_handeye2 calibrate.launch.py \
  calibration_type:=eye_in_hand \
  name:=my_eih_calib \
  robot_base_frame:=/base_link \
  robot_effector_frame:=/ee_link \
  tracking_base_frame:=/optical_origin \
  tracking_marker_frame:=/optical_target \
  freehand_robot_movement:=true
```

It will then be the user's responsibility to make the robot publish its own pose into `tf`. Please check that the robot's pose is updated correctly in 
RViz before starting to acquire samples (the robot driver may not work while the teaching mode button is pressed, etc).

The same applies to the validity of the samples. For the calibration to be found reliably, the end effector must be rotated as much as possible 
(up to 90°) about each axis, in both directions. Translating the end effector is not necessary, but can't hurt either.

<img src="docs/img/02_plan_movements.png" width="345"/> <img src="docs/img/04_plan_show.png" width="495"/>

#### Tips for accuracy

The following tips are given in [1], paragraph 1.3.2.

- Maximize rotation between poses.
- Minimize the distance from the target to the camera of the tracking system.
- Minimize the translation between poses.
- Use redundant poses.
- Calibrate the camera intrinsics if necessary / applicable.
- Calibrate the robot if necessary / applicable.

### Publishing
The `publish.launch.py` starts a node that publishes the transformation found during calibration in `tf`.
The parameters are automatically loaded from the yaml file, according to the specified namespace.
For convenience, you can include this file within your own launch script. You can include this file multiple times to 
publish many calibrations simultaneously; the following example publishes one eye-on-base and one eye-in-hand calibration:
```bash
ros2 launch easy_handeye2 publish.launch.py name:=my_eob_calib
ros2 launch easy_handeye2 publish.launch.py name:=my_eih_calib
```
You can have any number of calibrations at once (provided you specify distinct namespaces). 
If you perform again any calibration, you do not need to do anything: the next time you start the system, 
the publisher will automatically fetch the latest information. You can also manually restart the publisher 
nodes (e.g. with `rqt_launch`), if you don't want to shut down the whole system.

### FAQ
#### Why is the calibration wrong?
Please check the [troubleshooting](docs/troubleshooting.md)

#### How can I ...
##### Calibrate an RGBD camera (e.g. Kinect, Xtion, ...) with a robot for automatic object collision avoidance with MoveIt! ?
This is a perfect example of an eye-on-base calibration. You can take a look at this [example launch file](docs/example_launch/ur5_kinect_calibration.launch) written for a UR5 and a Kinect via aruco_ros, or the [example for LWR iiwa with Xtion/Kinect](docs/example_launch/iiwa_kinect_xtion_calibration.launch).
##### Disable the automatic robotic movements GUI?
You can pass the argument `freehand_robot_movement:=true` to `calibrate.launch`.
##### Calibrate one robot against multiple tracking systems?
You can just override the `name` argument of `calibrate.launch.py` to be always different, such that they will never collide. Using the same `namespace` as argument to multiple inclusions of `publish.launch` will allow you to publish each calibration in `tf`.
##### Find the transformation between the bases of two robots?
You could perform the eye-on-base calibration against the same tracking system, and concatenate the results.
##### Find the transformation between two tracking systems?
You could perform the eye-on-base calibration against the same robot, and concatenate the results. This will work also if the tracking systems are completely different and do not use the same markers.

## References

[1] *Tsai, Roger Y., and Reimar K. Lenz. "A new technique for fully autonomous
and efficient 3D robotics hand/eye calibration." Robotics and Automation, IEEE
Transactions on 5.3 (1989): 345-358.*
