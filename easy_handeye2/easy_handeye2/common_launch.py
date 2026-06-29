
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals


arg_calibration_type = DeclareLaunchArgument('calibration_type', choices=["eye_in_hand", "eye_on_base"],
                                             description="Type of the calibration")
arg_tracking_base_frame = DeclareLaunchArgument('tracking_base_frame')
arg_tracking_marker_frame = DeclareLaunchArgument('tracking_marker_frame')
arg_robot_base_frame = DeclareLaunchArgument('robot_base_frame')
arg_robot_effector_frame = DeclareLaunchArgument('robot_effector_frame')
arg_move_group_namespace = DeclareLaunchArgument(
    'move_group_namespace',
    default_value='/',
    description='MoveIt namespace used to resolve the selected move group for optional joint-state recording.',
)
arg_move_group = DeclareLaunchArgument(
    'move_group',
    default_value='',
    description='Optional MoveIt move group to record joint states for while taking calibration samples.',
)

is_eye_in_hand = LaunchConfigurationEquals('calibration_type', 'eye_in_hand')
