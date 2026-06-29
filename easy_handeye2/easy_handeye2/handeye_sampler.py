import os
import pathlib
import threading
from typing import Optional
import xml.etree.ElementTree as ET

import easy_handeye2_msgs.msg
import tf2_ros
import yaml
from sensor_msgs.msg import JointState
from rclpy.parameter_client import AsyncParameterClient
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rosidl_runtime_py import message_to_yaml, set_message_fields
from easy_handeye2_msgs.msg import Sample, SampleList
import rclpy
from rclpy.time import Duration, Time

from easy_handeye2 import SAMPLES_DIRECTORY
from easy_handeye2.handeye_calibration import HandeyeCalibrationParameters

import easy_handeye2


class HandeyeSampler:
    """
    Manages the samples acquired from tf.
    """

    def __init__(self, node: rclpy.node.Node, handeye_parameters: HandeyeCalibrationParameters):
        self.node = node
        self.handeye_parameters = handeye_parameters

        # tf structures
        self.tfBuffer: tf2_ros.Buffer = Buffer(cache_time=Duration(seconds=2), node=node)
        """
        used to get transforms to build each sample
        """
        self.tfListener: tf2_ros.TransformListener = TransformListener(self.tfBuffer, self.node, spin_thread=True)
        """
        used to get transforms to build each sample
        """
        self.tfBroadcaster: tf2_ros.TransformBroadcaster = TransformBroadcaster(self.node)
        """
        used to publish the calibration after saving it
        """

        # internal input data
        self.samples: easy_handeye2.msg.SampleList = SampleList()
        """
        list of acquired samples
        """
        self._joint_state_lock = threading.Lock()
        self._latest_joint_state: JointState | None = None
        self._tracked_joint_names: list[str] = []
        self._joint_state_subscription = None
        self._joint_state_warning_emitted = False
        self._configure_joint_state_capture()

    def wait_for_tf_init(self) -> bool:
        """
        Waits until all needed frames are present in tf.
        """
        base_frame = self.handeye_parameters.robot_base_frame
        effector_frame = self.handeye_parameters.robot_effector_frame
        camera_frame = self.handeye_parameters.tracking_base_frame
        marker_frame = self.handeye_parameters.tracking_marker_frame
        self.node.get_logger().info('Checking that the expected transforms are available in tf')
        self.node.get_logger().info(f'Robot transform: {base_frame} -> {effector_frame}')
        self.node.get_logger().info(f'Tracking transform: {camera_frame} -> {marker_frame}')
        try:
            self.tfBuffer.lookup_transform(base_frame, effector_frame, Time(), Duration(seconds=10))
        except tf2_ros.TransformException as e:
            self.node.get_logger().error(
                'The specified tf frames for the robot base and hand do not seem to be connected')
            self.node.get_logger().error('Run the following command and check its output:')
            self.node.get_logger().error(f'ros2 run tf2_ros tf2_echo {base_frame} {effector_frame}')
            self.node.get_logger().error(
                f'You may need to correct the base_frame or effector_frame argument passed to the easy_handeye2 launch file')
            self.node.get_logger().error(f'Underlying tf exception: {e}')
            return False

        try:
            self.tfBuffer.lookup_transform(camera_frame, marker_frame, Time(), Duration(seconds=10))
        except tf2_ros.TransformException as e:
            self.node.get_logger().error(
                'The specified tf frames for the tracking system base/camera and marker do not seem to be connected')
            self.node.get_logger().error('Run the following command and check its output:')
            self.node.get_logger().error(f'ros2 run tf2_ros tf2_echo {camera_frame} {marker_frame}')
            self.node.get_logger().error(
                f'You may need to correct the base_frame or effector_frame argument passed to the easy_handeye2 launch file')
            self.node.get_logger().error(f'Underlying tf exception: {e}')
            return False

        self.node.get_logger().info('All expected transforms are available on tf; ready to take samples')
        return True

    def _get_transforms(self, time: Optional[rclpy.time.Time] = None) -> Sample | None:
        """
        Samples the transforms at the given time.
        """
        if time is None:
            time = self.node.get_clock().now() - rclpy.time.Duration(nanoseconds=200000000)

        # here we trick the library (it is actually made for eye_in_hand only). Trust me, I'm an engineer
        try:
            if self.handeye_parameters.calibration_type == 'eye_in_hand':
                robot = self.tfBuffer.lookup_transform(self.handeye_parameters.robot_base_frame,
                                                       self.handeye_parameters.robot_effector_frame, time,
                                                       Duration(seconds=1))
            else:
                robot = self.tfBuffer.lookup_transform(self.handeye_parameters.robot_effector_frame,
                                                       self.handeye_parameters.robot_base_frame, time,
                                                       Duration(seconds=1))
            tracking = self.tfBuffer.lookup_transform(self.handeye_parameters.tracking_base_frame,
                                                      self.handeye_parameters.tracking_marker_frame, time,
                                                      Duration(seconds=1))
        except tf2_ros.ExtrapolationException as e:
            self.node.get_logger().error(f'Failed to get the tracking transform: {e}')
            return None

        ret = Sample()
        ret.robot = robot.transform
        ret.tracking = tracking.transform
        self._attach_group_joint_state(ret)
        return ret

    def _configure_joint_state_capture(self):
        move_group = self.handeye_parameters.move_group.strip()
        if not move_group:
            self.node.get_logger().info('No move_group configured for calibration sample recording; TF data only will be stored')
            return

        try:
            self._tracked_joint_names = self._resolve_group_joint_names(move_group)
        except Exception as exc:
            self.node.get_logger().warn(
                f'Failed to resolve move group "{move_group}" for calibration sample recording: {exc}'
            )
            self._tracked_joint_names = []
            return

        if not self._tracked_joint_names:
            self.node.get_logger().warn(
                f'Move group "{move_group}" was resolved, but it exposes no active joints; TF data only will be stored'
            )
            return

        self._joint_state_subscription = self.node.create_subscription(
            JointState,
            'joint_states',
            self._joint_state_callback,
            10,
        )
        self.node.get_logger().info(
            f'Recording joint states for move group "{move_group}" with joints: {", ".join(self._tracked_joint_names)}'
        )

    def _resolve_group_joint_names(self, move_group: str) -> list[str]:
        move_group_node = self._move_group_node_name()
        param_client = AsyncParameterClient(self.node, move_group_node)
        if not param_client.wait_for_services(timeout_sec=3.0):
            raise RuntimeError(f'parameter service for node "{move_group_node}" is not available')

        requested_names = ['robot_description', 'robot_description_semantic']
        future = param_client.get_parameters(requested_names)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if not future.done():
            raise RuntimeError(f'timed out reading robot description parameters from "{move_group_node}"')

        result = future.result()
        if result is None:
            raise RuntimeError(f'failed to read robot description parameters from "{move_group_node}"')

        params = {
            name: value.string_value
            for name, value in zip(requested_names, result.values)
        }
        robot_description = params.get('robot_description', '')
        robot_description_semantic = params.get('robot_description_semantic', '')
        if not robot_description or not robot_description_semantic:
            raise RuntimeError(
                f'"{move_group_node}" does not expose robot_description and robot_description_semantic parameters'
            )

        return self._extract_group_joints_from_descriptions(robot_description, robot_description_semantic, move_group)

    def _move_group_node_name(self) -> str:
        namespace = (self.handeye_parameters.move_group_namespace or '/').strip()
        if not namespace or namespace == '/':
            return '/move_group'
        namespace = '/' + namespace.strip('/')
        return f'{namespace}/move_group'

    def _extract_group_joints_from_descriptions(
        self,
        robot_description: str,
        robot_description_semantic: str,
        move_group: str,
    ) -> list[str]:
        urdf_root = ET.fromstring(robot_description)
        srdf_root = ET.fromstring(robot_description_semantic)

        joints_by_parent: dict[str, list[dict[str, str]]] = {}
        for joint in urdf_root.findall('joint'):
            parent = joint.find('parent')
            child = joint.find('child')
            if parent is None or child is None:
                continue
            joints_by_parent.setdefault(parent.attrib['link'], []).append({
                'name': joint.attrib['name'],
                'type': joint.attrib.get('type', ''),
                'child': child.attrib['link'],
            })

        groups = {group.attrib['name']: group for group in srdf_root.findall('group')}
        if move_group not in groups:
            raise RuntimeError(f'group "{move_group}" not found in robot_description_semantic')

        resolved: list[str] = []
        seen: set[str] = set()

        def add_joint(joint_name: str):
            if joint_name not in seen:
                seen.add(joint_name)
                resolved.append(joint_name)

        def collect_chain(base_link: str, tip_link: str):
            chain_joints = self._find_chain_joints(joints_by_parent, base_link, tip_link)
            if chain_joints is None:
                raise RuntimeError(f'could not resolve chain from "{base_link}" to "{tip_link}"')
            for joint_name in chain_joints:
                add_joint(joint_name)

        def collect_group(group_name: str):
            group = groups.get(group_name)
            if group is None:
                raise RuntimeError(f'group "{group_name}" not found in robot_description_semantic')
            for chain in group.findall('chain'):
                collect_chain(chain.attrib['base_link'], chain.attrib['tip_link'])
            for joint in group.findall('joint'):
                add_joint(joint.attrib['name'])
            for subgroup in group.findall('group'):
                collect_group(subgroup.attrib['name'])

        collect_group(move_group)
        return resolved

    def _find_chain_joints(
        self,
        joints_by_parent: dict[str, list[dict[str, str]]],
        base_link: str,
        tip_link: str,
    ) -> list[str] | None:
        def dfs(link_name: str, active_joints: list[str], visited_links: set[str]) -> list[str] | None:
            if link_name == tip_link:
                return active_joints
            if link_name in visited_links:
                return None

            next_visited = set(visited_links)
            next_visited.add(link_name)
            for joint in joints_by_parent.get(link_name, []):
                next_active = list(active_joints)
                if joint['type'] != 'fixed':
                    next_active.append(joint['name'])
                result = dfs(joint['child'], next_active, next_visited)
                if result is not None:
                    return result
            return None

        return dfs(base_link, [], set())

    def _joint_state_callback(self, msg: JointState):
        with self._joint_state_lock:
            self._latest_joint_state = msg

    def _attach_group_joint_state(self, sample: Sample):
        if not self._tracked_joint_names:
            return

        with self._joint_state_lock:
            joint_state = self._latest_joint_state

        if joint_state is None:
            if not self._joint_state_warning_emitted:
                self.node.get_logger().warn(
                    'A move group was configured for calibration sample recording, but no /joint_states message has been received yet'
                )
                self._joint_state_warning_emitted = True
            return

        joint_map = {
            name: position
            for name, position in zip(joint_state.name, joint_state.position)
        }
        missing_joints = [name for name in self._tracked_joint_names if name not in joint_map]
        if missing_joints:
            if not self._joint_state_warning_emitted:
                self.node.get_logger().warn(
                    'Skipping joint-state recording for this sample because /joint_states is missing move-group joints: '
                    + ', '.join(missing_joints)
                )
                self._joint_state_warning_emitted = True
            return

        sample.joint_names = list(self._tracked_joint_names)
        sample.joint_values = [joint_map[name] for name in self._tracked_joint_names]

    def current_transforms(self) -> Sample | None:
        return self._get_transforms()

    def take_sample(self) -> bool:
        """
        Samples the transformations and appends the sample to the list.
        """
        try:
            self.node.get_logger().info("Taking a sample...")
            self.node.get_logger().info("all frames: " + self.tfBuffer.all_frames_as_string())
            sample = self._get_transforms()
            if sample is None:
                return False

            self.node.get_logger().info("Got a sample")
            new_samples = self.samples.samples
            new_samples.append(sample)
            self.samples.samples = new_samples
            return True
        except:
            return False

    def remove_sample(self, index: int) -> int:
        """
        Removes a sample from the list. Returns the updated number of samples
        """
        if 0 <= index < len(self.samples.samples):
            new_samples = self.samples.samples
            del new_samples[index]
            self.samples.samples = new_samples
        return len(self.samples.samples)

    def get_samples(self) -> easy_handeye2_msgs.msg.SampleList:
        """
        Returns the samples accumulated so far.
        """
        return self.samples

    @staticmethod
    def _filepath_for_samplelist(name) -> pathlib.Path:
        return SAMPLES_DIRECTORY / f'{name}.samples'

    def load_samples(self) -> bool:
        filepath = HandeyeSampler._filepath_for_samplelist(self.handeye_parameters.name)
        with open(filepath) as f:
            m = yaml.full_load(f.read())
            ret = SampleList()
            set_message_fields(ret, m)
            self.samples = ret
        return True

    def save_samples(self) -> bool:
        if not os.path.exists(SAMPLES_DIRECTORY):
            os.makedirs(SAMPLES_DIRECTORY)
        filepath = HandeyeSampler._filepath_for_samplelist(self.handeye_parameters.name)
        with open(filepath, 'w') as f:
            f.write(message_to_yaml(self.samples))
        return True
