#!/usr/bin/env python
"""

"""

import rospy
import math
import numpy as np
import transformations

from std_msgs.msg import (Bool)
from geometry_msgs.msg import (Pose)

# from kortex_driver.msg import (BaseCyclic_Feedback)
from kortex_driver.srv import (
    Stop,
    Base_ClearFaults,
    # ApplyEmergencyStop,
)
from kinova_positional_control.srv import (PidVelocityLimit)
from relaxed_ik_ros1.msg import (EEPoseGoals)


class KinovaPositionalControl:
    """
    
    """

    def __init__(
        self,
        robot_name='my_gen3',
        mounting_angles_deg=(0.0, 0.0, 0.0),
        ee_starting_position=(0.57, 0.0, 0.43),
        workspace_radius=1.2,
        starting_pose={
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1, 0.0, 0.0, 0]),
        },
    ):
        """
        
        """

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name

        # Rotation matrices around axes. If the arm is mounted on the table,
        # mounting angles should all be zeros.
        self.ROTATE_X = transformations.rotation_matrix(
            math.radians(mounting_angles_deg[0]),
            (1, 0, 0),
        )
        self.ROTATE_Y = transformations.rotation_matrix(
            math.radians(mounting_angles_deg[1]),
            (0, 1, 0),
        )
        self.ROTATE_Z = transformations.rotation_matrix(
            math.radians(mounting_angles_deg[2]),
            (0, 0, 1),
        )

        # Rotation matrices from Global Coordinate System (parallel to the
        # floor, X facing forward globally) to Relaxed IK Coordinate System
        # (parallel to Kinova Arm base, X facing according to Kinova Arm) and
        # back. If the arm is mounted on the table, no rotations will be
        # applied.
        self.ROTATE_GCS_TO_RIKCS = transformations.concatenate_matrices(
            self.ROTATE_X,
            self.ROTATE_Y,
            self.ROTATE_Z,
        )
        self.ROTATE_RIKCS_TO_GCS = transformations.inverse_matrix(
            self.ROTATE_GCS_TO_RIKCS
        )

        self.WORKSPACE_CENTER = np.negative(ee_starting_position)
        self.WORKSPACE_RADIUS = workspace_radius

        self.STARTING_POSE = starting_pose

        # # Private variables:

        # # Public variables:
        self.is_initialized = False
        self.joint_control_initialized = False

        self.is_motion_finished = True

        # Input pose in Global and Relaxed IK coordinate systems.
        self.input_pose = {
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
        }

        # Last commanded Relaxed IK pose is required to compensate controller
        # input.
        self.last_relaxed_ik_pose = {
            'gcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                },
            'rikcs':
                {
                    'position': np.array([0.0, 0.0, 0.0]),
                    'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
                }
        }

        # # Service provider:

        # # Service subscriber:
        self.__pid_velocity_limit = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/pid/velocity_limit',
            PidVelocityLimit,
        )

        self.__stop_arm = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/stop',
            Stop,
        )
        self.__clear_arm_faults = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/base/clear_faults',
            Base_ClearFaults,
        )

        # # Topic publisher:
        self.__relaxed_ik_target_rikcs = rospy.Publisher(
            f'/{self.ROBOT_NAME}/relaxed_ik/ee_pose_goals',
            EEPoseGoals,
            queue_size=1,
        )
        self.__relaxed_ik_commanded_gcs = rospy.Publisher(
            f'/{self.ROBOT_NAME}/relaxed_ik/commanded_pose_gcs',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/input_pose',
            Pose,
            self.__input_pose_callback,
        )

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/pid/motion_finished',
            Bool,
            self.__pid_motion_finished_callback,
        )

    # # Service handlers:

    # # Topic callbacks:
    def __input_pose_callback(self, msg):
        """
        
        """

        self.input_pose['gcs']['position'][0] = msg.position.x
        self.input_pose['gcs']['position'][1] = msg.position.y
        self.input_pose['gcs']['position'][2] = msg.position.z

        self.input_pose['gcs']['orientation'][0] = msg.orientation.w
        self.input_pose['gcs']['orientation'][1] = msg.orientation.x
        self.input_pose['gcs']['orientation'][2] = msg.orientation.y
        self.input_pose['gcs']['orientation'][3] = msg.orientation.z

    def __pid_motion_finished_callback(self, msg):
        """
        
        """

        if not self.is_initialized:
            self.joint_control_initialized = True

        self.is_motion_finished = msg.data

    # # Private methods:
    def __compose_pose_message(self, target_pose):
        """
        target_pose: dict
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0])
        
        """

        # NOTE: These two checks might not be needed, check function usage.
        if not isinstance(target_pose, dict):
            raise TypeError('target_pose is not a dictionary.')

        for key in ['position', 'orientation']:
            if key not in target_pose:
                raise KeyError(f'key {key} not found in target_pose.')

        pose_message = Pose()
        pose_message.position.x = target_pose['position'][0]
        pose_message.position.y = target_pose['position'][1]
        pose_message.position.z = target_pose['position'][2]

        pose_message.orientation.w = target_pose['orientation'][0]
        pose_message.orientation.x = target_pose['orientation'][1]
        pose_message.orientation.y = target_pose['orientation'][2]
        pose_message.orientation.z = target_pose['orientation'][3]

        return pose_message

    def __check_boundaries(self, position):
        # Calculate the vector from the center to the position.
        vector_to_position = [
            position[i] - self.WORKSPACE_CENTER[i]
            for i in range(len(self.WORKSPACE_CENTER))
        ]

        # Calculate the distance from the center to the position.
        distance_to_position = math.sqrt(
            sum([x**2 for x in vector_to_position])
        )

        # If the position is inside the sphere, return its coordinates.
        if distance_to_position <= self.WORKSPACE_RADIUS:
            return position

        # Calculate the vector from the center to the closest position on the
        # sphere surface.
        vector_to_surface = [
            vector_to_position[i] * self.WORKSPACE_RADIUS / distance_to_position
            for i in range(len(self.WORKSPACE_CENTER))
        ]

        # Calculate the coordinates of the closest position on the sphere
        # surface.
        closest_position = np.array(
            [
                self.WORKSPACE_CENTER[i] + vector_to_surface[i]
                for i in range(len(self.WORKSPACE_CENTER))
            ]
        )

        return closest_position

    def __wait_for_motion(self):
        """Blocks code execution until the flag is set or a node is shut down.
        
        """

        rospy.sleep(1)  # Allow a motion to start.

        while not self.is_motion_finished and not rospy.is_shutdown():
            pass

    # # Public methods:
    def main_loop(self):
        """
        
        """

        if not self.is_initialized:
            return

        self.set_target_pose(self.input_pose['gcs'], 'gcs')

        # Publish a commanded target position in Global CS.
        self.__relaxed_ik_commanded_gcs.publish(
            self.__compose_pose_message(self.last_relaxed_ik_pose['gcs'])
        )

    def node_shutdown(self):
        """
        
        """

        print(
            f'\n/{self.ROBOT_NAME}/positional_control: node is shutting down...'
        )

        # Stop the arm motion.
        self.__stop_arm()

        print(f'\n/{self.ROBOT_NAME}/positional_control: node has shut down.')

    def initialization(self):
        """
        
        """

        print(
            f'\n/{self.ROBOT_NAME}/positional_control: waiting for joints control...\n'
        )

        # Wait for joints control to initialize.
        while not self.joint_control_initialized and not rospy.is_shutdown():
            pass

        print(
            f'\n/{self.ROBOT_NAME}/positional_control: joints control has initialized.\n'
        )

        self.__clear_arm_faults()

        # Limit joint velocities to 20% for homing.
        self.__pid_velocity_limit(0.2)

        print(
            f'\n/{self.ROBOT_NAME}/positional_control: homing has started...\n'
        )

        # Let the node get initialized.
        rospy.sleep(2)

        # Starting pose.
        self.input_pose = {'gcs': self.STARTING_POSE}
        self.set_target_pose(self.input_pose['gcs'], 'gcs')
        self.__wait_for_motion()

        print(
            f'\n/{self.ROBOT_NAME}/positional_control: homing has finished.\n'
        )

        self.__pid_velocity_limit(1.0)

        self.is_initialized = True

    def set_target_pose(self, target_pose, coordinate_system):
        """
        target_pose: dict
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0])
        
        """

        if not isinstance(target_pose, dict):
            raise TypeError('target_pose is not a dictionary.')

        for key in ['position', 'orientation']:
            if key not in target_pose:
                raise KeyError(f'Key {key} was not found in target_pose.')

            if not isinstance(target_pose[key], np.ndarray):
                raise TypeError(
                    'Dictionary values should be of type np.ndarray.'
                )

        # Check if coordinates are within the arm's workspace.
        target_pose['position'] = self.__check_boundaries(
            target_pose['position']
        )

        self.last_relaxed_ik_pose['gcs'] = target_pose.copy()
        self.last_relaxed_ik_pose['rikcs'] = target_pose.copy()

        if coordinate_system == 'gcs':
            # Update target pose in Relaxed IK CS.
            self.last_relaxed_ik_pose['rikcs']['position'] = np.matmul(
                self.ROTATE_GCS_TO_RIKCS[0:3, 0:3],
                target_pose['position'],
            )

        elif coordinate_system == 'rikcs':
            # Update target pose in Global IK CS.
            self.last_relaxed_ik_pose['gcs']['position'] = np.matmul(
                self.ROTATE_RIKCS_TO_GCS[0:3, 0:3],
                target_pose['position'],
            )

        else:
            raise ValueError('Invalid coordinate_system value.')

        # Form a message for the right arm.
        right_arm_pose_rikcs = self.__compose_pose_message(
            self.last_relaxed_ik_pose['rikcs']
        )

        # TODO: Add left arm support.
        # Form a message for the left arm.
        left_arm_pose_rikcs = self.__compose_pose_message(
            self.last_relaxed_ik_pose['rikcs']
        )

        # Form a message for the relaxed IK setpoint topic.
        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.ee_poses.append(right_arm_pose_rikcs)
        ee_pose_goals.ee_poses.append(left_arm_pose_rikcs)
        ee_pose_goals.header.seq = 0

        self.__relaxed_ik_target_rikcs.publish(ee_pose_goals)


def main():
    """
    
    """

    rospy.init_node('positional_control')

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    pose_controller = KinovaPositionalControl(
        robot_name=kinova_name,
        workspace_radius=1.0,
    )

    rospy.on_shutdown(pose_controller.node_shutdown)

    pose_controller.initialization()

    print(f'\n/{kinova_name}/positional_control: ready.\n')

    while not rospy.is_shutdown():
        pose_controller.main_loop()


if __name__ == '__main__':
    main()
