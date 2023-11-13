#!/usr/bin/env python
"""

"""

import rospy
import numpy as np
import transformations

from geometry_msgs.msg import (Pose)

from oculus_ros.msg import (ControllerButtons)
from kinova_positional_control.srv import (
    GripperForceGrasping,
    GripperPosition,
)
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Pose

class OculusKinovaMapping:
    """
    
    """

    def __init__(
        self,
        robot_name='my_gen3',
        controller_side='right',
        tracking_mode='press',
        headset_mode='table',
    ):
        """
        
        """

        if controller_side not in ['right', 'left']:
            raise ValueError(
                'controller_side should be either "right" or "left".'
            )

        if tracking_mode not in ['hold', 'press']:
            raise ValueError(
                'tracking_mode should be either "hold" or "press".'
            )

        # # Private constants:

        # # Public constants:
        self.ROBOT_NAME = robot_name
        self.CONTROLLER_SIDE = controller_side
        self.TRACKING_MODE = tracking_mode
        self.HEADSET_MODE = headset_mode

        self.gripper_val = 0
        
        self.rec_100 = 0
        self.flagg = 0

        self.vive_stop = 0

        self.vive_menu = 0

        self.vive_buttons = [0,0,0,0]
        self.vive_axes = [0,0,0]

        self.trigger_press = False

        # # Private variables:
        self.__oculus_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.__oculus_buttons = ControllerButtons()

        self.__tracking_state_machine_state = 0
        self.__gripper_state_machine_state = 0
        self.__mode_state_machine_state = 0
        self.__control_mode = 'position'

        # # Public variables:
        self.pose_tracking = False

        # Last commanded Relaxed IK pose is required to compensate controller
        # input.
        self.last_relaxed_ik_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # This difference is calculated each time the tracking is started and
        # subracted from future inputs during current tracking to compensate for
        # linear misalignment between the global and relaxed_ik coordinate
        # systems.
        self.oculus_relaxed_ik_difference = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        # # ROS node:

        # # Service provider:

        # # Service subscriber:
        self.__gripper_force_grasping = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/gripper/force_grasping',
            GripperForceGrasping,
        )
        self.__gripper_position = rospy.ServiceProxy(
            f'/{self.ROBOT_NAME}/gripper/position',
            GripperPosition,
        )

        # # Topic publisher:
        self.__kinova_pose = rospy.Publisher(
            f'/{self.ROBOT_NAME}/input_pose',
            Pose,
            queue_size=1,
        )

        # # Topic subscriber:
        rospy.Subscriber(
            '/Right_Hand',
            TransformStamped,
            self.__input_pose_callback,
        )

        rospy.Subscriber(
            '/vive/controller_LHR_FF7FBBC0/joy', 
            Joy, 
            self.callback_vive_b)

        rospy.Subscriber(
            f'/{self.ROBOT_NAME}/relaxed_ik/commanded_pose_gcs',
            Pose,
            self.__commanded_pose_callback,
        )

    # # Service handlers:

    # # Topic callbacks:
    def callback_vive_b(self, msg):
        
        self.vive_buttons = msg.buttons
        self.vive_axes = msg.axes
        
        self.gripper_val = self.vive_axes[2]

        self.trigger_press = False
        
        if self.gripper_val == 1:  # Trigger button to hold the gripper state
            self.rec_100 += 1
            self.trigger_press = True
            # vive_menu += 1
            rospy.sleep(0.5)

        if self.vive_buttons[2] == 1:  # Side button to start control
            self.flagg = 1
            rospy.sleep(0.5)
            # print("started")

        if self.vive_buttons[0] == 1:
            self.vive_menu += 1
            # print("home", vive_menu)
            rospy.sleep(0.5)

        if self.vive_buttons[2] == 1 and self.vive_axes[0] == 0:  # Side button as the stop button
            # if vive_menu % 2 == 0 and vive_menu != 0:
            self.vive_stop += 1
            # print("pause", self.vive_stop)
            rospy.sleep(0.5)

    def __input_pose_callback(self, msg):
        """

        """

        negation = 1

        # if self.HEADSET_MODE == 'head':
        #     negation = -1

        self.__oculus_pose['position'][0] = -msg.transform.translation.x
        self.__oculus_pose['position'][1] = -msg.transform.translation.y
        self.__oculus_pose['position'][2] = msg.transform.translation.z - 0.96 + 0.25

        self.__oculus_pose['orientation'][0] = msg.transform.rotation.w
        self.__oculus_pose['orientation'][1] = msg.transform.rotation.x
        self.__oculus_pose['orientation'][2] = msg.transform.rotation.y
        self.__oculus_pose['orientation'][3] = msg.transform.rotation.z

    def __oculus_buttons_callback(self, message):
        """

        """

        self.__oculus_buttons = message

    def __commanded_pose_callback(self, message):
        """
        
        """

        self.last_relaxed_ik_pose['position'][0] = message.position.x
        self.last_relaxed_ik_pose['position'][1] = message.position.y
        self.last_relaxed_ik_pose['position'][2] = message.position.z

        self.last_relaxed_ik_pose['orientation'][0] = message.orientation.w
        self.last_relaxed_ik_pose['orientation'][1] = message.orientation.x
        self.last_relaxed_ik_pose['orientation'][2] = message.orientation.y
        self.last_relaxed_ik_pose['orientation'][3] = message.orientation.z

    # # Private methods:
    def __tracking_state_machine(self, button):
        """
        
        """

        # State 0: Grip button was pressed.
        if (self.__tracking_state_machine_state == 0 and button):
            self.__tracking_state_machine_state = 1

            if self.TRACKING_MODE == 'hold':
                self.__calculate_compensation()
                self.pose_tracking = True

        # State 1: Grip button was released. Tracking is activated.
        elif (self.__tracking_state_machine_state == 1 and not button):
            if self.TRACKING_MODE == 'press':
                self.__tracking_state_machine_state = 2
                self.__calculate_compensation()
                self.pose_tracking = True

            elif self.TRACKING_MODE == 'hold':
                self.__tracking_state_machine_state = 0
                self.pose_tracking = False

        # State 2: Grip button was pressed. Tracking is deactivated.
        elif (self.__tracking_state_machine_state == 2 and button):
            self.__tracking_state_machine_state = 3

            self.pose_tracking = False

        # State 3: Grip button was released.
        elif (self.__tracking_state_machine_state == 3 and not button):
            self.__tracking_state_machine_state = 0

    def __mode_state_machine(self, button):
        """
        
        """

        # State 0: Button was pressed.
        if (self.__mode_state_machine_state == 0 and button):
            self.__mode_state_machine_state = 1

            self.__control_mode = 'full'
            self.__calculate_compensation()

        # State 1: Button was released.
        elif (self.__mode_state_machine_state == 1 and not button):
            self.__mode_state_machine_state = 3

        # State 2: Button was pressed.
        elif (self.__mode_state_machine_state == 3 and button):
            self.__mode_state_machine_state = 4

            self.__control_mode = 'position'

        # State 3: Button was released.
        elif (self.__mode_state_machine_state == 4 and not button):
            self.__mode_state_machine_state = 0

    def __gripper_state_machine(self, button):
        """
        
        """

        # State 0: Button was pressed.
        if (self.__gripper_state_machine_state == 0 and button):
            self.__gripper_force_grasping(0.0)  # 0.0 for default current.
            self.__gripper_state_machine_state = 1

        # State 1: Button was released. Force grasping is activated.
        elif (self.__gripper_state_machine_state == 1 and not button):
            self.__gripper_state_machine_state = 2

        # State 2: Button was pressed. Open the gripper.
        elif (self.__gripper_state_machine_state == 2 and button):
            self.__gripper_position(0.0)  # 0.0 for open position.
            self.__gripper_state_machine_state = 3

        # State 3: Button was released.
        elif (self.__gripper_state_machine_state == 3 and not button):
            self.__gripper_state_machine_state = 0

    def __calculate_compensation(self):
        """Calculates the compensation for coordinate systems misalignment.
        
        """

        self.oculus_relaxed_ik_difference['position'] = (
            self.__oculus_pose['position']
            - self.last_relaxed_ik_pose['position']
        )
        self.oculus_relaxed_ik_difference['orientation'] = (
            transformations.quaternion_multiply(
                self.last_relaxed_ik_pose['orientation'],
                transformations.quaternion_inverse(
                    self.__oculus_pose['orientation']
                ),
            )
        )

    # # Public methods:
    def main_loop(self):
        """
        
        """

        self.__tracking_state_machine(self.vive_buttons[2])

        if self.pose_tracking:
            self.publish_kinova_pose()

        self.__gripper_state_machine(self.trigger_press)
        self.__mode_state_machine(self.vive_buttons[0])

    def publish_kinova_pose(self):
        """
        
        """

        compensated_input_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        compensated_input_pose['position'] = (
            self.__oculus_pose['position']
            - self.oculus_relaxed_ik_difference['position']
        )

        # Use fixed orientation.
        compensated_input_pose['orientation'] = (
            self.last_relaxed_ik_pose['orientation']
        )

        # Use oculus orientation.
        if self.__control_mode == 'full':
            compensated_input_pose['orientation'] = (
                transformations.quaternion_multiply(
                    self.oculus_relaxed_ik_difference['orientation'],
                    self.__oculus_pose['orientation'],
                )
            )

        pose_message = Pose()
        pose_message.position.x = compensated_input_pose['position'][0]
        pose_message.position.y = compensated_input_pose['position'][1]
        pose_message.position.z = compensated_input_pose['position'][2]

        pose_message.orientation.w = compensated_input_pose['orientation'][0]
        pose_message.orientation.x = compensated_input_pose['orientation'][1]
        pose_message.orientation.y = compensated_input_pose['orientation'][2]
        pose_message.orientation.z = compensated_input_pose['orientation'][3]

        self.__kinova_pose.publish(pose_message)


def node_shutdown():
    """
    
    """

    print('\nNode is shutting down...\n')

    # TODO: Stop arm motion.

    print('\nNode is shut down.\n')


def main():
    """

    """

    # # ROS node:
    rospy.init_node('oculus_kinova_mapping')
    rospy.on_shutdown(node_shutdown)

    kinova_name = rospy.get_param(
        param_name=f'{rospy.get_name()}/robot_name',
        default='my_gen3',
    )

    controller_side = rospy.get_param(
        param_name=f'{rospy.get_name()}/controller_side',
        default='right',
    )

    tracking_mode = rospy.get_param(
        param_name=f'{rospy.get_name()}/tracking_mode',
        default='press',
    )

    oculus_kinova_mapping = OculusKinovaMapping(
        robot_name=kinova_name,
        controller_side=controller_side,
        tracking_mode=tracking_mode,
        headset_mode='table',
    )
    print('\nOculus-Kinova mapping is ready.\n')

    while not rospy.is_shutdown():
        oculus_kinova_mapping.main_loop()


if __name__ == '__main__':
    main()