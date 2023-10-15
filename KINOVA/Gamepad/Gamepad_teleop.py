#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
from kortex_driver.srv import *
from kortex_driver.msg import *
from sensor_msgs.msg import Joy

flagg = 0
flagg_home = 0
gamepad_val_x = 0
gamepad_val_y = 0
gamepad_val_z_trans = 0
gamepad_val_z_rot = 0
sum_test = 0
cam_switch = 0
gripper_val = 0
grip_com = 0

class ExampleFullArmMovement:
    def __init__(self):
        try:
            rospy.init_node('example_full_arm_movement_python')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
            rospy.wait_for_service(play_cartesian_trajectory_full_name)
            self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name, PlayCartesianTrajectory)

            play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
            rospy.wait_for_service(play_joint_trajectory_full_name)
            self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            # For Gamepad ########################
            self.gamepad = rospy.Subscriber('/joy', Joy, self.callback_gamepad, queue_size = 1)
            self.pub_vel = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size = 1)
            # For Gamepad ########################
        
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
    
    # For Gamepad ########################
    def callback_gamepad(self,data):

        self.gamepad_button = data.buttons
        self.gamepad_axes = data.axes
        global flagg, sum_test, flagg_home
        global gamepad_val_x, gamepad_val_y, gamepad_val_z_trans, gamepad_val_z_rot, gripper_val, grip_com, cam_switch

        
        gamepad_val_x = 5*(self.gamepad_axes[1])/100
        gamepad_val_y = 5*(self.gamepad_axes[0])/100
        gamepad_val_z_trans = 5*(self.gamepad_axes[4])/100
        gamepad_val_z_rot = 5*(self.gamepad_axes[3])/100

        if self.gamepad_axes[5] == -1:
            gripper_val += 1
            rospy.sleep(0.5)

        if gripper_val % 2 == 1:
            grip_com = 1
        else:
            grip_com = 0

        #gripper_val = 1 - (self.gamepad_axes[5] + 1) / 2 # change the range of the raw data from -1 ~ 1 to 1 ~ 0

        if self.gamepad_button[0] == 1:
            flagg = 1
        if self.gamepad_button[1] == 1:
            sum_test += 1
        if self.gamepad_button[2] == 1:
            flagg_home += 1
        if self.gamepad_button[3] == 1:
            cam_switch += 1
    # For Gamepad ########################  


    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return True

    def example_set_cartesian_reference_frame(self):
        # Prepare the request with the frame we want to set
        req = SetCartesianReferenceFrameRequest()
        req.input.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_MIXED

        # Call the service
        try:
            self.set_cartesian_reference_frame()
        except rospy.ServiceException:
            rospy.logerr("Failed to call SetCartesianReferenceFrame")
            return False
        else:
            rospy.loginfo("Set the cartesian reference frame successfully")

        # Wait a bit
        rospy.sleep(0.25)
        return True

    def example_send_cartesian_pose(self):
        # Get the actual cartesian pose to increment it
        # You can create a subscriber to listen to the base_feedback
        # Here we only need the latest message in the topic though
        feedback = rospy.wait_for_message("/" + self.robot_name + "/base_feedback", BaseCyclic_Feedback)
        global gamepad_val
        req = PlayCartesianTrajectoryRequest()
        req.input.target_pose.x = feedback.base.commanded_tool_pose_x 
        req.input.target_pose.y = feedback.base.commanded_tool_pose_y
        req.input.target_pose.z = feedback.base.commanded_tool_pose_z + gamepad_val
        req.input.target_pose.theta_x = feedback.base.commanded_tool_pose_theta_x
        req.input.target_pose.theta_y = feedback.base.commanded_tool_pose_theta_y
        req.input.target_pose.theta_z = feedback.base.commanded_tool_pose_theta_z

        pose_speed = CartesianSpeed()
        pose_speed.translation = 0.1
        pose_speed.orientation = 15

        # The constraint is a one_of in Protobuf. The one_of concept does not exist in ROS
        # To specify a one_of, create it and put it in the appropriate list of the oneof_type member of the ROS object : 
        req.input.constraint.oneof_type.speed.append(pose_speed)

        # Call the service
        rospy.loginfo(gamepad_val)
        try:
            self.play_cartesian_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayCartesianTrajectory")
            return False
        else:
            return True

    def example_send_joint_angles(self):
        # Create the list of angles
        req = PlayJointTrajectoryRequest()
        # Here the arm is vertical (all zeros)
        for i in range(self.degrees_of_freedom):
            temp_angle = JointAngle() 
            temp_angle.joint_identifier = i
            temp_angle.value = 0.0
            req.input.joint_angles.joint_angles.append(temp_angle)
        
        # Send the angles
        rospy.loginfo("Sending the robot vertical...")
        try:
            self.play_joint_trajectory(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call PlayJointTrajectory")
            return False
        else:
            return True

    def example_send_gripper_command(self, value):
        # Initialize the request
        # This works for the Robotiq Gripper 2F_85
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        # rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            return True

    def main(self):
        global flagg,gamepad_val_x, gamepad_val_y, gamepad_val_z_trans, gamepad_val_z_rot, sum_test, flagg_home,grip_com, cam_switch
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/full_arm_movement_python")
        except:
            pass

        if success:
            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            # success &= self.example_home_the_robot()
            # rospy.sleep(5.0)
            #*******************************************************************************

            #*******************************************************************************
            # Example of gripper command
            # Let's fully open the gripper
            '''if self.is_gripper_present:
                success &= self.example_send_gripper_command()
                rospy.sleep(2.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")'''
            #*******************************************************************************

            #*******************************************************************************
            # Set the reference frame to "Mixed"
            success &= self.example_set_cartesian_reference_frame()

            # Example of cartesian pose
            # Let's make it move in Z
            msg_test = TwistCommand()

            while flagg == 0:

                if self.is_gripper_present:
                    success &= self.example_send_gripper_command(grip_com)
                    # print(gripper_val)
                else:
                    rospy.logwarn("No gripper is present on the arm.")

                if cam_switch %2 == 0:
                    if flagg_home % 2 == 0:
                        if sum_test % 2 == 0:
                            msg_test.reference_frame = 3 # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                            msg_test.twist.linear_x = gamepad_val_x*3
                            msg_test.twist.linear_y = gamepad_val_y*3
                            msg_test.twist.linear_z = gamepad_val_z_trans*3
                            msg_test.twist.angular_x = 0
                            msg_test.twist.angular_y = 0
                            msg_test.twist.angular_z = 0
                            msg_test.duration = 0
                            self.pub_vel.publish(msg_test)
                        else:
                            msg_test.reference_frame = 3 # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                            msg_test.twist.linear_x = 0
                            msg_test.twist.linear_y = 0
                            msg_test.twist.linear_z = 0
                            msg_test.twist.angular_x = -gamepad_val_y*10
                            msg_test.twist.angular_y = gamepad_val_x*10
                            msg_test.twist.angular_z = gamepad_val_z_rot*10
                            msg_test.duration = 0
                            self.pub_vel.publish(msg_test)
                    else: 
                        success &= self.example_home_the_robot()
                        rospy.sleep(5)
                else:
                    if flagg_home % 2 == 0:
                        if sum_test % 2 == 0:
                            msg_test.reference_frame = 2 # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                            msg_test.twist.linear_x = gamepad_val_y*3
                            msg_test.twist.linear_y = gamepad_val_x*3
                            msg_test.twist.linear_z = gamepad_val_z_trans*3
                            msg_test.twist.angular_x = 0
                            msg_test.twist.angular_y = 0
                            msg_test.twist.angular_z = 0
                            msg_test.duration = 0
                            self.pub_vel.publish(msg_test)
                        else:
                            msg_test.reference_frame = 2 # 0 = UNSPECIFIED; 1 = MIXED; 2 = TOOL; 3 = BASE
                            msg_test.twist.linear_x = 0
                            msg_test.twist.linear_y = 0
                            msg_test.twist.linear_z = 0
                            msg_test.twist.angular_x = gamepad_val_x*10
                            msg_test.twist.angular_y = -gamepad_val_y*10
                            msg_test.twist.angular_z = -gamepad_val_z_rot*10
                            msg_test.duration = 0
                            self.pub_vel.publish(msg_test)
                    else: 
                        success &= self.example_home_the_robot()
                        rospy.sleep(5)
            # Example of gripper command
            # Let's close the gripper at 50%
            '''if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.5)
                rospy.sleep(2.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")'''    
            #*******************************************************************************
        
        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/full_arm_movement_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    ex = ExampleFullArmMovement()
    ex.main()
