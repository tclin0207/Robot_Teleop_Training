#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy
import tf.transformations
import socket
from math import pi
import numpy

RATE = 20

# degree to radians
def d2r(i):
    return float(i) * pi / 180.0

def fillin_pos(data, rot=[0, 0, 0, 0]):
    #rot is the offset of orientation for given frame
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "world"
    pose.pose.position.x = float(data[0])
    pose.pose.position.y = float(data[1])
    pose.pose.position.z = float(data[2])
    pose.pose.orientation.x = float(data[3])
    pose.pose.orientation.y = float(data[4])
    pose.pose.orientation.z = float(data[5])
    pose.pose.orientation.w = float(data[6])

    msg = TransformStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"
    msg.child_frame_id = "test"
    msg.transform.translation.x = float(data[0])
    msg.transform.translation.y = float(data[1])
    msg.transform.translation.z = float(data[2])
    msg.transform.rotation.x = float(data[3])
    msg.transform.rotation.y = float(data[4])
    msg.transform.rotation.z = float(data[5])
    msg.transform.rotation.w = float(data[6])

    return pose, msg

def fillin_joy(data, ctrl_name):
    msg = Joy()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ctrl_name
    msg.axes =  [float(i) for i in data[0:3]]
    msg.buttons = [int(i) for i in data[3:8]]
    return msg


def vive_status_pub():
    # connect state and reconnect counter
    con_state = False
    con_cnt = 0
    # Don't know what is the /vive/twist3 or 4 is for.
    #pub_twist_r = rospy.Publisher('/vive/twist3', PoseStamped, queue_size=1)
    #pub_twist_l = rospy.Publisher('/vive/twist4', PoseStamped, queue_size=1)

    # This one is send to control the head camera
    #pub_head    = rospy.Publisher('/vive/twist5', PoseStamped, queue_size=1)
    # pub_head  = rospy.Publisher('/Head_twist', TransformStamped, queue_size=1)
    pub_head  = rospy.Publisher('/Head_Motion', PoseStamped, queue_size=1)
    
    # Those are send to control the end effectors
    pub_pos_r = rospy.Publisher('/Right_Hand',TransformStamped, queue_size=1)
    pub_pos_l = rospy.Publisher('/Left_Hand', TransformStamped, queue_size=1)

    pub_key_l = rospy.Publisher('/Left_Buttons', Joy, queue_size=1)
    pub_key_r = rospy.Publisher('/Right_Buttons', Joy, queue_size=1)

    # eye_gaze_x = rospy.Publisher('/Gaze_X', Float64, queue_size=1)
    # eye_gaze_y = rospy.Publisher('/Gaze_Y', Float64, queue_size=1)
    # eye_gaze_z = rospy.Publisher('/Gaze_Z', Float64, queue_size=1)

    # chest_pub = rospy.Publisher('/Chest', TransformStamped, queue_size=1)
    # lua_pub = rospy.Publisher('/L_Upperarm', TransformStamped, queue_size=1)
    # rua_pub = rospy.Publisher('/R_Upperarm', TransformStamped, queue_size=1)
    # lla_pub = rospy.Publisher('/L_Lowerarm', TransformStamped, queue_size=1)
    # rla_pub = rospy.Publisher('/R_Lowerarm', TransformStamped, queue_size=1)
    # waist_pub = rospy.Publisher('/Waist', TransformStamped, queue_size=1)

    # pupil_pub = rospy.Publisher('/Pupil', Float64MultiArray, queue_size=1)
    # blink_pub = rospy.Publisher('/Blink', Float64MultiArray, queue_size=1)

    # pupil_data = Float64MultiArray()
    # blink_data = Float64MultiArray()

    rospy.init_node('vive_status', anonymous=True)
    rate = rospy.Rate(RATE)

    his_x = 0
    his_y = 0

    while not rospy.is_shutdown():
        
        try:
            sock.settimeout(1)
            buffer, addr = sock.recvfrom(2048)
            if not con_state:
                print("connected")
                con_state = True
            # print(buffer)
            buffer = buffer.decode()
            buffer = buffer.split(',')
            #print(buffer) 

            head_pos, head_tran= fillin_pos(buffer[1:8])
            right_pos, msg_r = fillin_pos(buffer[9:16])
            left_pos, msg_l  = fillin_pos(buffer[17:24])

            # joy_r = fillin_joy(buffer[25:32], 'controller_LHR_FF7FBBC0')
            # joy_l = fillin_joy(buffer[33:40], 'controller_LHR_FFFB7FC3')

            joy_r = fillin_joy(buffer[25:33], 'right_buttons')
            joy_l = fillin_joy(buffer[34:42], 'left_buttons')

            # print(buffer)
            # gaze_x = float(buffer[41])
            # gaze_y = float(buffer[42])
            # gaze_z = float(buffer[43])
            # print(gaze_x, gaze_y)

            # chest, msg_c = fillin_pos(buffer[45:52])
            # _, msg_lua = fillin_pos(buffer[52:59])
            # _, msg_rua = fillin_pos(buffer[59:66])
            # _, msg_lla = fillin_pos(buffer[66:73])
            # _, msg_rla = fillin_pos(buffer[73:80])
            # _, msg_waist = fillin_pos(buffer[80:87])

            # print(buffer[85:92])
            # left_pupil = float(buffer[88])
            # right_pupil = float(buffer[89])
            # pupil_data.data = [left_pupil, right_pupil]
            # print(left_pupil, right_pupil)

            # left_blink = float(buffer[91])
            # right_blink = float(buffer[92])
            # blink_data.data = [left_blink, right_blink]
            # print(left_blink, right_blink)

            #pub_twist_r.publish(right_pos)
            pub_pos_r.publish(msg_r)
            #pub_twist_l.publish(left_pos)
            pub_pos_l.publish(msg_l)
            # pub_head.publish(head_tran)
            pub_head.publish(head_pos)
            pub_key_r.publish(joy_r)
            pub_key_l.publish(joy_l)
            # chest_pub.publish(msg_c)
            # lua_pub.publish(msg_lua)
            # rua_pub.publish(msg_rua)
            # lla_pub.publish(msg_lla)
            # rla_pub.publish(msg_rla)
            # waist_pub.publish(msg_waist)

            # print(msg_r)

            # eye_gaze_x.publish(gaze_x)
            # eye_gaze_y.publish(gaze_y)

            # if (numpy.isnan(gaze_x) == False and numpy.isnan(gaze_y) == False):
            #     eye_gaze_x.publish(gaze_x)
            #     eye_gaze_y.publish(gaze_y)
            #     his_x = gaze_x
            #     his_y = gaze_y

            # else:
            #     eye_gaze_x.publish(his_x)
            #     eye_gaze_y.publish(his_y)
                
            # print(his_x, his_y)



            # pupil_pub.publish(pupil_data)
            # blink_pub.publish(blink_data)


            rate.sleep()
        except:
            print('no server detected, reconnecting ' + str(con_cnt))
            con_state = False
            con_cnt += 1
            rate.sleep()
            message = 'request'
            sock.sendto(message.encode(), address)

if __name__ == '__main__':
    address = ("10.0.0.189", 23023)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    message = 'request'
    sock.sendto(message.encode(), address)
    try:
        vive_status_pub()
    except rospy.ROSInterruptException:
        pass
