import threading
import cv2
import os
import numpy as np
import rospy
import tf2_ros
import tf.transformations
import math
from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Pose
from std_msgs.msg import String, Float64
from skimage import img_as_float

class videoThread(threading.Thread):
    def __init__(self, threadID, name, ip_addr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.rgb_img = np.zeros((800, 450, 3), np.uint8) # (960, 540, 3)

        self.__tracking_button = 0
        self.__mode_button = 0

        self.controller_x = 0
        self.controller_y = 0
        self.controller_z = 0

        # read controller image with transparent background
        self.controller_img = cv2.imread('vive_controller.png', cv2.IMREAD_UNCHANGED)
        self.controller_img = cv2.resize(self.controller_img, (156, 87), interpolation = cv2.INTER_AREA)

        self.cap_rs2 = cv2.VideoCapture(
            'v4l2src device=/dev/video4 ! videoscale ! video/x-raw,width=800,height=450,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        self.out_send = cv2.VideoWriter(
            'appsrc! videoconvert ! video/x-raw,format=YUY2 ! jpegenc! rtpjpegpay ! udpsink host=192.168.0.173 port=2337 sync=false',
            cv2.CAP_GSTREAMER, 0, 25, (800, 450)) # (960, 540)

        self.running = True

        rospy.init_node('user_interface')
        # KINOVA eye-in-hand camera ########################
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback_image)
        # control state ########################
        rospy.Subscriber('/robot_activation', Float64, self.__tracking_button_callback, queue_size=1)
        rospy.Subscriber('/rotation_activation', Float64, self.__mode_button_callback, queue_size=1)
        # controller feedback
        rospy.Subscriber('/Right_Hand', TransformStamped, self.__input_pose_callback)

    def callback_image(self, data):
        # global rgb_img
        self.rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")

    def __tracking_button_callback(self, data):

        self.__tracking_button = data.data

    def __mode_button_callback(self, data):

        self.__mode_button = data.data

    def __input_pose_callback(self, msg):

        self.controller_x = msg.transform.translation.x
        self.controller_y = msg.transform.translation.y
        self.controller_z = msg.transform.translation.z

    def run(self):
        
        while not rospy.is_shutdown():
            
            # read the video stream from realsense camera
            ret_rs2, frame_rs2 = self.cap_rs2.read()
            
            # specify eye-in-hand camera
            frame_rs1 = self.rgb_img

            # sec view
            color_rec = (255,153,51) # light blue
            thick_rec = 3
            resized_frame = cv2.resize(frame_rs1, (320, 180), interpolation = cv2.INTER_AREA) # (400, 225)            
            frame_rs2[2:182, 478:798] = resized_frame # [2:227, 558:958]
            cv2.rectangle(frame_rs2, (478,182), (798,2), color_rec, thick_rec) # (558,227), (958,2)
            cv2.rectangle(frame_rs2, (476, 215), (800, 183), color_rec, cv2.FILLED) 
            cv2.putText(frame_rs2, 'EYE-IN-HAND CAMERA VIEW', (485, 205), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            # make sure controller is detect
            if self.controller_x == 0 and self.controller_y == 0 and self.controller_z == 0:
                
                frame_rs2_shadow = frame_rs2.copy()
                cv2.rectangle(frame_rs2_shadow, (0,450),(800,0), (50, 50, 50), cv2.FILLED)
                alpha=0.8
                frame_rs2 = cv2.addWeighted(frame_rs2_shadow, alpha, frame_rs2, 1 - alpha, 0)
                cv2.putText(frame_rs2, 'CONNECTING...', (220, 250), cv2.FONT_HERSHEY_SIMPLEX, 2, (100, 100, 255), 5, cv2.LINE_AA)

            else:

                ####### display controller guide #######
                alpha_channel = self.controller_img[:, :, 3] / 255 # convert from 0-255 to 0.0-1.0
                overlay_colors = self.controller_img[:, :, :3]
                alpha_mask = np.dstack((alpha_channel, alpha_channel, alpha_channel))
                h, w = self.controller_img.shape[:2]
                controller_pic_x = 490
                controller_pic_y = 350
                background_subsection = frame_rs2[controller_pic_y:controller_pic_y+h, controller_pic_x:controller_pic_x+w]
                composite = background_subsection * (1 - alpha_mask) + overlay_colors * alpha_mask
                frame_rs2[controller_pic_y:controller_pic_y+h, controller_pic_x:controller_pic_x+w] = composite
                

                ####### display control status #######
                if self.__tracking_button == 2:
                    tracking_text = 'CONTROLLING'
                    tracking_color = (0, 255, 0)
                    ####### display style I #######
                    # cv2.putText(frame_rs2, 'TRANS.', (480, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                    # cv2.rectangle(frame_rs2, (565, 430), (580, 415), (255, 255, 255), 2) 
                    # cv2.rectangle(frame_rs2, (567, 428), (578, 417), (0, 255, 0), cv2.FILLED) 
                    # cv2.putText(frame_rs2, 'ROT.', (610, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                    # cv2.rectangle(frame_rs2, (670, 430), (685, 415), (255, 255, 255), 2)    
                    # if self.__mode_button == 3:
                    #     cv2.rectangle(frame_rs2, (672, 428), (683, 417), (0, 255, 0), cv2.FILLED) 
                    ####### display style II #######
                    # MENU button: rotation control (de)activation
                    cv2.arrowedLine(frame_rs2, (527, 400), (527, 435), (255, 255, 255), 2, tipLength = 0.3)

                    if self.__mode_button == 3:
                        mode_text = 'ON'
                        mode_color = (0, 255, 0)
                        cv2.putText(frame_rs2, 'PRESS TO STOP ROTATE', (540, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 255), 2, cv2.LINE_AA)
                    else:
                        mode_text = 'OFF'
                        mode_color = (100, 100, 255)
                        # MENU button: rotation control (de)activation
                        cv2.putText(frame_rs2, 'PRESS TO ROTATE', (540, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)

                    cv2.putText(frame_rs2, 'ROTATION:', (490, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(frame_rs2, mode_text, (605, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2, cv2.LINE_AA)
                    # GRIP button: start(pause) robot control
                    cv2.arrowedLine(frame_rs2, (579, 350), (579, 370), (255, 255, 255), 2, tipLength = 0.3)
                    cv2.putText(frame_rs2, 'SQUEEZE TO PAUSE', (587, 365), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 255), 2, cv2.LINE_AA)

                else:
                    tracking_text = 'PAUSE'
                    tracking_color = (100, 100, 255)   
                    cv2.arrowedLine(frame_rs2, (579, 350), (579, 370), (255, 255, 255), 2, tipLength = 0.3)     
                    cv2.arrowedLine(frame_rs2, (579, 434), (579, 414), (255, 255, 255), 2, tipLength = 0.3)
                    cv2.putText(frame_rs2, 'SQUEEZE TO START', (587, 365), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        
                cv2.putText(frame_rs2, 'ROBOT:', (490, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.putText(frame_rs2, tracking_text, (575, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.7, tracking_color, 2, cv2.LINE_AA)         
                # status monitor
                cv2.rectangle(frame_rs2, (478,448), (798,268), tracking_color, 3)
            

            # send out the visual feedback to unity
            self.out_send.write(frame_rs2)

            # display user interface            
            cv2.namedWindow('Viewpoint', cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow('Viewpoint', 960, 540)
            cv2.imshow('Viewpoint', frame_rs2)

            # subscibe keyborad input
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
            
        self.running = False
        # self.cap_rs1.release()
        # self.cap_rs2.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    ip_addr = '130.215.181.94'

    v_thread = videoThread(0, 'videoT', ip_addr)
    v_thread.start()
