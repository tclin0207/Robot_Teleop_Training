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
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float64, Bool

class videoThread(threading.Thread):
    def __init__(self, threadID, name, ip_addr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.rgb_img = np.zeros((480, 640, 3), np.uint8)

        self.__tracking_button = 0
        self.__mode_button = 0

        self.cap_rs2 = cv2.VideoCapture(
            'v4l2src device=/dev/video4 ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        self.running = True

        rospy.init_node('user_interface')
        # KINOVA eye-in-hand camera ########################
        # rospy.Subscriber('/camera/color/image_raw', Image, self.callback_image)
        # Oculus buttons feedback ########################
        rospy.Subscriber('/robot_activation', Float64, self.__tracking_button_callback, queue_size=1)
        rospy.Subscriber('/rotation_activation', Float64, self.__mode_button_callback, queue_size=1)

    # def callback_image(self, data):
    #     # global rgb_img
    #     self.rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")

    def __tracking_button_callback(self, data):

        self.__tracking_button = data.data

    def __mode_button_callback(self, data):

        self.__mode_button = data.data

    def run(self):
        
        while not rospy.is_shutdown():
            
            # read the video stream from realsense camera
            ret_rs2, frame_rs2 = self.cap_rs2.read()
            
            # specify eye-in-hand camera
            # frame_rs1 = self.rgb_img

            # create user interface
            # color_rec = (255,255,255)
            # thick_rec = 2
            # resized_frame = cv2.resize(frame_rs1, (280, 210), interpolation = cv2.INTER_AREA)            
            # frame_rs2[2:212, 358:638] = resized_frame
            # cv2.rectangle(frame_rs2, (358,210), (638,0), color_rec, thick_rec)

            ####### display control status #######
            if self.__tracking_button == 2:
                tracking_text = 'ACTIVATED'
                tracking_color = (0, 255, 0)
                cv2.putText(frame_rs2, 'TRANS.', (365, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.rectangle(frame_rs2, (450, 460), (465, 445), (255, 255, 255), 2) 
                cv2.rectangle(frame_rs2, (452, 458), (463, 447), (0, 255, 0), cv2.FILLED) 
                cv2.putText(frame_rs2, 'ROT.', (495, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                cv2.rectangle(frame_rs2, (555, 460), (570, 445), (255, 255, 255), 2)    
                if self.__mode_button == 3:
                    cv2.rectangle(frame_rs2, (557, 458), (568, 447), (0, 255, 0), cv2.FILLED) 

            else:
                tracking_text = 'READY'
                tracking_color = (255, 255, 255)        
            
            # if self.__mode_button == 3:
            #     mode_text = 'ROTATION ON'
            #     mode_color = (0, 128, 0)
            # else:
            #     mode_text = 'ROTATION OFF'
            #     mode_color = (255, 255, 255)

            # cv2.putText(frame_rs2, mode_text, (450, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2, cv2.LINE_AA)
            cv2.putText(frame_rs2, 'ROBOT:', (365, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(frame_rs2, tracking_text, (450, 430), cv2.FONT_HERSHEY_SIMPLEX, 0.7, tracking_color, 2, cv2.LINE_AA)
            

            # display user interface            
            cv2.namedWindow('Viewpoint', cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow('Viewpoint', 1280, 960)
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
