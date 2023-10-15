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
# from kortex_driver.msg import BaseCyclic_Feedback
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Float64

class videoThread(threading.Thread):
    def __init__(self, threadID, name, ip_addr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.t = 0
        self.width = 640
        self.height = 480
        self.rgb_img = np.zeros((480, 640, 3), np.uint8)
        self.cam_switch = 0
        self.sum_test = 0
        self.gamepad_val_x = 0
        self.gamepad_val_y = 0
        self.gamepad_val_z_trans = 0
        self.gamepad_val_z_rot = 0
        # self.cap_rs1 = cv2.VideoCapture(
        #     'v4l2src device=/dev/video6 ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        self.cap_rs2 = cv2.VideoCapture(
            'v4l2src device=/dev/video6 ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
        
        # self.out_send = cv2.VideoWriter(
        #     'appsrc! videoconvert ! video/x-raw,format=YUY2 ! jpegenc! rtpjpegpay ! udpsink host=130.215.181.94 port=2332 sync=false',
        #     cv2.CAP_GSTREAMER, 0, 25, (self.width, self.height))

        self.running = True

        rospy.init_node('cam_test')
        # From Vive ########################
        self.gamepad = rospy.Subscriber('/joy', Joy, self.callback_gamepad, queue_size = 1)
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback_image)
        # From Vive ########################

    def callback_image(self, data):
        # global rgb_img
        self.rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")

    # For Gamepad ########################
    def callback_gamepad(self,data):

        self.gamepad_button = data.buttons
        self.gamepad_axes = data.axes

        self.gamepad_val_x = 25*(self.gamepad_axes[1])
        self.gamepad_val_y = 25*(self.gamepad_axes[0])
        self.gamepad_val_z_trans = 25*(self.gamepad_axes[4])
        self.gamepad_val_z_rot = 25*(self.gamepad_axes[3])


        if self.gamepad_button[1] == 1:
            self.sum_test += 1
        if self.gamepad_button[3] == 1:
            self.cam_switch += 1
    # For Gamepad ########################  

    def run(self):
        
        while not rospy.is_shutdown():
            print(self.gamepad_val_x, self.gamepad_val_y)
            # read the video stream from second realsense camera
            ret_rs2, frame_rs2 = self.cap_rs2.read()
            # frame_rs1 = cv2.resize(self.rgb_img, (640, 480), interpolation = cv2.INTER_AREA)
            frame_rs1 = self.rgb_img

            color_rec = (255,255,255)
            thick_rec = 2
            # resized_frame = cv2.resize(frame_rs1, (640, 480), interpolation = cv2.INTER_AREA)
            if self.cam_switch %2 == 0:
                resized_frame = cv2.resize(frame_rs1, (280, 210), interpolation = cv2.INTER_AREA)            
                frame_rs2[2:212, 358:638] = resized_frame
                cv2.rectangle(frame_rs2, (358,210), (638,0), color_rec, thick_rec)
                if self.sum_test %2 == 0:
                    status_text = 'Trans'
                else:
                    status_text = 'Rot'
                cv2.putText(frame_rs2, status_text, (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA) # orange (30, 105, 210)
                cv2.circle(frame_rs2, (525,450), 25, (255,255,255))
                cv2.circle(frame_rs2, (600,450), 25, (255,255,255))
                cv2.circle(frame_rs2, (525-int(self.gamepad_val_y),450-int(self.gamepad_val_x)), 10, (0,255,255), -1)
                cv2.circle(frame_rs2, (600-int(self.gamepad_val_z_rot),450-int(self.gamepad_val_z_trans)), 10, (0,255,255), -1)
                
                cv2.namedWindow('Viewpoint', cv2.WINDOW_GUI_NORMAL)
                cv2.resizeWindow('Viewpoint', 1280, 960)
                cv2.imshow('Viewpoint', frame_rs2)
            else:
                resized_frame_rs1 = cv2.resize(frame_rs1, (640, 480), interpolation = cv2.INTER_AREA)  
                resized_frame_rs2 = cv2.resize(frame_rs2, (280, 210), interpolation = cv2.INTER_AREA)            
                resized_frame_rs1[2:212, 358:638] = resized_frame_rs2
                cv2.rectangle(resized_frame_rs1, (358,210), (638,0), color_rec, thick_rec)
                if self.sum_test %2 == 0:
                    status_text = 'Trans'
                else:
                    status_text = 'Rot'
                cv2.putText(resized_frame_rs1, status_text, (500, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)
                cv2.circle(resized_frame_rs1, (525,450), 25, (255,255,255))
                cv2.circle(resized_frame_rs1, (600,450), 25, (255,255,255))
                cv2.circle(resized_frame_rs1, (525-int(self.gamepad_val_y),450-int(self.gamepad_val_x)), 10, (0,255,255), -1)
                cv2.circle(resized_frame_rs1, (600-int(self.gamepad_val_z_rot),450-int(self.gamepad_val_z_trans)), 10, (0,255,255), -1)
                
                cv2.namedWindow('Viewpoint', cv2.WINDOW_GUI_NORMAL)
                cv2.resizeWindow('Viewpoint', 1280, 960)
                cv2.imshow('Viewpoint', resized_frame_rs1)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
            # elif key & 0xFF == ord('s'): 
            
        self.running = False
        # self.cap_rs1.release()
        # self.cap_rs2.release()
        # self.out_send.release()
        cv2.destroyAllWindows()
        #threading.Thread.exit()

if __name__ == '__main__':
    ip_addr = '130.215.181.94'

    v_thread = videoThread(0, 'videoT', ip_addr)
    v_thread.start()
