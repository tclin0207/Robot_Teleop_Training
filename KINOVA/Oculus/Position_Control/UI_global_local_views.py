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
from std_msgs.msg import String, Float64

class videoThread(threading.Thread):
    def __init__(self, threadID, name, ip_addr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.rgb_img = np.zeros((480, 640, 3), np.uint8)

        self.cap_rs2 = cv2.VideoCapture(
            'v4l2src device=/dev/video6 ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        self.running = True

        rospy.init_node('user_interface')
        # KINOVA eye-in-hand camera ########################
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback_image)
        # KINOVA eye-in-hand camera ########################

    def callback_image(self, data):
        # global rgb_img
        self.rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")


    def run(self):
        
        while not rospy.is_shutdown():
            
            # read the video stream from realsense camera
            ret_rs2, frame_rs2 = self.cap_rs2.read()
            
            # specify eye-in-hand camera
            frame_rs1 = self.rgb_img

            # create user interface
            color_rec = (255,255,255)
            thick_rec = 2
            resized_frame = cv2.resize(frame_rs1, (280, 210), interpolation = cv2.INTER_AREA)            
            frame_rs2[2:212, 358:638] = resized_frame
            cv2.rectangle(frame_rs2, (358,210), (638,0), color_rec, thick_rec)

            # display user interface            
            cv2.namedWindow('Viewpoint', cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow('Viewpoint', 1280, 960)
            cv2.imshow('Viewpoint', frame_rs2)

            # subscibe keyborad input
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
            
        self.running = False
        self.cap_rs1.release()
        self.cap_rs2.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    ip_addr = '130.215.181.94'

    v_thread = videoThread(0, 'videoT', ip_addr)
    v_thread.start()
