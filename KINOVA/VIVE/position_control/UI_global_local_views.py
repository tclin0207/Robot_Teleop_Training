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
        self.rgb_img = np.zeros((800, 450, 3), np.uint8) # (960, 540, 3)

        self.cap_rs2 = cv2.VideoCapture(
            'v4l2src device=/dev/video4 ! videoscale ! video/x-raw,width=800,height=450,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        self.out_send = cv2.VideoWriter(
            'appsrc! videoconvert ! video/x-raw,format=YUY2 ! jpegenc! rtpjpegpay ! udpsink host=192.168.0.173 port=2337 sync=false',
            cv2.CAP_GSTREAMER, 0, 25, (800, 450)) # (960, 540)

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
            resized_frame = cv2.resize(frame_rs1, (320, 180), interpolation = cv2.INTER_AREA) # (400, 225)            
            frame_rs2[2:182, 478:798] = resized_frame # [2:227, 558:958]
            cv2.rectangle(frame_rs2, (478,182), (798,2), color_rec, thick_rec) # (558,227), (958,2)

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
