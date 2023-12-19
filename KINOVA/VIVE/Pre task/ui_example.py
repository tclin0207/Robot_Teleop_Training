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
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped, Pose
# from kortex_driver.msg import BaseCyclic_Feedback
import json
import copy
from argparse import ArgumentParser
import cv2.aruco as aruco

vive_base_button_b = (0, 0, 0, 0)
vive_base_axes_b = (0, 0, 0)


class videoThread(threading.Thread):
    def __init__(self, threadID, name, ip_addr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.t = 0
        self.width = 640
        self.height = 480
        self.obj_marker = {'205':[0,0]}
        self.scanning_marker = {'207':[0,0]}
        # self.camera_x = 0
        # self.camera_y = 0
        # self.camera_z = 0
        # self.camera_x_2 = 0
        # self.camera_y_2 = 0
        # self.camera_z_2 = 0
        # self.c_b_x = 0
        # self.c_b_y = 0
        # self.c_b_z = 0
        # self.c_b_x_2 = 0
        # self.c_b_y_2 = 0
        # self.c_b_z_2 = 0
        # self.pixel_x = 0
        # self.pixel_y = 0
        # self.pixel_x_2 = 0
        # self.pixel_y_2 = 0
        # self.height = 200
        # self.flagg = 0
        # self.vive_home = 0
        # self.vive_stop = 0
        # self.gaze_x = 0
        # self.gaze_y = 0
        # self.rec_100 = 0
        # self.gripper_current = 0
        # self.gripper_position = 0  # fully open
        # self.vive_base_button_b = (0, 0, 0, 0)
        # self.vive_base_axes_b = (0, 0, 0)
        # self.rgb_img = np.zeros((480, 640, 3), np.uint8)
        # self.rgb_img_1 = np.zeros((480, 640, 3), np.uint8)
        # self.rgb_img_2 = np.zeros((480, 640, 3), np.uint8)
        # self.circle_coord = (0, 0)
        # self.pause = 0
        # self.ee = TransformStamped()
        # self.vive_home_again = 0

        # self.cap_rs1 = cv2.VideoCapture(
        #     'v4l2src device=/dev/video6 ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        # self.cap_rs2 = cv2.VideoCapture(
        #     'v4l2src device=/dev/video5 ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink',
        #     cv2.CAP_GSTREAMER)

        self.out_send = cv2.VideoWriter(
            'appsrc! videoconvert ! video/x-raw,format=YUY2 ! jpegenc! rtpjpegpay ! udpsink host=130.215.181.94 port=2332 sync=false',
            cv2.CAP_GSTREAMER, 0, 25, (640, 480))

        self.running = True

        rospy.init_node('wearable_cam')
        # From Vive ########################
        # self.vive_b = rospy.Subscriber('/vive/controller_LHR_FF7FBBC0/joy', Joy, self.callback_vive_b, queue_size=1)
        rospy.Subscriber('/camera/color/image_raw', Image, self.callback_image)
        # rospy.Subscriber('/ee_loc', TransformStamped, self.callback_ee)
        # rospy.Subscriber('/aruco_simple/pose2', Pose, self.callback_camera)
        # rospy.Subscriber('/aruco_simple/pose', Pose, self.callback_camera_marker2)
        # self.eye_x = rospy.Subscriber('/Gaze_X', Float64, self.callback_gaze_x, queue_size=1)
        # self.eye_y = rospy.Subscriber('/Gaze_Y', Float64, self.callback_gaze_y, queue_size=1)
        # self.main_image = rospy.Publisher('/main_detection', String, queue_size=1)
        # self.ar_image = rospy.Publisher('/ar_detection', String, queue_size=1)
        # rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, self.callback_motor)
        # self.ee_loc = rospy.Publisher('/EE', Float64MultiArray, queue_size=1)
        # self.button_press = rospy.Publisher('/button_press', String, queue_size=1)
        # self.target_pub = rospy.Publisher('/target_indicator', Float64, queue_size=1)
        # From Vive ########################

    # def circle_gaze(self, ee_loc, gaze):

    #     value = abs(np.linalg.norm(np.asarray(gaze) - np.asarray(ee_loc)))

    #     if value <= 50:

    #         gaze_value = 1

    #     else:

    #         gaze_value = 0

    #     return gaze_value

    def detect_markers(self, out, id):

        try:

            aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL) 
            arucoParameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(out, aruco_dict, parameters=arucoParameters)
            # print(ids)
            value = np.where(ids==id)
            corner = corners[value[0][0]]
            mid_x_ar = int((corner[0][0][0]+corner[0][1][0])/2)
            mid_y_ar = int((corner[0][0][1]+corner[0][2][1])/2)

        except:
            return 0, 0, 0

        return mid_x_ar, mid_y_ar, corner

    def dist_check(self, corners):
        
        return_val = 0
        try:

            if abs(corners[0][0][1] - corners[0][2][1]) > 100:

                return_val = 1
            
            elif abs(corners[0][1][0] - corners[0][3][0]) > 100:

                return_val = 1
        except:
            return 0
        return return_val
    
    # def id_loc(self, id):

    #     try:
    #         value = np.where(ids=id)
    #         corner = corners[value[0][0]]
    #         mid_x_ar = int((corner[0][0][0]+corner[0][1][0])/2)
    #         mid_y_ar = int((corner[0][0][1]+corner[0][2][1])/2)
    #     except:
    #         return 0,0
    #     return mid_x_ar,mid_y_ar
    
    # def callback_ee(self, data):

    #     # print(data)
    #     self.ee = data

    # def callback_motor(self, data):
    #     # global gripper_current, gripper_position
    #     self.gripper_current = float(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].current_motor)
    #     self.gripper_position = float(data.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0].position)


    # def area_gaze(self, bottom_left, top_right, gaze):

    #     pip_presence = 0

    #     if gaze[0] <= top_right[0] and gaze[0] >= bottom_left[0]:
    #         if gaze[1] >= top_right[1] and gaze[1] <= bottom_left[1]:
    #             pip_presence = 1

    #     return  pip_presence

    # def aruco_gaze_main(self, gaze, marker):

    #     if abs(marker[0] - gaze[0]) <= 50 and abs(marker[1] - gaze[1]) <= 50:
    #         obj_presence = 1
    #     else:
    #         obj_presence = 0

    #     return obj_presence

    # def aruco_gaze_pip(self, gaze, marker):

    #     if abs(marker[0] - gaze[0]) <= 10 and abs(marker[1] - gaze[1]) <= 10:
    #         obj_presence = 1
    #     else:
    #         obj_presence = 0

    #     return obj_presence

    # def coord_correction(self, bottom_left):

    #     if bottom_left[0] < 0:
    #         bottom_left[0] = 15
    #     if bottom_left[0] > 640:
    #         bottom_left[0] = 625
    #     if bottom_left[1] < 0:
    #         bottom_left[1] = 15
    #     if bottom_left[1] >480:
    #         bottom_left[1] = 465

    #     return bottom_left


    # def aruco_detect(self, x, y, z):

    #     try:
    #         pixel_x = (611.055419921875 * x / z) + 316.6255187988281
    #         pixel_y = (611.2431030273438 * y / z) + 229.1320343017578
    #     except:
    #         return 0, 0

    #     return pixel_x, pixel_y


    # def callback_gaze_x(self, data):

    #     self.gaze_x = data.data
    #     # print(self.gaze_x)

    # def callback_gaze_y(self, data):

    #     self.gaze_y = data.data
    #     # print(self.gaze_y)

    def callback_image(self, data):
        # global rgb_img
        self.rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")

    # def callback_vive_b(self, data):

    #     self.vive_base_button_b = data.buttons  # vive_base_button_b[2] will be 1 if press trackpad or side button ...
    #     self.vive_base_axes_b = data.axes

    #     gripper_val = self.vive_base_axes_b[2]

    #     if gripper_val == 1:  # Trigger button to hold the gripper state
    #         self.rec_100 += 1
    #         rospy.sleep(0.5)

    #     if self.vive_base_button_b[2] == 1:  # Side button to start control
    #         self.flagg = 1
    #         self.pause += 1
    #         rospy.sleep(0.5)
    #         # print("started")

    #     if self.vive_base_button_b[0] == 1:
    #         self.vive_home += 1
    #         # print("home", vive_home)
    #         rospy.sleep(0.5)

    #     if self.vive_base_button_b[2] == 1 and self.vive_base_axes_b[0] == 0:  # Side button as the stop button
    #         # if vive_home % 2 == 0 and vive_home != 0:
    #         self.vive_stop += 1
    #         # print("pause", self.vive_stop)
    #         rospy.sleep(0.5)

    # def callback_camera(self, data):
    #     # global camera_x, camera_y, camera_z
    #     # global pixel_x, pixel_y
    #     # global c_b_x, c_b_y, c_b_z

    #     self.camera_x = data.position.x
    #     self.camera_y = data.position.y
    #     self.camera_z = data.position.z

    #     self.c_b_x = 0.5 * self.camera_z - 0.866 * self.camera_y - 0.1
    #     self.c_b_y = -self.camera_x + 0.35
    #     self.c_b_z = -0.866 * self.camera_z - 0.5 * self.camera_y + 0.85

    #     # ------- transfer from webcam coordinate to image plane -------
    #     self.pixel_x = (
    #                            611.055419921875 * self.camera_x / self.camera_z) + 316.6255187988281  # get it by $rostopic echo /camera/color/camera_info
    #     self.pixel_y = (
    #                            611.2431030273438 * self.camera_y / self.camera_z) + 229.1320343017578  # get it by $rostopic echo /camera/color/camera_info
    #     # ------- ---------------------------------------------- -------

    # def callback_camera_marker2(self, data):
    #     # global camera_x_2, camera_y_2, camera_z_2
    #     # global pixel_x_2, pixel_y_2
    #     # global c_b_x_2, c_b_y_2, c_b_z_2

    #     self.camera_x_2 = data.position.x
    #     self.camera_y_2 = data.position.y
    #     self.camera_z_2 = data.position.z

    #     self.c_b_x_2 = 0.5 * self.camera_z_2 - 0.866 * self.camera_y_2 - 0.1
    #     self.c_b_y_2 = -self.camera_x_2 + 0.35
    #     self.c_b_z_2 = -0.866 * self.camera_z_2 - 0.5 * self.camera_y_2 + 0.85

    #     # ------- transfer from webcam coordinate to image plane -------
    #     self.pixel_x_2 = (
    #                              611.055419921875 * self.camera_x_2 / self.camera_z_2) + 316.6255187988281  # get it by $rostopic echo /camera/color/camera_info
    #     self.pixel_y_2 = (
    #                              611.2431030273438 * self.camera_y_2 / self.camera_z_2) + 229.1320343017578  # get it by $rostopic echo /camera/color/camera_info
        # ------- ---------------------------------------------- -------

    def run(self):
        # global vive_base_button_b, vive_base_axes_b
        # print(vive_base_button_b[2])

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rospy.sleep(1)

        # flag = 0
        # zoom_in = 0
        # zoom_out = 0
        # shift_x = 0
        # shift_y = 0
        rate = rospy.Rate(30)

        # old_mid_x_201 = 0
        # old_mid_y_201 = 0
        # old_mid_x_202 = 0
        # old_mid_y_202 = 0
        # old_mid_x_203 = 0
        # old_mid_y_203 = 0
        # old_mid_x_204 = 0
        # old_mid_y_204 = 0
        # old_mid_x_205 = 0
        # old_mid_y_205 = 0
        # old_mid_x_206 = 0
        # old_mid_y_206 = 0
        # old_mid_x_207 = 0
        # old_mid_y_207 = 0
        # old_mid_x_208 = 0
        # old_mid_y_208 = 0
        # old_mid_x_209 = 0
        # old_mid_y_209 = 0
        # old_mid_x_210 = 0
        # old_mid_y_210 = 0
        # old_mid_x_211 = 0
        # old_mid_y_211 = 0
        # old_mid_x_212 = 0
        # old_mid_y_212 = 0
        # old_mid_x_213 = 0
        # old_mid_y_213 = 0
        # old_mid_x_214 = 0
        # old_mid_y_214 = 0
        # old_mid_x_215 = 0
        # old_mid_y_215 = 0
        # old_mid_x_216 = 0
        # old_mid_y_216 = 0
        
        # ee = Float64MultiArray()
        
        # ------- video record ------- #
        # parser = ArgumentParser()
        # parser.add_argument('filename', metavar='N', type=str, nargs='+',
        #             help='an integer for the accumulator')
        # args = parser.parse_args()
        # name_of_file = args.filename[-1]
        
        # video = cv2.VideoWriter(name_of_file + 'video.avi', cv2.VideoWriter_fourcc(*'MJPG'), 25, (640, 480))
        # ------- video record ------- #
        
        # pause = 0

        while not rospy.is_shutdown():

            # read the video stream from second realsense camera
            # ret_rs2, frame_rs2 = self.cap_rs2.read()
            
            # frame_rs1 = cv2.resize(self.rgb_img, (640, 480), interpolation = cv2.INTER_AREA)
            frame_rs1 = self.rgb_img
            main_img = copy.deepcopy(frame_rs1)
            # obj1_main_detect = 0

            mid_x_ar, mid_y_ar, corners = self.detect_markers(main_img, 207)

            # if isinstance(corners, np.ndarray):
            #     print(corners[0][3])

            print_ = self.dist_check(corners)
            if mid_x_ar != 0:
                if print_:
                    cv2.circle(main_img, (int(mid_x_ar), int(mid_y_ar)), 20, (51,51,255), cv2.FILLED)

            mid_x_ar, mid_y_ar, corners = self.detect_markers(main_img, 205)

            # if isinstance(corners, np.ndarray):
            #     print(corners[0][3])

            print_ = self.dist_check(corners)
            if mid_x_ar != 0:
                if print_:
                    cv2.circle(main_img, (int(mid_x_ar), int(mid_y_ar)), 20, (51,51,255), cv2.FILLED)
            
            mid_x_ar, mid_y_ar, corners = self.detect_markers(main_img, 202)

            # if isinstance(corners, np.ndarray):
            #     print(corners[0][3])

            print_ = self.dist_check(corners)
            if mid_x_ar != 0:
                if print_:
                    cv2.circle(main_img, (int(mid_x_ar), int(mid_y_ar)), 20, (51,51,255), cv2.FILLED)

            # # get the robot end effector w.r.t. image plane: (pixel_x_r, pixel_y_r)
            # # robot_ee = tfBuffer.lookup_transform('base_link', 'robotiq_arg2f_base_link', rospy.Time())
            # # # ------- transfer from robot base frame to webcam coordinate -------
            # ee_x = self.ee.transform.translation.x
            # ee_y = self.ee.transform.translation.y
            # ee_z = self.ee.transform.translation.z
            # # ee.data = [ee_x, ee_y, ee_z]
            # ee_x_new = round(ee_x,2)
            # ee_y_new = round(ee_y,2)
            # ee_z_new = round(ee_z,2)
            
            # ------- Display Robot Status ------- #
            # if self.vive_home == 0:
            #     status_text = 'WAITING'
            # elif self.vive_home % 2 == 1:
            #     status_text = 'HOME'
            # elif self.vive_home % 4 == 2 and self.vive_home != 1:
            #     status_text = 'READY'

            # if self.flagg == 1:
            #     status_text = 'TELEOP'
            # if self.vive_stop > 0 and self.vive_stop % 2 == 0:
            #     pause += 1
            #     status_text = 'PAUSE'
            #     self.vive_home_again = 0
            #     if self.vive_base_button_b[0] == 1:
            #         self.vive_home_again += 1
            #         self.flagg = 0
            #         self.vive_stop = 0
            #         rospy.sleep(0.5)
            
            # if self.vive_home_again == 1 and self.flagg == 0:
            #     status_text = 'HOME'
            # # print(self.vive_home_again)


            # cv2.putText(frame_rs1, status_text, (500, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3, cv2.LINE_AA)
            # frame_zoom = frame_rs1[0:720, 0+160:1280-160]
            # resized_frame_rs1 = cv2.resize(frame_zoom, (640, 480), interpolation=cv2.INTER_AREA)
            # out = resized_frame_rs1
            # out = frame_rs1.copy()

            # ------- Aruco tags ------- #
            # try:
            #     mid_x_ar_201, mid_y_ar_201 = self.detect_markers(out, 201, old_mid_x_201, old_mid_y_201) # red object 1
            #     mid_x_ar_202, mid_y_ar_202 = self.detect_markers(out, 202, old_mid_x_202, old_mid_y_202) # red box 
            #     mid_x_ar_203, mid_y_ar_203 = self.detect_markers(out, 203, old_mid_x_203, old_mid_y_203) # green object 1
            #     mid_x_ar_204, mid_y_ar_204 = self.detect_markers(out, 204, old_mid_x_204, old_mid_y_204) # green box
            #     mid_x_ar_205, mid_y_ar_205 = self.detect_markers(out, 205, old_mid_x_205, old_mid_y_205) # yellow object 1
            #     mid_x_ar_206, mid_y_ar_206 = self.detect_markers(out, 206, old_mid_x_206, old_mid_y_206) # yellow box
            #     mid_x_ar_207, mid_y_ar_207 = self.detect_markers(out, 207, old_mid_x_207, old_mid_y_207) # red object 2
            #     mid_x_ar_208, mid_y_ar_208 = self.detect_markers(out, 208, old_mid_x_208, old_mid_y_208) # yellow object 2
            #     mid_x_ar_209, mid_y_ar_209 = self.detect_markers(out, 209, old_mid_x_209, old_mid_y_209) # green object 2
            #     mid_x_ar_210, mid_y_ar_210 = self.detect_markers(out, 210, old_mid_x_210, old_mid_y_210) # blue object 1
            #     mid_x_ar_211, mid_y_ar_211 = self.detect_markers(out, 211, old_mid_x_211, old_mid_y_211) # location 1
            #     mid_x_ar_212, mid_y_ar_212 = self.detect_markers(out, 212, old_mid_x_212, old_mid_y_212) # location 2
            #     mid_x_ar_213, mid_y_ar_213 = self.detect_markers(out, 213, old_mid_x_213, old_mid_y_213) # location 3
            #     mid_x_ar_214, mid_y_ar_214 = self.detect_markers(out, 214, old_mid_x_214, old_mid_y_214) # location 4
            #     mid_x_ar_215, mid_y_ar_215 = self.detect_markers(out, 215, old_mid_x_215, old_mid_y_215) # location 5
            #     mid_x_ar_216, mid_y_ar_216 = self.detect_markers(out, 216, old_mid_x_216, old_mid_y_216) # location 6
            #     old_mid_x_201 = mid_x_ar_201
            #     old_mid_y_201 = mid_y_ar_201
            #     old_mid_x_202 = mid_x_ar_202
            #     old_mid_y_202 = mid_y_ar_202
            #     old_mid_x_203 = mid_x_ar_203
            #     old_mid_y_203 = mid_y_ar_203
            #     old_mid_x_204 = mid_x_ar_204
            #     old_mid_y_204 = mid_y_ar_204
            #     old_mid_x_205 = mid_x_ar_205
            #     old_mid_y_205 = mid_y_ar_205
            #     old_mid_x_206 = mid_x_ar_206
            #     old_mid_y_206 = mid_y_ar_206
            #     old_mid_x_207 = mid_x_ar_207
            #     old_mid_y_207 = mid_y_ar_207
            #     old_mid_x_208 = mid_x_ar_208
            #     old_mid_y_208 = mid_y_ar_208
            #     old_mid_x_209 = mid_x_ar_209
            #     old_mid_y_209 = mid_y_ar_209
            #     old_mid_x_210 = mid_x_ar_210
            #     old_mid_y_210 = mid_y_ar_210
            #     old_mid_x_211 = mid_x_ar_211
            #     old_mid_y_211 = mid_y_ar_211
            #     old_mid_x_212 = mid_x_ar_212
            #     old_mid_y_212 = mid_y_ar_212
            #     old_mid_x_213 = mid_x_ar_213
            #     old_mid_y_213 = mid_y_ar_213
            #     old_mid_x_214 = mid_x_ar_214
            #     old_mid_y_214 = mid_y_ar_214
            #     old_mid_x_215 = mid_x_ar_215
            #     old_mid_y_215 = mid_y_ar_215
            #     old_mid_x_216 = mid_x_ar_216
            #     old_mid_y_216 = mid_y_ar_216
            # except:
            #     mid_x_ar_201 = old_mid_x_201
            #     mid_y_ar_201 = old_mid_y_201
            #     mid_x_ar_202 = old_mid_x_202
            #     mid_y_ar_202 = old_mid_y_202
            #     mid_x_ar_203 = old_mid_x_203
            #     mid_y_ar_203 = old_mid_y_203
            #     mid_x_ar_204 = old_mid_x_204
            #     mid_y_ar_204 = old_mid_y_204
            #     mid_x_ar_205 = old_mid_x_205
            #     mid_y_ar_205 = old_mid_y_205
            #     mid_x_ar_206 = old_mid_x_206
            #     mid_y_ar_206 = old_mid_y_206
            #     mid_x_ar_207 = old_mid_x_207
            #     mid_y_ar_207 = old_mid_y_207
            #     mid_x_ar_208 = old_mid_x_208
            #     mid_y_ar_208 = old_mid_y_208
            #     mid_x_ar_209 = old_mid_x_209
            #     mid_y_ar_209 = old_mid_y_209
            #     mid_x_ar_210 = old_mid_x_210
            #     mid_y_ar_210 = old_mid_y_210
            #     mid_x_ar_211 = old_mid_x_211
            #     mid_y_ar_211 = old_mid_y_211
            #     mid_x_ar_212 = old_mid_x_212
            #     mid_y_ar_212 = old_mid_y_212
            #     mid_x_ar_213 = old_mid_x_213
            #     mid_y_ar_213 = old_mid_y_213
            #     mid_x_ar_214 = old_mid_x_214
            #     mid_y_ar_214 = old_mid_y_214
            #     mid_x_ar_215 = old_mid_x_215
            #     mid_y_ar_215 = old_mid_y_215
            #     mid_x_ar_216 = old_mid_x_216
            #     mid_y_ar_216 = old_mid_y_216
            # cv2.circle(out, (int(mid_x_ar_201), int(mid_y_ar_201)), 20, (51,51,255), 3) # red 1
            # cv2.circle(out, (int(mid_x_ar_203), int(mid_y_ar_203)), 20, (0,255,128), 3) # green 1
            # cv2.circle(out, (int(mid_x_ar_205), int(mid_y_ar_205)), 20, (51,255,255), 3) # yellow 1
            # cv2.circle(out, (int(mid_x_ar_207), int(mid_y_ar_207)), 20, (51,51,255), 3) # red 2
            # cv2.circle(out, (int(mid_x_ar_208), int(mid_y_ar_208)), 20, (0,255,128), 3) # green 2
            # cv2.circle(out, (int(mid_x_ar_209), int(mid_y_ar_209)), 20, (51,255,255), 3) # yellow 2
            # cv2.circle(out, (int(mid_x_ar_210), int(mid_y_ar_210)), 20, (255,0,0), 3) # blue 1
            
            # cv2.circle(out, (int(mid_x_ar_202), int(mid_y_ar_202)), 50, (51,51,255), 3) # red box 
            # cv2.circle(out, (int(mid_x_ar_204), int(mid_y_ar_204)), 50, (0,255,128), 3) # green box
            # cv2.circle(out, (int(mid_x_ar_206), int(mid_y_ar_206)), 50, (51,255,255), 3) # yellow box

            # cv2.circle(out, (int(mid_x_ar_211), int(mid_y_ar_211)), 50, (255,255,255), 3) # location 1
            # cv2.circle(out, (int(mid_x_ar_212), int(mid_y_ar_212)), 50, (255,255,255), 3) # location 2
            # cv2.circle(out, (int(mid_x_ar_213), int(mid_y_ar_213)), 50, (255,255,255), 3) # location 3
            # cv2.circle(out, (int(mid_x_ar_214), int(mid_y_ar_214)), 50, (255,255,255), 3) # location 4
            # cv2.circle(out, (int(mid_x_ar_215), int(mid_y_ar_215)), 50, (255,255,255), 3) # location 5
            # cv2.circle(out, (int(mid_x_ar_216), int(mid_y_ar_216)), 50, (255,255,255), 3) # location 6

            # ------- AR for depth ------- #
            # print(self.gripper_position)
            # if self.gripper_position <= 55 and ee_z <= 0.08:
            #     if ee_x >= 0.4 and ee_x <= 0.55 and ee_y >= 0.19 and ee_y <= 0.29: # red object
            #         red_obj = ((1-ee_x)*100-53)*10
            #         cv2.line(out, (mid_x_ar_201-20,mid_y_ar_201+20), (mid_x_ar_201-20,mid_y_ar_201-40) ,(255,255,255), 2)
            #         cv2.line(out, (mid_x_ar_201+20,mid_y_ar_201+20), (mid_x_ar_201+20,mid_y_ar_201-40) ,(255,255,255), 2)
            #         cv2.putText(out, 'F', (mid_x_ar_201-5, mid_y_ar_201-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         cv2.putText(out, 'B', (mid_x_ar_201-5, mid_y_ar_201+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         if ee_x >= 0.46 and ee_x <= 0.49 and ee_z <= 0.02:
            #             red_obj_color = (0,0,255)
            #         else:
            #             red_obj_color = (255,255,255)
            #         cv2.line(out, (mid_x_ar_201-20,int(mid_y_ar_201+red_obj)), (mid_x_ar_201+20,int(mid_y_ar_201+red_obj)) ,red_obj_color, 2)
            #     if ee_x >= 0.03 and ee_x <= 0.18 and ee_y >= 0.23 and ee_y <= 0.34: # green object
            #         green_obj = ((1-ee_x)*100-90)*10
            #         cv2.line(out, (mid_x_ar_203-20,mid_y_ar_203+60), (mid_x_ar_203-20,mid_y_ar_203-20) ,(255,255,255), 2)
            #         cv2.line(out, (mid_x_ar_203+20,mid_y_ar_203+60), (mid_x_ar_203+20,mid_y_ar_203-20) ,(255,255,255), 2)
            #         cv2.putText(out, 'F', (mid_x_ar_203-5, mid_y_ar_203-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         cv2.putText(out, 'B', (mid_x_ar_203-5, mid_y_ar_203+70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         if ee_x >= 0.09 and ee_x <= 0.12 and ee_z <= 0.03:
            #             green_obj_color = (0,255,128)
            #         else:
            #             green_obj_color = (255,255,255)
            #         cv2.line(out, (mid_x_ar_203-20,int(mid_y_ar_203+green_obj)), (mid_x_ar_203+20,int(mid_y_ar_203+green_obj)) ,green_obj_color, 2)
            #     if ee_x >= 0.1 and ee_x <= 0.25 and ee_y >= 0.42 and ee_y <= 0.53: # yellow object
            #         yellow_obj = ((1-ee_x)*100-82)*10
            #         cv2.line(out, (mid_x_ar_205-20,mid_y_ar_205+60), (mid_x_ar_205-20,mid_y_ar_205-20) ,(255,255,255), 2)
            #         cv2.line(out, (mid_x_ar_205+20,mid_y_ar_205+60), (mid_x_ar_205+20,mid_y_ar_205-20) ,(255,255,255), 2)
            #         cv2.putText(out, 'F', (mid_x_ar_205-5, mid_y_ar_205-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         cv2.putText(out, 'B', (mid_x_ar_205-5, mid_y_ar_205+70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         if ee_x >= 0.16 and ee_x <= 0.19 and ee_z <= 0.03:
            #             yellow_obj_color = (51,255,255)
            #         else:
            #             yellow_obj_color = (255,255,255)
            #         cv2.line(out, (mid_x_ar_205-20,int(mid_y_ar_205+yellow_obj)), (mid_x_ar_205+20,int(mid_y_ar_205+yellow_obj)) ,yellow_obj_color, 2)

            # elif self.gripper_position > 55 and ee_z <= 0.15:
            #     if ee_x >= 0.14 and ee_x <= 0.29 and ee_y >= 0.27 and ee_y <= 0.38: # red box
            #         red_box = ((1-ee_x)*100-77)*18
            #         cv2.line(out, (mid_x_ar_202-60,mid_y_ar_202+85), (mid_x_ar_202-60,mid_y_ar_202-20) ,(255,255,255), 2)
            #         cv2.line(out, (mid_x_ar_202+60,mid_y_ar_202+85), (mid_x_ar_202+60,mid_y_ar_202-20) ,(255,255,255), 2)
            #         cv2.putText(out, 'F', (mid_x_ar_202-5, mid_y_ar_202-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         cv2.putText(out, 'B', (mid_x_ar_202-5, mid_y_ar_202+90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         if ee_x >= 0.2 and ee_x <= 0.23 and ee_z <= 0.1:
            #             red_box_color = (0,0,255)
            #         else:
            #             red_box_color = (255,255,255)
            #         cv2.line(out, (mid_x_ar_202-60,int(mid_y_ar_202+red_box)), (mid_x_ar_202+60,int(mid_y_ar_202+red_box)) ,red_box_color, 2)
            #     if ee_x >= 0.33 and ee_x <= 0.49 and ee_y >= 0.37 and ee_y <= 0.48: # green box
            #         green_box = ((1-ee_x)*100-58)*15
            #         cv2.line(out, (mid_x_ar_204-50,mid_y_ar_204+75), (mid_x_ar_204-50,mid_y_ar_204-20) ,(255,255,255), 2)
            #         cv2.line(out, (mid_x_ar_204+50,mid_y_ar_204+75), (mid_x_ar_204+50,mid_y_ar_204-20) ,(255,255,255), 2)
            #         cv2.putText(out, 'F', (mid_x_ar_204-5, mid_y_ar_204-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         cv2.putText(out, 'B', (mid_x_ar_204-5, mid_y_ar_204+80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         if ee_x >= 0.39 and ee_x <= 0.43 and ee_z <= 0.1:
            #             green_box_color = (0,255,128)
            #         else:
            #             green_box_color = (255,255,255)
            #         cv2.line(out, (mid_x_ar_204-50,int(mid_y_ar_204+green_box)), (mid_x_ar_204+50,int(mid_y_ar_204+green_box)) ,green_box_color, 2)
            #     if ee_x >= 0.21 and ee_x <= 0.37 and ee_y >= 0.05 and ee_y <= 0.17: # yellow box
            #         yellow_box = ((1-ee_x)*100-70)*15
            #         cv2.line(out, (mid_x_ar_206-50,mid_y_ar_206+85), (mid_x_ar_206-50,mid_y_ar_206-20) ,(255,255,255), 2)
            #         cv2.line(out, (mid_x_ar_206+70,mid_y_ar_206+85), (mid_x_ar_206+70,mid_y_ar_206-20) ,(255,255,255), 2)
            #         cv2.putText(out, 'F', (mid_x_ar_206-5, mid_y_ar_206-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         cv2.putText(out, 'B', (mid_x_ar_206-5, mid_y_ar_206+90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
            #         if ee_x >= 0.27 and ee_x <= 0.31 and ee_z <= 0.1:
            #             yellow_box_color = (51,255,255)
            #         else:
            #             yellow_box_color = (255,255,255)
            #         cv2.line(out, (mid_x_ar_206-50,int(mid_y_ar_206+yellow_box)), (mid_x_ar_206+70,int(mid_y_ar_206+yellow_box)) ,yellow_box_color, 2)
            # ------- AR for depth ------- #

            # ------- Secondary View ------- #
            # out_w_sec = out.copy()
            # frame_zoom = frame_rs2[0+72:480-72, 0+96:640-96]
            # resized_frame_zoom = cv2.resize(frame_zoom, (640, 480), interpolation=cv2.INTER_AREA)
            # resized_frame = cv2.resize(resized_frame_zoom, (200, 150), interpolation=cv2.INTER_AREA)
            # out[2:152, 2:202] = resized_frame
            # cv2.rectangle(out, (2, 152), (202, 2), (255, 255, 255), 2)
            # resized_frame = cv2.resize(resized_frame_zoom, (280, 210), interpolation=cv2.INTER_AREA)
            # out[2:212, 2:282] = resized_frame
            # cv2.rectangle(out, (2, 212), (282, 2), (255, 255, 255), 2)
            # cv2.putText(out, 'LEFT', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)  # orange (30, 105, 210)
            # ------- Secondary View ------- #

            
            self.out_send.write(main_img)
            # if target == 0:
            #     self.out_send.write(out)
            # else:
            #     self.out_send.write(out_w_sec)
            # rate.sleep()

            cv2.namedWindow('Viewpoint', cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow('Viewpoint', 1280, 960)
            # target = np.random.randint(0,7)
            # cv2.imshow('Viewpoint', frame_rs1)
            cv2.imshow('Viewpoint', main_img)
            # if target == 0:
            #     cv2.imshow('Viewpoint', out)
            #     # video.write(main_img) # video record
            # else:
            #     cv2.imshow('Viewpoint', out_w_sec) # showing gaze location
            #     # video.write(out_frame) # video record

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
            # elif key & 0xFF == ord('s'):

        self.running = False
        # self.cap_rs1.release()
        # self.cap_rs2.release()
        self.out_send.release()
        cv2.destroyAllWindows()
        # threading.Thread.exit()


if __name__ == '__main__':
    ip_addr = '130.215.181.94'

    v_thread = videoThread(0, 'videoT', ip_addr)
    v_thread.start()
