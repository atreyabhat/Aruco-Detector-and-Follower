#!/usr/bin/env python


#This Program is tested on Gazebo Simulator
#This script uses the cv_bridge package to convert images coming on the topic
#sensor_msgs/Image to OpenCV messages and then convert their colors from RGB to HSV
#then apply a threshold for hues near the color yellow to obtain the binary image
#to be able to see only the yellow line and then follow that line
#It uses an approach called proportional and simply means

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image,Range,LaserScan
from geometry_msgs.msg import Twist

import cv2.aruco as aruco
import sys, time, math

#--- Define Tag

marker_size  = 10 #- [cm]
ids = []
tvec = []
#tvec = [0,0,0]


#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_webcam.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_webcam.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters  = aruco.DetectorParameters_create()

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN



def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])




class Detector:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()
                cv2.namedWindow("window", 1)

                self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

                self.sonar_sub = rospy.Subscriber('sensor/sonar_front',Range,self.sonar_callback)

                #self.laser_sub = rospy.Subscriber('scan',LaserScan,self.laser_callback)
                

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)

                self.twist = Twist()

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                

                
                global ids
                corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
                #print(ids)

                global shape
                shape = image.shape

                
                if ids is not None :
        
        #-- ret = [rvec, tvec, ?]
        #-- array of rotation and position of each marker in camera(image
        #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera(image
        #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera(image
                        ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

        #-- Unpack the output, get only the first
                        global tvec
                        rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

        #-- Draw the detected marker and put a reference(image over it
                        aruco.drawDetectedMarkers(image, corners)
                        aruco.drawAxis(image, camera_matrix, camera_distortion, rvec, tvec, 10)


                 #-- Print the tag position in camera(image
                        str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                        cv2.putText(image, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Obtain the rotation matrix tag->camera
                        R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
                        R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        #-- Print the marker's attitude respect to camera(image
                        str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),math.degrees(yaw_marker))
                        cv2.putText(image, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


        #-- Now get Position and attitude f the camera respect to the marker
                        pos_camera = -R_tc*np.matrix(tvec).T

                        str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                        cv2.putText(image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Get the attitude of the camera respect to the(image
                        roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                        str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                            math.degrees(yaw_camera))
                        cv2.putText(image, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        cv2.putText(image, str(ids), (0, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


                        #CMD_VEL PUB

                        

                    #cv2.putText(image, ids, (300, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow("window", image)
                cv2.waitKey(3)

        
        def sonar_callback(self, msg):

                front = msg.range
                #print(front)
                
                reached = 0
            

                
                
                h, w, d = (1080,1920,3)

                err_x = tvec[0] + 10
                err_z = tvec[2]
                #print(tvec[0])

                #print()

                #print("%2.0f" %err_x,int(ids[0]))

                

                if(ids is not None):
                    self.twist.angular.z = -float(err_x)/100
                    
                    if(err_x>-5 and err_x<5 and front<=10):
                        self.twist.linear.x = 0.1*(float(err_z)/100)

                    else:
                        self.twist.linear.x = 0.0
                        #self.twist.angular.z = 0

                else:
                    self.twist.angular.z = 0.1
                    self.twist.linear.x = 0    

                    
                self.cmd_vel_pub.publish(self.twist)
        '''

        def laser_callback(self,msg):

                #print(msg.ranges[0],msg.ranges[90],msg.ranges[180],msg.ranges[270],msg.ranges[359])

                
                detect_array = [0,0,0]
                reached = 0

                if(min(msg.ranges[0:20]) < 0.3 or min(msg.ranges[340:359])<0.3):
                    detect_array[0]=1
                else:
                    detect_array[0]=0
                if(min(msg.ranges[300:340])<0.3):
                    detect_array[2]=1
                else:
                    detect_array[2]=0
                if(min(msg.ranges[30:70])<0.3):
                    detect_array[1]=1
                else:
                    detect_array[1]=0

                print(detect_array)

            

                
                
                h, w, d = (1080,1920,3)

                err_x = tvec[0] + 10
                err_z = tvec[2]
                #print(tvec[0])

                #print()

                #print("%2.0f" %err_x,int(ids[0]))

                

                if(ids is not None and not(max(detect_array))):
                    self.twist.angular.z = -float(err_x)/100
                    
                    if(err_x>-5 and err_x<5):

                        if((max(detect_array))):
                            self.twist.linear.x = 0

                            if(detect_array[0]):
                                self.twist.linear.x = 0
                            if(detect_array[1]):
                                self.twist.angular.z = 0.1
                            if(detect_array[2]):
                                self.twist.angular.z = -0.1


                        else:
                            self.twist.linear.x = 0.1*(float(err_z)/100)

                else:
                    self.twist.linear.x = 0    

                    
                self.cmd_vel_pub.publish(self.twist)

             '''   
                

rospy.init_node('aruco_detector')
detector = Detector()
rospy.spin()