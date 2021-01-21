#!/usr/bin/python
"""
Sonar array composed of 3 sonars (left, center, right)

Subscribes to the three topics 0 (left) 1(center) 2 (right)

calculates a correction to the cmd_vel that is:
    multiplicative for x
    additive for angle

"""
import rospy, cv2, cv_bridge
import math,time
import numpy as np
from sensor_msgs.msg import Image,Range,LaserScan
from geometry_msgs.msg import Twist

import cv2.aruco as aruco
import sys, time, math



#--- Define Tag

marker_size  = 10 #- [cm]
ids = []
tvec = [0,0,0]


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





DIST_STEER_ENGAGE   = 1.1
DIST_BREAK          = 0.5

DIST_LAT_ENGAGE     = 0.4

K_FRONT_DIST_TO_SPEED   = 0.9
K_LAT_DIST_TO_STEER     = 1.9

TIME_KEEP_STEERING      = 1.5


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

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class Detect_Avoid():
    def __init__(self):
        
        self.range_center   = 3
        self.range_left     = 3
        self.range_right    = 3
        
        self.sub_center = rospy.Subscriber("/sensor/sonar_front", Range, self.update_range_front)
        self.sub_left   = rospy.Subscriber("/sensor/sonar_left_front", Range, self.update_range_left)
        self.sub_right  = rospy.Subscriber("/sensor/sonar_right_front", Range, self.update_range_right)
        rospy.loginfo("Subscribers set")
        
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                        Image, self.image_callback)

        #self.sonar_sub = rospy.Subscriber('sensor/sonar_front',Range,self.sonar_callback)

        #self.laser_sub = rospy.Subscriber('scan',LaserScan,self.laser_callback)
                

        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=1)
        
        self.twist = Twist()
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0
        
    def update_range_front(self, message):
        
        self.range_center = message.range
       

    def update_range_right(self, message):
        
        self.range_right  = message.range
            
        
    def update_range_left(self, message):
       
        self.range_left   = message.range 
            
        #rospy.loginfo("Sonar array: %.1f  %.1f  %.1f"%(self.range_center, self.range_left, self.range_right))



    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                

                
        global ids
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
                #print(ids)

        #global shape
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
            cv2.putText(image, str_position, (0, 100), font, 2, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T

        #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

        #-- Print the marker's attitude respect to camera(image
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),math.degrees(yaw_marker))
            cv2.putText(image, str_attitude, (0, 150), font, 2, (0, 255, 0), 2, cv2.LINE_AA)


        #-- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc*np.matrix(tvec).T

            #str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
            #cv2.putText(image, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

        #-- Get the attitude of the camera respect to the(image
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            #str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),math.degrees(yaw_camera))
            #cv2.putText(image, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(image, 'Marker ID is = '+ str(ids), (0, 250), font, 2, (0, 255, 0), 2, cv2.LINE_AA)


                        #CMD_VEL PUB

                        

                    #cv2.putText(image, ids, (300, 300), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        #cv2.imshow("aruco", image)
        cv2.namedWindow('aruco',cv2.WINDOW_NORMAL)
        cv2.imshow("aruco", image)
        cv2.resizeWindow("aruco",800,800)
        cv2.waitKey(3)

        #return (ids,tvec)




    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """
        
        break_action   = 1.0
        steer_action   = 0.0
        
        #--- Get the minimum distance
        self.ranges   = min([self.range_center, self.range_left, self.range_right])
        #print "%.2f    %.2f  %.2f  %.2f"%(ranges,  self.range_left, self.range_center, self.range_right)
        
        if self.range_center < DIST_STEER_ENGAGE:
            #--- Start applying the break
            # break_action   = (range - DIST_BREAK)/(DIST_STEER_ENGAGE - DIST_BREAK) 
            adim_dist      = self.ranges/DIST_STEER_ENGAGE
            if self.ranges < DIST_BREAK:
                break_action   =  K_FRONT_DIST_TO_SPEED*(self.ranges/DIST_BREAK)
                break_action   = saturate(break_action, 0, 1)
                rospy.loginfo("Engaging break %.1f"%break_action)
            
            #--- Apply steering, proportional to how close is the object
            steer_action       = K_LAT_DIST_TO_STEER*(1.0 - adim_dist)
            steer_action       = self.get_signed_steer(steer_action)
            
            steer_action   = saturate(steer_action, -1.5, 1.5)
            rospy.loginfo("Steering command %.2f"%steer_action)
            
        return (break_action, steer_action)
        
    def get_signed_steer(self, steer_action):
    
        if time.time() > self._time_steer + TIME_KEEP_STEERING:
            print ("> Update steer_action sign")
            self._time_steer  = time.time()
            
            #-- If the lwft object is closer, turn right (negative)
            if self.range_left < self.range_right:
                steer_action = -steer_action      

            if steer_action >= 0:   self._steer_sign_prev = 1
            else:                   self._steer_sign_prev = -1
            
        else:   
            steer_action *= self._steer_sign_prev
            
        return (steer_action)
        
    def run(self):
        
        #--- Set the control rate
        rate = rospy.Rate(5)

        self.last_marker = [[0]]




            
        while not rospy.is_shutdown():
            #-- Get the control action
            break_action, steer_action = self.get_control_action()

            #ids,tvec = self.image_callback()
           
            # rospy.loginfo("Throttle = %3.1f    Steering = %3.1f"%(break_action, steer_action))
            
            #-- update the message

            h, w, d = (1080,1920,3)

            
            err_z = tvec[2]
            err_x = tvec[0] - w/100*(err_z/90)

            reached = 0
            centre = 0
            rate = rospy.Rate(10)

        

            reached = 0

            if(str(ids)==[[2]] and self.last_marker==[[2]]):
                self.last_marker = [[1]]

            if(str(ids)==[[1]] and self.last_marker==[[1]]):
                self.last_marker = [[2]]


                   
            
            
            if(self.ranges > 0.1):

                if(ids is not None and str(ids)!=str(self.last_marker)):


                    self.twist.angular.z = -break_action*float(err_x)/200       
                    if(abs(err_x)<10 and not reached):     

                        self.fnGoStraight((break_action*float(err_z)/150))
                        centre=1

                    if(err_z<=20):
                        self.fnStop()
                   
                        if(self.last_marker==[[2]]):
                            self.last_marker = [[1]]
                        elif(self.last_marker==[[1]]):
                            self.last_marker = [[2]]
                        self.twist.angular.z = 0.5
                        #time.sleep(3)                      
                        

                    if(self.ranges < 0.1):
                        self.fnStop()

                else:
                    self.fnStop()
                    self.twist.angular.z = 0.5

                    if(self.ranges < 0.1):
                        self.fnStop()

            else:

                self.fnStop()
                self.fnavoid(break_action,steer_action)


                



            

            '''

                if(ids is not None):
                    self.twist.angular.z = -float(err_x)/100
                    
                    if(err_x>-5 and err_x<5 and front<=15):
                        self.twist.linear.x = 0.1*(float(err_z)/100)

                    else:
                        self.twist.linear.x = 0.0
                else:
                    self.twist.linear.x = 0    
            
            '''
            
            
            self.cmd_vel_pub.publish(self.twist)

            #rate.sleep()        



    def fnStop(self):
        self.twist = Twist()
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

    def fnavoid(self,break_action,steer_action):
        #Kp = 1.2

        self.twist = Twist()
        self.twist.linear.x = break_action
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = steer_action*0.8
        self.cmd_vel_pub.publish(self.twist) 

    

    def fnGoStraight(self,break_action):
        self.twist = Twist()
        self.twist.linear.x = break_action/4
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)
            
if __name__ == "__main__":

    rospy.init_node('detect_avoid')
    
    detect_avoid = Detect_Avoid()
    detect_avoid.run()            