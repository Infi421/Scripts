#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Twist

LIST_OF_ARUCOS = [0, 4, 2 ,1]
ROTATING_SPEED = 200 
LINEAR_SPEED = 150
THRESHOLD_AREA_MIN = 3800 
THRESHOLD_AREA_MAX = 4000 


class TARUCO_Nav:
    def __init__(self):
        rospy.init_node('aruco_tag_detector', anonymous=True)
        
        # self.count = 0
        # self.list_of_arucos = [0, 1, 2, 3]
        self.target_aruco = 0
        # self.recent_aruco = None 
        
        self.bridge = CvBridge()
        
        #rospy.Subscriber('/zed2/zed_node/right/image_rect_color', Image, self.image_callback)
        rospy.Subscriber('/video_output', Image, self.image_callback) 

        self.camera_focal_length_x = 700  # Focal length in pixels
        self.aruco_tag_size = 0.1  # Size of the ArUco tag in meters
        
        self.left_grid = 640/3
        self.right_grid = 2*self.left_grid         #If face RIGHT-LEFT problem, swap left_grid with right_grid
        
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        self.active = True 


    def finding_aruco(self):
        if self.active:
            rospy.loginfo("CMD_VEL: ROVER FINDING ARUCOS")
            data = Twist()
            data.angular.z = ROTATING_SPEED
            self.pub.publish(data)
    
    def stop(self):
        if self.active:
            rospy.loginfo("CMD_VEL: ROVER STOPPED")
            data = Twist()
            data.angular.z = 0
            data.linear.x = 0 
            self.pub.publish(data)
        
    def move_forward(self):
        if self.active:
            rospy.loginfo("CMD_VEL: ROVER MOVING FORWARD")
            data = Twist()
            data.linear.x = LINEAR_SPEED
            self.pub.publish(data)
    
    def move_right(self):
        if self.active:
            rospy.loginfo("CMD_VEL: ROVER MOVING RIGHT")
            data = Twist()
            data.angular.z = ROTATING_SPEED
            self.pub.publish(data)
    
    def move_left(self):
        if self.active:
            rospy.loginfo("CMD_VEL: ROVER MOVING LEFT")
            data = Twist()
            data.angular.z = - ROTATING_SPEED
            self.pub.publish(data)
        
    
    def reached(self, area, id):
        if area > THRESHOLD_AREA_MIN and area < THRESHOLD_AREA_MAX:
            self.stop()
            self.active = False 
            return True  
    

    
    def end_message(self):
        rospy.loginfo("TRAVERSAL DONEEEEEE!!!!!!!!!!!!!!!")
        self.stop() 
    
    def image_callback(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            height, width, channels = cv_image.shape # 360, 640 
            
            aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
            parameters = aruco.DetectorParameters_create()

            corners, ids, rejectedImgPoints = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
            
            message = ""
            
            if ids is not None:
                count = 0
                print(ids)
                for accepted_id in LIST_OF_ARUCOS:
                    for id, corner in zip(ids, corners): 
                    
                        center_x = None
                        center_y = None
                        if id == accepted_id :
                            aruco.drawDetectedMarkers(cv_image, corners, ids)
                            left_top, right_top, right_bottom, left_bottom  = corner[0]
                            center = (left_top + right_top + right_bottom + left_bottom)/4
                            center_x = center[0]
                            center_y = center[1]
                            count = count +1
                        
                            area = cv2.contourArea(corner[0])
                    
                        if center_x == None and center_y == None:
                            continue

                        # LEFT
                        if center_x <= self.left_grid:        
                            self.move_left()
                        
                        #RIGHT
                        elif center_x >= self.right_grid:
                            
                            self.move_right()
                        
                        #MIDDLE 
                        else: 
                            if not self.reached(area, id):
                                self.move_forward()
                            else:
                                self.finding_aruco()
                
                    if count != 0:
                        break

            elif ids is None:                  #NEW ADDED
                self.finding_aruco()       
                       

            cv2.imshow('Aruco Tags Detection', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error processing image: {}".format(str(e)))

if __name__ == '__main__':
    try:
        detector = TARUCO_Nav()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()