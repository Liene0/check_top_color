#!/usr/bin/env python3

#ros
import rospy
from robotbase.srv import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#python
import cv2
import numpy as np

bridge = CvBridge()
kernel = np.ones((3, 3), np.uint8)
global Check
global num # no. of pole
Check = 0 # 0 = null : 1 = red : 2 = blue
num = 0
def show_image(name,img):
    cv2.imshow(name, img)
    cv2.waitKey(3)


def image_callback(img_msg):
    #set 
    global Check,num
    num = num
    Check = Check
    cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    h, w, d = cv_image.shape
    wsize = 35
    if num ==0:
        aim = cv_image[150:250,int((w/2)-wsize):int((w/2)+wsize)]
    elif num == 1:
        aim = cv_image[150:250,int((w/2)-wsize):int((w/2)+wsize)]
    
    hsv = cv2.cvtColor(aim, cv2.COLOR_BGR2HSV)

    #set color
    lower_red = np.array([0,0,70]) #[160,20,70]
    upper_red = np.array([190,255,255])
    lower_blue = np.array([101,50,38])
    upper_blue = np.array([110,255,255])
    Red = cv2.inRange(hsv, lower_red, upper_red)
    Blue = cv2.inRange(hsv, lower_blue, upper_blue)
    
    #Filter
    Redm = cv2.erode(Red, kernel, iterations=1)
    Bluem = cv2.erode(Blue, kernel, iterations=1)
    Redmask = cv2.dilate(Redm, kernel, iterations=1)
    Bluemask = cv2.dilate(Bluem, kernel, iterations=1)

    #solve
    ha, wa, da = aim.shape
    state = 0
    for i in range(ha):
        for j in range(wa):
            if state == 0 :
                if Redmask[i,j]>Bluemask[i,j]:
                    print("Top is Red!")
                    Check = 1
                    state = 1
                    break
                elif Redmask[i,j]<Bluemask[i,j]:
                    print("Top is Blue!")
                    Check = 2
                    state = 1
                    break
                else:
                    print("Not have")
                    Check = 0
                    state = 1
                    break

    show_image("mask",Redmask)
    show_image("mask",Bluemask)
    show_image("hsv",hsv)
    
    return Check

def solcol(req):
    global Check,num
    num = req.num
    print (Check)
    return ChkColResponse(Check) 

def check_color():
    rospy.init_node('Check_Color', anonymous=True)
    sub_image = rospy.Subscriber("/rgbd/color/image_raw", Image, image_callback)#/rgb_to_depth/image_raw
    s = rospy.Service('ChkCol',ChkCol,solcol)
    

if __name__ == "__main__":
    check_color()
    rospy.spin()
# sub_image = rospy.Subscriber("/rgb_to_depth/image_raw", Image, image_callback)
# while not rospy.is_shutdown():
#     rospy.spin()