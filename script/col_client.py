#!/usr/bin/env python3
import rospy
from robotbase.srv import *

def chkcol_clients(x):
    rospy.wait_for_service('ChkCol')
    try:
        chkcol = rospy.ServiceProxy('ChkCol',ChkCol)
        resp1 = chkcol(int(x))

        return resp1.state
    except rospy.ServiceException as e :
        print("Service call failed %s"%e)

if __name__ == "__main__" :
    x = 1
    print("Top is" , chkcol_clients(x)) 