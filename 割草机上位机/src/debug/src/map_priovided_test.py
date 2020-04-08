#!/usr/bin/env python
import rospy
from msg_srv_act.srv import S_map_server
import cv2
import numpy as np
def Rqueset_Golbal_map():
    rospy.init_node('map_provided_client_node')
    server_is_on = rospy.wait_for_service('map_provided_server',timeout=10)
    if server_is_on == False:
        rospy.loginfo("map_provided_server not started ")
        return
    try:
        request_global_map = rospy.ServiceProxy('map_provided_server', S_map_server)
        resp1 = request_global_map(0)
        print(resp1)
        return resp1
    except rospy.ServiceException as exc:
        print("Service call failed!  because:" + str(exc))

if __name__ == '__main__':
    Rqueset_Golbal_map()
    rospy.spin()