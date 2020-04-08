#!/usr/bin/env python
import rospy
from msg_srv_act.msg import M_location_pose
import matplotlib.pyplot as plt
import copy
import numpy as np
import math
from msg_srv_act.srv import S_map_server
import cv2

def Rqueset_Golbal_map():

    server_is_on = rospy.wait_for_service('map_provided_server',timeout=10)
    if server_is_on == False:
        rospy.loginfo("map_provided_server not started ")
        return
    try:
        request_global_map = rospy.ServiceProxy('map_provided_server', S_map_server)
        resp1 = request_global_map(0)
        return resp1
    except rospy.ServiceException as exc:
        print("Service call failed!  because:" + str(exc))

class Display_Map(object):
    def __init__(self):

        self.global_map = Rqueset_Golbal_map()
        map_shape0 = int(self.global_map.hight)
        map_shape1 = int(self.global_map.width)
        map_grid = copy.deepcopy(self.global_map.map_data)
        map_grid = [int(x) for x in map_grid]
        map_grid = np.array(map_grid)
        self.map = np.array(map_grid,dtype=np.uint8).reshape(map_shape0,map_shape1)
        self.now_color = (100)
        self.path_color = (150)
        self.draw_frequency = 1

    def draw(self,pose_data):
        self.draw_frequency += 1
        if self.draw_frequency % 5 == 0:
            plt.clf()
            x = pose_data.location_pose[0]
            y = pose_data.location_pose[1]
            pose_x = int((x - self.global_map.origin[0]) / self.global_map.resolution)
            pose_y = int((y - self.global_map.origin[1]) / self.global_map.resolution)

            self.map[pose_x, pose_y] = self.now_color
            plt.imshow(self.map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=255)
            self.map[pose_x, pose_y] = self.path_color
            plt.pause(0.001)

    def updata_map(self):
        self.global_map = Rqueset_Golbal_map()
        map_shape0 = int(self.global_map.hight)
        map_shape1 = int(self.global_map.width)
        map_grid = copy.deepcopy(self.global_map.map_data)
        map_grid = [int(x) for x in map_grid]
        map_grid = np.array(map_grid)
        self.map = np.array(map_grid,dtype=np.uint8).reshape(map_shape0,map_shape1)


display = Display_Map()

def Display(pose_data):
    display.draw(pose_data)
    return

def subscription():

    rospy.init_node('display_location', anonymous=True)

    rospy.Subscriber("EKF_location_pose", M_location_pose, Display)

    rospy.loginfo("display node is Ready")


    rospy.spin()

if __name__ == '__main__':
    try:
        subscription()
    except rospy.ROSInterruptException:
        print(__file__)
        pass