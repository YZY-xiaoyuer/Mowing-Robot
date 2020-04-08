#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
import numpy as np
import yaml
from msg_srv_act.srv import S_map_server, S_map_serverResponse
import copy
from os import path

dir_path = path.dirname(__file__)
parent_path = path.dirname(dir_path)

file_path = parent_path + "/map_pitcure/"
global_map_name = "global_map"

def Get_Global_Map(fila_path):
    yaml_file = fila_path
    file = open(yaml_file, 'r', encoding="utf-8")
    file_data = file.read()
    file.close()
    data = yaml.load(file_data,Loader=yaml.FullLoader)
    return data

def Map_Priovided(req):
    if req.server_type == 0:
        yaml_path = file_path + global_map_name +".yaml"
        map_yaml_data = Get_Global_Map(yaml_path)
        map_path = map_yaml_data.get("image")
        map = cv2.imread(map_path,cv2.COLOR_BGR2GRAY)
        answer = S_map_serverResponse()
        answer.resolution = copy.deepcopy(map_yaml_data.get("resolution"))
        answer.hight = map.shape[0]
        answer.width = map.shape[1]
        answer.origin = copy.deepcopy(map_yaml_data.get("origin"))
        answer.map_data = copy.deepcopy(bytes(map))
        return answer

def Map_Provided_Server_Node():

    map_provided_server = rospy.Service('map_provided_server', S_map_server, Map_Priovided)
    rospy.loginfo("map_provided_server is ready")


if __name__ == '__main__':
    try:
        rospy.init_node('map_provided_server_node')
        Map_Provided_Server_Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass