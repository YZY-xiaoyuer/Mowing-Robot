#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
import actionlib
from msg_srv_act.msg import A_build_mapAction,A_build_mapFeedback,A_build_mapResult
from msg_srv_act.msg import M_location_pose
import numpy as np
import yaml
from os import path

dir_path = path.dirname(__file__)
parent_path = path.dirname(dir_path)

file_path = parent_path + "/map_pitcure/"
global_map_name = "global_map"

OUTSIDE_COLOR = 200
BOUNDARY_COLOR = 0
WALKABLE_COLOR = 255


# boundary_range The boundary range is the range that expresses the boundary of the map and the current path
# 1 means one pixel (resolution = 5cm)

def Global_Map_Process(src_file, dst_file, boundary_range=0):
    kernel_size = 3
    src_img = cv2.imread(src_file, cv2.COLOR_BGR2GRAY)
    ret, img = cv2.threshold(src_img, 127, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
    img = cv2.erode(img, kernel)
    for i in range(10):
        contours, hierarchy = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 3:
            
            canvas = np.full((src_img.shape[0], src_img.shape[1]), int(255), dtype=np.uint8)
            cons_inside = cv2.drawContours(canvas, contours, 0, BOUNDARY_COLOR, i + 2)

            contours, hierarchy = cv2.findContours(cons_inside, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            canvas = np.full((src_img.shape[0], src_img.shape[1]), int(255), dtype=np.uint8)
            process_img = cv2.drawContours(canvas, contours, 1, BOUNDARY_COLOR, 2 + boundary_range)


            contours, hierarchy = cv2.findContours(process_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            canvas = np.full((src_img.shape[0], src_img.shape[1]), int(OUTSIDE_COLOR), dtype=np.uint8)
            cons_outside = cv2.drawContours(canvas, contours, 1, BOUNDARY_COLOR, 2)

            contours, hierarchy = cv2.findContours(cons_outside, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            cons_outside = cv2.drawContours(cons_outside, contours, 0, WALKABLE_COLOR, -1)
            cv2.imwrite(dst_file, cons_outside)

            break
        else:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
            img = cv2.erode(img, kernel)
    return


def Build_Map(array_x=[], array_y=[], resolution=0.05):
    min_x = min(array_x) - 1
    min_y = min(array_y) - 1
    max_x = max(array_x) + 1
    max_y = max(array_y) + 1
    origin_x = min_x
    origin_y = min_y
    x_size = int((max_x - min_x + resolution) / resolution)
    y_size = int((max_y - min_y + resolution) / resolution)
    matrix_map = np.full((x_size, y_size), int(WALKABLE_COLOR), dtype=np.uint8)
    for i in range(min(len(array_x), len(array_y))):
        map_x = (array_x[i] - origin_x) / resolution
        map_y = (array_y[i] - origin_y) / resolution
        if map_x %1 == 0:
            matrix_map_x = int(map_x)
        else:
            matrix_map_x = int(map_x) + 1
        if map_x %1 == 0:
            matrix_map_y = int(map_y)
        else:
            matrix_map_y = int(map_y) + 1
        matrix_map[matrix_map_x, matrix_map_y] = 0
    origin_map_name = file_path + global_map_name +"_origin.png"
    cv2.imwrite(origin_map_name, matrix_map)

    pitcure_name = file_path + global_map_name +".png"
    Global_Map_Process(origin_map_name,pitcure_name)

    yaml_file_name = file_path + global_map_name + ".yaml"
    py_object = {'image': pitcure_name,
                 'resolution': resolution,
                 'origin': [origin_x, origin_y, 0],
                 'occupied_thresh': 0.65,
                 'free_thresh': 0.196,
                 'negate': 0,
                 }
    file = open(yaml_file_name, 'w', encoding='utf-8')
    yaml.dump(py_object, file)
    file.close()


class Build_Map_Server(object):
    def __init__(self,name):
        self.map_data = []
        self.resolution = 0.05     # m

        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name, A_build_mapAction, self.execute, False)
        self.subscribe = rospy.Subscriber("EKF_location_pose", M_location_pose, self.analyse,  queue_size=100)
        self.subscribe.unregister()
        self._as.start()
        self.end = 0
        self._feedback = A_build_mapFeedback()
        self._result = A_build_mapResult()

    def execute(self, goal):
        if (goal.action_type == 0):
            self.subscribe.unregister()
            if self.map_data != []:
                map_data_x = []
                map_data_y = []
                for i in range(len(self.map_data)):
                    map_data_x.append(self.map_data[i].location_pose[0])
                    map_data_y.append(self.map_data[i].location_pose[1])
                Build_Map(map_data_x,map_data_y,self.resolution)
            else:
                rospy.Duration(2)
            self._result.result = 1
            self._as.set_succeeded(self._result)
        elif (goal.action_type == 1):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.action_name)
                self._as.set_preempted()
                return
            self.end = 0
            self.map_data = []
            self._feedback.state = 1
            self.subscribe = rospy.Subscriber("EKF_location_pose", M_location_pose, self.analyse,  queue_size = 10)
            rospy.Duration(2)
            self._as.set_aborted()

    def analyse(self,pose_data):
        self.map_data.append(pose_data)
        rospy.logdebug(self.map_data)

def Build_Map_Server_Node():
    server = Build_Map_Server("build_map_action_server")
    rospy.loginfo("build_map_server is ready")


if __name__ == '__main__':
    try:
        rospy.init_node('build_map_action_server_node')
        Build_Map_Server_Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass