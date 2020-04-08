#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import copy
try:
    import subimage_coverage
except:
    from planning_application import subimage_coverage

import cv2

OUTSIDE_COLOR = 200
MAP_RESIZE = 5

from msg_srv_act.srv import S_map_server,S_coverage_server,S_coverage_serverResponse

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
        rospy.logerr("Service call failed!  because:" + str(exc))

def Coverage_Server(req):
    global_map = Rqueset_Golbal_map()
    map_shape0 = int(global_map.hight)
    map_shape1 = int(global_map.width)
    map_grid = copy.deepcopy(global_map.map_data)
    map_grid = [int(x) for x in map_grid]
    map_grid = np.array(map_grid)
    map_grid = np.array(map_grid,dtype=np.uint8).reshape(map_shape0,map_shape1)

    # Zoom the map so that the map grid size is equal to the machine size
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21))
    map_grid = cv2.erode(map_grid, kernel)
    img_resize = cv2.resize(map_grid, (int(map_grid.shape[1] / MAP_RESIZE), int(map_grid.shape[0] / MAP_RESIZE)))
    ret,img_process = cv2.threshold(img_resize, 150, 255, cv2.THRESH_BINARY)
    mask =  np.zeros([img_process.shape[0]+2, img_process.shape[1]+2], np.uint8)
    cv2.floodFill(img_process,mask,(0,0),OUTSIDE_COLOR, cv2.FLOODFILL_FIXED_RANGE)


    coverage = subimage_coverage.Subimage_coverage(img_process,img_process,[0,0])
    answer = S_coverage_serverResponse()
    start_pose_x = (req.start_position[0] - global_map.origin[0]) / global_map.resolution
    start_pose_y = (req.start_position[1] - global_map.origin[1]) / global_map.resolution

    map_x = int(start_pose_x/MAP_RESIZE)
    map_y = int(start_pose_y/MAP_RESIZE)

    map_start_point = [map_x,map_y]
    path = coverage.get_coverage_path(map_start_point)
    #coverage.draw_ion()
    path_x = []
    path_y = []
    for i in range(len(path)):
        path_x.append(path[i][0]*MAP_RESIZE*global_map.resolution +global_map.origin[0])
        path_y.append(path[i][1]*MAP_RESIZE*global_map.resolution +global_map.origin[1])

    answer.path_x = copy.deepcopy(path_x)
    answer.path_y = copy.deepcopy(path_y)

    return answer

def Subimage_Converage_Server():
    route_plan_server = rospy.Service("coverage_plan_server", S_coverage_server, Coverage_Server)
    rospy.loginfo("coverage_plan_server is ready")
    return


if __name__ == '__main__':
    try:
        rospy.init_node('coverage_server_node')
        Subimage_Converage_Server()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass