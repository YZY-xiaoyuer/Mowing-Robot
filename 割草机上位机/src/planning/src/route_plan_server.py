#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import copy
try:
    import A_star
except:
    from planning_application import A_star
import cv2

from msg_srv_act.srv import S_route_plan_server,S_map_server,S_route_plan_serverResponse


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

def Route_Plan(req):
    global_map = Rqueset_Golbal_map()
    map_shape0 = int(global_map.hight)
    map_shape1 = int(global_map.width)
    map_grid = copy.deepcopy(global_map.map_data)
    map_grid = [int(x) for x in map_grid]
    map_grid = np.array(map_grid)
    map_grid = np.array(map_grid,dtype=np.uint8).reshape(map_shape0,map_shape1)
    a_star = A_star.AStar(map_grid,map_shape0,map_shape1)

    start_pose_x = (req.start_position[0] - global_map.origin[0]) / global_map.resolution
    start_pose_y = (req.start_position[1] - global_map.origin[1]) / global_map.resolution

    map_x = int(start_pose_x)
    map_y = int(start_pose_y)

    map_start_point = [map_x,map_y]
    goal_pose_x = (req.goal_position[0] - global_map.origin[0]) / global_map.resolution
    goal_pose_y = (req.goal_position[1] - global_map.origin[1]) / global_map.resolution
    goal_map_x = int(goal_pose_x)
    goal_map_y = int(goal_pose_y)

    #map_start_point = [160,175]
    map_goal_piont = [goal_map_x,goal_map_y]
    path = a_star.get_path(map_start_point,map_goal_piont)
    answer = S_route_plan_serverResponse()

    # draw the path when test the route plan
    #a_star.draw_path()
    path_x = []
    path_y = []
    for i in range(len(path)):
        path_x.append(path[i][0]*global_map.resolution +global_map.origin[0])
        path_y.append(path[i][1]*global_map.resolution +global_map.origin[1])
    if len(path) > 1:
        path_x.append(req.goal_position[0])
        path_y.append(req.goal_position[1])
    answer.path_x = copy.deepcopy(path_x)
    answer.path_y = copy.deepcopy(path_y)

    return answer

def Route_Plan_Server():
    route_plan_server = rospy.Service("rout_plan_server", S_route_plan_server, Route_Plan)
    rospy.loginfo("rout_plan_server is ready")
    return

if __name__ == '__main__':
    try:
        rospy.init_node('rout_plan_server_node')
        Route_Plan_Server()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass