#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import copy
from msg_srv_act.srv import S_along_edge_server,S_map_server,S_along_edge_serverResponse
try:
    import along_edge_plan
except:
    from planning_application import along_edge_plan


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

def Along_Edge(req):
    global_map = Rqueset_Golbal_map()
    map_shape0 = int(global_map.hight)
    map_shape1 = int(global_map.width)
    map_grid = copy.deepcopy(global_map.map_data)
    map_grid = [int(x) for x in map_grid]
    map_grid = np.array(map_grid)
    map_grid = np.array(map_grid,dtype=np.uint8).reshape(map_shape0,map_shape1)
    along_edge = along_edge_plan.Along_side(map_grid)

    answer = S_along_edge_serverResponse()
    start_pose_x = (req.start_position[0] - global_map.origin[0]) / global_map.resolution
    start_pose_y = (req.start_position[1] - global_map.origin[1]) / global_map.resolution

    map_x = int(start_pose_x)
    map_y = int(start_pose_y)

    map_start_point = [map_x,map_y]
    path = along_edge.get_coverage_path(map_start_point)
    if path == 1:
        rospy.logdebug("the starting position  error")
    else:
        path_x = []
        path_y = []
        for i in range(len(path)):
            path_x.append(path[i][0]*global_map.resolution +global_map.origin[0])
            path_y.append(path[i][1]*global_map.resolution +global_map.origin[1])

        answer.path_x = copy.deepcopy(path_x)
        answer.path_y = copy.deepcopy(path_y)

    return answer

def Along_Edge_Server():
    along_edge_server = rospy.Service("along_edge_server", S_along_edge_server, Along_Edge)
    rospy.loginfo("along_edge_server is ready")
    return


if __name__ == '__main__':
    try:
        rospy.init_node('along_edge_server_node')
        Along_Edge_Server()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass