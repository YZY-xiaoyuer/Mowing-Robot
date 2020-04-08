#!/usr/bin/env python
import rospy
from msg_srv_act.srv import S_coverage_server,S_map_server,S_location_pose_server

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

def Request_Now_Pose():
    server_is_on = rospy.wait_for_service('location_pose_server',timeout=10)
    if server_is_on == False:
        rospy.loginfo("location_pose_server not started ")
        return
    try:
        request_location_pose = rospy.ServiceProxy('location_pose_server', S_location_pose_server)
        resp1 = request_location_pose(1)
        return resp1
    except rospy.ServiceException as exc:
        print("Service call failed!  because:" + str(exc))
        return

def Rqueset_Coverage_Plan():

    server_is_on = rospy.wait_for_service('coverage_plan_server',timeout=10)
    if server_is_on == False:
        rospy.loginfo("coverage_plan_server not started ")
        return
    try:
        request_coverage_plan = rospy.ServiceProxy('coverage_plan_server', S_coverage_server)
        now_pose = Request_Now_Pose()
        now_pose_x = now_pose.pose[0]
        now_pose_y = now_pose.pose[1]
        start_point = [now_pose_x, now_pose_y]

        resp1 = request_coverage_plan(start_point)
        print(resp1)
        return resp1
    except rospy.ServiceException as exc:
        print("Service call failed!  because:" + str(exc))

if __name__ == '__main__':
    rospy.init_node('coverage_plan_clent_node')
    Rqueset_Coverage_Plan()
    rospy.spin()