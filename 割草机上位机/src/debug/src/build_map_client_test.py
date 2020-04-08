#!/usr/bin/env python
# license removed for brevity
import rospy
import actionlib
from msg_srv_act.msg import A_build_mapAction,A_build_mapGoal
import protocol_client_test

def Start_Build_Map():
    rospy.init_node('build_map_client_node')
    client = actionlib.SimpleActionClient('build_map_action_server', A_build_mapAction)
    client.wait_for_server()

    goal = A_build_mapGoal()
    goal.action_type = 1
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(timeout=rospy.Duration(0))
    result = client.get_result()
    print(result)

def End_Build_Map():
    rospy.init_node('build_map_client_node')
    client = actionlib.SimpleActionClient('build_map_action_server', A_build_mapAction)
    client.wait_for_server()

    goal = A_build_mapGoal()
    goal.action_type = 0
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(timeout=rospy.Duration(0))
    result = client.get_result()
    print(result)

if __name__ == '__main__':
    if 0:
        Start_Build_Map()
        protocol_client_test.send_location_cmd_client(2,1)
    else:
        End_Build_Map()
        #protocol_client_test.send_location_cmd_client(2,0)
    rospy.spin()
