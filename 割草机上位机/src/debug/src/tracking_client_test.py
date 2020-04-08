#!/usr/bin/env python
import rospy
import route_plan_client_test
import coverage_plan_client_test
import along_edge_client_test
from msg_srv_act.msg import A_trackingAction,A_trackingGoal,A_trackingFeedback
import actionlib

def Done_cb(state,result):
    print(state,result)


def FreeBack_cb(freeback):
    rospy.logdebug(freeback)


def Request_Tracking():
    #path = route_plan_client_test.Rqueset_Route_Plan()
    #path = coverage_plan_client_test.Rqueset_Coverage_Plan()
    path = along_edge_client_test.Rqueset_Along_Edge_Plan()
    client = actionlib.SimpleActionClient('tracking_server_node', A_trackingAction)
    client.wait_for_server()

    goal = A_trackingGoal()
    goal.action_type = 1
    goal.path_x = path.path_x
    goal.path_y = path.path_y
    # Fill in the goal here
    client.send_goal(goal,done_cb=Done_cb,feedback_cb=FreeBack_cb)
    #client.wait_for_result(timeout=rospy.Duration(0))
    #result = client.get_result()
    #print(result)

if __name__ == '__main__':
    rospy.init_node('tracking_client_node')
    Request_Tracking()
    rospy.spin()