#!/usr/bin/env python
import rospy
import sys
try:
    import EKF_location
except:
    from location_application import EKF_location
from msg_srv_act.srv import S_location_pose_server, S_location_pose_serverResponse
import copy
import time

def Provided_Pose(req):

    if req.server_type == 1:
        location = EKF_location.ekf_location_v_w_pitch_yaw
        location_pose = location.ekf_location.get_state()
        return S_location_pose_serverResponse(location_pose)

def Location_Pose_Server():
    location_server = rospy.Service('location_pose_server', S_location_pose_server, Provided_Pose)
    rospy.loginfo("location_pose_server is ready")

if __name__ == '__main__':
    try:
        rospy.init_node('location_pose_node', anonymous=True)
        Location_Pose_Server()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass