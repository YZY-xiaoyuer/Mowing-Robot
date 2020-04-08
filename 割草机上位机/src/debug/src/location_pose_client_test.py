#!/usr/bin/env python
import rospy
from msg_srv_act.srv import S_location_pose_server

def Rqueset_Location_Pose():
    server_is_on = rospy.wait_for_service('location_pose_server',timeout=10)
    if server_is_on == False:
        rospy.loginfo("location_pose_server not started ")
        return
    try:
        request_location_pose = rospy.ServiceProxy('location_pose_server', S_location_pose_server)
        resp1 = request_location_pose(1)
        print(resp1)
        return resp1
    except rospy.ServiceException as exc:
        print("Service call failed!  because:" + str(exc))
        return

if __name__ == '__main__':
    try:
        rospy.init_node('location_pose_test_node', anonymous=True)
        Rqueset_Location_Pose()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass
