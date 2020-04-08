#!/usr/bin/env python3
import rospy
try:
    import EKF_location_node
except:
    from location_application import EKF_location_node
try:
    import location_pose_server
except:
    from location_application import location_pose_server

def Location_Server():
    EKF_location_node.EKF_location()
    location_pose_server.Location_Pose_Server()
    return

if __name__ == '__main__':
    try:
        rospy.init_node("location_node", anonymous=True)
        Location_Server()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass