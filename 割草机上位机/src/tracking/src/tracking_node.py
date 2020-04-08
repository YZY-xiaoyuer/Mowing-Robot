#!/usr/bin/env python
# license removed for brevity
import rospy
try:
    import tracking_server
except:
    from tracking_application import tracking_server



def Tracking_Server():
    tracking_server.Server_Init()
    return

if __name__ == '__main__':
    try:
        rospy.init_node('tracking_node', anonymous=True)
        Tracking_Server()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass