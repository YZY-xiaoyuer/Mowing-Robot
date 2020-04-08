#!/usr/bin/env python
import sys
import rospy
import time

# move forward (2,100,0,1000)  turn left/rigit(3, 0, 0.2, 90)-- v(mm/s) w(rad/s)  s_or_angle (mm or angle)
def send_mov_cmd_client(cmd_type = 0, v = 0, w = 0, s_or_angle = 0.0):
    from msg_srv_act.srv import S_mov_control_cmd

    server_is_on = rospy.wait_for_service('send_mov_cmd',timeout=10)
    if server_is_on == False:
        rospy.loginfo("protocol server not started ")
        return
    try:
        send_mov_cmd = rospy.ServiceProxy('send_mov_cmd', S_mov_control_cmd)
        resp1 = send_mov_cmd(cmd_type, v, w, s_or_angle)
        if resp1.answer:
            rospy.logdebug("send cmd successful")
        else:
            rospy.logdebug("send cmd false")
        return resp1.answer
    except rospy.ServiceException as exc:
        rospy.logdebug("Service call failed!  because:" + str(exc))


def send_remote_cmd_client(cmd_type = 0, v = 0, w = 0, s_or_angle = 0):
    from msg_srv_act.srv import S_mov_control_cmd

    server_is_on = rospy.wait_for_service('send_remote_cmd',timeout=10)
    if server_is_on == False:
        rospy.logdebug("protocol server not started ")
        return
    try:
        send_remote_cmd = rospy.ServiceProxy('send_remote_cmd', S_mov_control_cmd)
        resp1 = send_remote_cmd(cmd_type, v, w, s_or_angle)
        if resp1.answer:
            rospy.logdebug("send cmd successful")
        else:
            rospy.logdebug("send cmd false")
        return resp1.answer
    except rospy.ServiceException as exc:
        rospy.logdebug("Service call failed!  because:" + str(exc))


def send_location_cmd_client(server_type,server_state):
    from msg_srv_act.srv import S_location_server

    server_is_on = rospy.wait_for_service('send_location_server',timeout=10)
    if server_is_on == False:
        rospy.loginfo("protocol server not started ")
        return
    try:
        send_location_cmd = rospy.ServiceProxy('send_location_server', S_location_server)
        resp1 = send_location_cmd(server_type, server_state)
        if resp1.answer:
            rospy.logdebug("send cmd successful")
        else:
            rospy.logdebug("send cmd false")
        return resp1.answer
    except rospy.ServiceException as exc:
        rospy.logdebug("Service call failed!  because:" + str(exc))




if __name__ == "__main__":
    if len(sys.argv) == 5:
        cmd_type = int(sys.argv[1])
        v = int(sys.argv[2])
        w = int(sys.argv[3])
        s_or_angle = int(sys.argv[4])
        send_mov_cmd_client(cmd_type, v, w, s_or_angle)
    else:
        #send_location_cmd_client(2,1)
        send_mov_cmd_client(3, 0, 0.8, 30)
        time.sleep(3)
        send_mov_cmd_client(2,300,0,1200)



