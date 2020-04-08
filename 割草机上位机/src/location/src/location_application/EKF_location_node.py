#!/usr/bin/env python

import rospy
from msg_srv_act.msg import M_location_pose, M_senser_data
import sys
try :
    import EKF_location
except:
    from location_application import EKF_location
import copy
import time

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
            print("send cmd successful")
        else:
            print("send cmd false")
        return resp1.answer
    except rospy.ServiceException as exc:
        print("Service call failed!  because:" + str(exc))

def Location_msg_add(location_data,location_covariance):
    location = M_location_pose()
    for i in range(len(location_data)):
        location.location_pose[i] = copy.deepcopy(float(location_data[i]))

    location.covariance_pose[0] = copy.deepcopy(location_covariance[0][0])
    location.covariance_pose[1] = copy.deepcopy(location_covariance[0][1])
    location.covariance_pose[2] = copy.deepcopy(location_covariance[0][2])
    location.covariance_pose[3] = copy.deepcopy(location_covariance[0][3])
    location.covariance_pose[4] = copy.deepcopy(location_covariance[0][4])
    location.covariance_pose[5] = copy.deepcopy(location_covariance[1][0])
    location.covariance_pose[6] = copy.deepcopy(location_covariance[1][1])
    location.covariance_pose[7] = copy.deepcopy(location_covariance[1][2])
    location.covariance_pose[8] = copy.deepcopy(location_covariance[1][3])
    location.covariance_pose[9] = copy.deepcopy(location_covariance[1][4])
    location.covariance_pose[10] = copy.deepcopy(location_covariance[2][0])
    location.covariance_pose[11] = copy.deepcopy(location_covariance[2][1])
    location.covariance_pose[12] = copy.deepcopy(location_covariance[2][2])
    location.covariance_pose[13] = copy.deepcopy(location_covariance[2][3])
    location.covariance_pose[14] = copy.deepcopy(location_covariance[2][4])
    location.covariance_pose[15] = copy.deepcopy(location_covariance[3][0])
    location.covariance_pose[16] = copy.deepcopy(location_covariance[3][1])
    location.covariance_pose[17] = copy.deepcopy(location_covariance[3][2])
    location.covariance_pose[18] = copy.deepcopy(location_covariance[3][3])
    location.covariance_pose[19] = copy.deepcopy(location_covariance[3][4])
    location.covariance_pose[20] = copy.deepcopy(location_covariance[4][0])
    location.covariance_pose[21] = copy.deepcopy(location_covariance[4][1])
    location.covariance_pose[22] = copy.deepcopy(location_covariance[4][2])
    location.covariance_pose[23] = copy.deepcopy(location_covariance[4][3])
    location.covariance_pose[24] = copy.deepcopy(location_covariance[4][4])
    return location


ekf_location = EKF_location.ekf_location_v_w_pitch_yaw
pub_location = rospy.Publisher('EKF_location_pose', M_location_pose, queue_size=10)

def Location(senser_data):
    ekf_location.Location_Realtime(senser_data)
    location_data = ekf_location.ekf_location.get_state()
    location_covariance = ekf_location.ekf_location.get_covar()
    location_pose = Location_msg_add(location_data,location_covariance)
    if senser_data.e_gps_mode == 4 or senser_data.e_gps_mode == 5:
        location_pose.location_state = 0
    else:
        location_pose.location_state = 1
    pub_location.publish(location_pose)
    rospy.logdebug(location_pose.location_pose)
    return


def EKF_location():
    Sub_senser_data = rospy.Subscriber("lower_machine_senser_data", M_senser_data, Location)
    send_location_cmd_client(2, 1)
    rospy.loginfo("the EKF_location is ready")


if __name__ == '__main__':
    try:
        rospy.init_node('ekf_location_node', anonymous=True)
        EKF_location()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass
