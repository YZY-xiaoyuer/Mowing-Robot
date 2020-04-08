#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import math
import actionlib
from msg_srv_act.msg import A_Heading_SyncAction,A_Heading_SyncFeedback,A_Heading_SyncResult
from msg_srv_act.msg import M_location_pose

try :
    import EKF_location as ekf
except:
    from location_application import EKF_location as ekf

try:
    import gauss_projection as gp
except:
    from location_application import gauss_projection as gp

def Calculate_x_y( B, L):

    b_split = math.modf(B / 100)
    l_split = math.modf(L / 100)
    B = float(b_split[1]) + float(b_split[0] * 100) / 60
    L = float(l_split[1]) + float(l_split[0] * 100) / 60
    x, y = gp.GetGSZS54(B, L)
    return x, y


def Coordinate_Alignment(x, y, Yaw):
    yaw_gps = []
    for i in range(1, len(x)):
        x_distance = x[i]-x[i-1]
        y_distance = y[i]-y[i-1]
        if (x_distance != 0) or (y_distance != 0):
            yaw_temp = math.atan2(y_distance, x_distance)
            yaw_gps.append(np.degrees(yaw_temp))

    yaw_imu_mean = np.mean(Yaw)

    if len(yaw_gps) == 0:
        yaw_gps_mean = yaw_imu_mean
    else:
        yaw_gps_mean = (np.mean(yaw_gps)+360) % 360
    return yaw_gps_mean - yaw_imu_mean, yaw_gps_mean


class Heading_Synchronization_Server(object):
    def __init__(self,name):
        self.heading_sync_data = []

        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name, A_Heading_SyncAction, self.execute, False)
        self.subscribe = rospy.Subscriber("EKF_location_pose", M_location_pose, self.analyse,  queue_size=100)
        self.subscribe.unregister()
        self._as.start()

        self._feedback = A_Heading_SyncFeedback()
        self._result = A_Heading_SyncResult()

    def execute(self, goal):
        if (goal.action_type == 0):
            self.subscribe.unregister()
            x = []
            y = []
            yaw = []
            if self.heading_sync_data != []:
                for i in range(len(self.heading_sync_data)):
                    x_now,y_now = Calculate_x_y(self.heading_sync_data.longitude,self.heading_sync_data.latitude)
                    x.append(x_now)
                    y.append(y_now)
                    yaw.append(self.heading_sync_data.yaw)
                ekf.ekf_location_v_w_pitch_yaw.yaw_offset, ekf.ekf_location_v_w_pitch_yaw.origin_gps_yaw =\
                    Coordinate_Alignment(x, y, yaw)
            else:
                rospy.Duration(2)
            self._result.result = 1
            self._as.set_succeeded(self._result)
        elif (goal.action_type == 1):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.action_name)
                self._as.set_preempted()
                return

            self.heading_sync_data = []
            self._feedback.state = 1
            self.subscribe = rospy.Subscriber("EKF_location_pose", M_location_pose, self.analyse,  queue_size = 10)
            rospy.Duration(2)
            self._as.set_aborted()

    def analyse(self,pose_data):
        self.heading_sync_data.append(pose_data)
        print(self.heading_sync_data)

def Heading_Synchronization_Server_Node():

    server = Heading_Synchronization_Server("heading_synchronization_server")
    rospy.loginfo("heading_synchronization_server is ready")

    return

if __name__ == '__main__':
    try:
        rospy.init_node('location_heading_sync_server', anonymous=True)
        Heading_Synchronization_Server_Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass
