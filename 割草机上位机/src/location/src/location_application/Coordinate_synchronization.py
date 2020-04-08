#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import math
import actionlib
from msg_srv_act.msg import A_Coor_SyncAction, A_Coor_SyncFeedback,A_Coor_SyncResult
from msg_srv_act.msg import M_location_pose

try:
    import EKF_location as ekf
except:
    from location_application import EKF_location as ekf

try:
    import gauss_projection as gp
except:
    from location_application import gauss_projection as gp

def Calculate_x_y(B, L):

    b_split = math.modf(B / 100)
    l_split = math.modf(L / 100)
    B = float(b_split[1]) + float(b_split[0] * 100) / 60
    L = float(l_split[1]) + float(l_split[0] * 100) / 60
    x, y = gp.GetGSZS54(B, L)
    return x, y





class Coordinate_Synchronization_Server(object):
    def __init__(self,name):
        self.coordinate_sync_data = []

        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name, A_Coor_SyncAction, self.execute, False)
        self.subscribe = rospy.Subscriber("EKF_location_pose", M_location_pose, self.analyse,  queue_size=100)
        self.subscribe.unregister()
        self._as.start()

        self._feedback = A_Coor_SyncFeedback()
        self._result = A_Coor_SyncResult()

    def execute(self, goal):
        if (goal.action_type == 0):
            self.subscribe.unregister()
            x_list = []
            y_list = []
            if self.coordinate_sync_data != []:
                for i in range(len(self.coordinate_sync_data)):
                    x_now,y_now = Calculate_x_y(self.heading_sync_data.longitude,self.heading_sync_data.latitude)
                    x_list.append(x_now)
                    y_list.append(y_now)
                ekf.ekf_location_v_w_pitch_yaw.x_offset = np.mean(x_list)
                ekf.ekf_location_v_w_pitch_yaw.y_offset = np.mean(y_list)
            else:
                rospy.Duration(2)
            self._result.result = 1
            self._as.set_succeeded(self._result)
        elif (goal.action_type == 1):

            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.action_name)
                self._as.set_preempted()
                return

            self.coordinate_sync_data = []
            self._feedback.state = 1
            self.subscribe = rospy.Subscriber("EKF_location_pose", M_location_pose, self.analyse,  queue_size = 10)
            rospy.Duration(2)
            self._as.set_aborted()

    def analyse(self,pose_data):
        self.coordinate_sync_data.append(pose_data)
        print(self.coordinate_sync_data)

def Coordinate_Synchronization_Server_Node():

    server = Coordinate_Synchronization_Server("coordinate_synchronization_server")
    rospy.loginfo("coordinate_synchronization_server is ready")

    return
if __name__ == '__main__':
    try:
        rospy.init_node('ekf_location_node', anonymous=True)
        Coordinate_Synchronization_Server_Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass
