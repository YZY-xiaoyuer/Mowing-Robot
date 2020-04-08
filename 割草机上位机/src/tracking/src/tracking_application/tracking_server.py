#!/usr/bin/env python
# license removed for brevity
import rospy
import actionlib
try:
    import tracking
except:
    from tracking_application import tracking

from msg_srv_act.msg import A_trackingAction,A_trackingFeedback,A_trackingResult
from msg_srv_act.msg import M_location_pose

MOVE_SPEED = 200
W_SPEED = 0.2

# move forward (2,100,0,1000)  turn left/rigit(3, 0, 0.2, 90)-- v(mm/s) w(rad/s)  s_or_angle (mm or angle)
def send_mov_cmd_client(cmd_type = 0, v = 0, w = 0.0, s_or_angle = 0):
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
        print("Service call failed!  because:" + str(exc))


def send_remote_cmd_client(cmd_type = 0, v = 0, w = 0, s_or_angle = 0):
    from msg_srv_act.srv import S_mov_control_cmd

    server_is_on = rospy.wait_for_service('send_remote_cmd',timeout=10)
    if server_is_on == False:
        rospy.loginfo("protocol server not started ")
        return
    try:
        send_remote_cmd = rospy.ServiceProxy('send_remote_cmd', S_mov_control_cmd)
        resp1 = send_remote_cmd(cmd_type, v, w, s_or_angle)
        if resp1.answer:
            print("send cmd successful")
        else:
            print("send cmd false")
        return resp1.answer
    except rospy.ServiceException as exc:
        print("Service call failed!  because:" + str(exc))

class Tracking_Server(object):
    def __init__(self, name):
        self.action_name = name
        self._as = actionlib.SimpleActionServer(self.action_name, A_trackingAction, self.execute, False)
        self.subscribe = rospy.Subscriber("EKF_location_pose", M_location_pose, self.analyse,  queue_size=10)
        self.subscribe.unregister()
        self._as.start()
        self._feedback = A_trackingFeedback()
        self._result = A_trackingResult()
        self.tracking = tracking.Track()
        self.control_frequence = 1
        rospy.loginfo("the tracking_action_node is ready.")

    def execute(self, goal):
        if goal.action_type == 1:
            path = []
            for i in range(len(goal.path_x)):
                path_point = [goal.path_x[i], goal.path_y[i]]
                path.append(path_point)
            self.tracking.create_track(path)
            self.subscribe = rospy.Subscriber("EKF_location_pose", M_location_pose, self.analyse,  queue_size=10)
            rospy.Duration(2)
            #self._as.set_aborted()
        elif goal.action_type == 0:
            self.subscribe.unregister()

    def analyse(self, now_location):
        if self.control_frequence % 40 == 0:
            location = now_location.location_pose[:3]
            control_data = self.tracking.update(location)
            if control_data[0] == 0 and control_data[1] == 0:
                send_mov_cmd_client(1)
                self._result.result = 1
                self._as.set_succeeded(self._result)

            elif control_data[0] == 0 and control_data[1] != 0:
                if control_data[1] > 0:
                    send_mov_cmd_client(3, 0, -W_SPEED, abs(control_data[1]))
                else:
                    send_mov_cmd_client(3, 0, W_SPEED, abs(control_data[1]))

            elif control_data[0] != 0 and control_data[1] == 0:
                send_mov_cmd_client(2, MOVE_SPEED,0, control_data[0]*1000)
            self._feedback.percent = int(self.tracking.line_num / self.tracking.path_line_count)
            self._as.publish_feedback(self._feedback)
            rospy.loginfo(control_data)
        self.control_frequence += 1
        return



def Server_Init():
    tracking_server = Tracking_Server("tracking_server_node")
    return

if __name__ == '__main__':
    try:
        rospy.init_node('tracking_server_node', anonymous=True)
        Server_Init()
        send_mov_cmd_client(3, 0, W_SPEED, 30)
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass