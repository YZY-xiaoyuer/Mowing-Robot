#!/usr/bin/env python
# license removed for brevity
import rospy
from msg_srv_act.srv import S_mov_control_cmd,S_mov_control_cmdResponse,S_answer_lower_machine, S_answer_lower_machineResponse
from msg_srv_act.srv import S_location_server, S_location_serverResponse
try:
    import protocol_process
except:
    from protocol_application import protocol_process

lower_machine_answer = True
usart_using = False
moving_cmd_num = 0
remote_cmd_num = 0

RESEND_WAIT_TIME = 100  #hz
RESEND_TIMES = 10
RECEIVE_WAIT_ANSWER_TIME = 100 # hz
RECEIVE_WAIT_ANSWER_TIMES = 5

PROTOCOL_COM = "/dev/ttyUSB0"

def Send_Remote_Cmd(req):
    global usart_using
    global lower_machine_answer
    global remote_cmd_num
    rate_wait = rospy.Rate(RESEND_WAIT_TIME)
    Usart_protocol = protocol_process.Protocol_Usart(PROTOCOL_COM)
    # the number of wait the usart
    for i in range(RESEND_TIMES):
        if usart_using == False:
            usart_using = True
            lower_machine_answer = False
            rate_again = rospy.Rate(RECEIVE_WAIT_ANSWER_TIME)
            # number of data resends
            Usart_protocol.Send_Remote_Cmd(remote_cmd_num, req.moving_cmd_type, req.v, req.w, req.s_or_angle)

            for j in range(RECEIVE_WAIT_ANSWER_TIMES):
                rate_again.sleep()
                if lower_machine_answer == True:
                    remote_cmd_num +=1
                    if remote_cmd_num >= 255:
                        remote_cmd_num = 0
                    usart_using = False
                    return S_mov_control_cmdResponse(1)
                else:
                    rospy.logdebug("the lower machine didn't answer.")
        rate_wait.sleep()
        lower_machine_answer = True
        usart_using = False
    return S_mov_control_cmdResponse(0)

def Send_Moving_Control_Cmd(req):
    global usart_using
    global lower_machine_answer
    global moving_cmd_num
    rate_wait = rospy.Rate(RESEND_WAIT_TIME)
    Usart_protocol = protocol_process.Protocol_Usart(PROTOCOL_COM)
    # the number of wait the usart
    for i in range(RESEND_TIMES):
        if usart_using == False:
            usart_using = True
            lower_machine_answer = False
            rate_again = rospy.Rate(RECEIVE_WAIT_ANSWER_TIME)
            # number of data resends
            Usart_protocol.Send_Miving_Cmd(moving_cmd_num, req.moving_cmd_type, req.v, req.w, req.s_or_angle)

            for j in range(RECEIVE_WAIT_ANSWER_TIMES):
                rate_again.sleep()
                if lower_machine_answer == True:
                    moving_cmd_num +=1
                    if moving_cmd_num >= 255:
                        moving_cmd_num = 0
                    usart_using = False
                    return S_mov_control_cmdResponse(1)
                else:
                    rospy.logdebug("the lower machine didn't answer.")
        rate_wait.sleep()
        lower_machine_answer = True
        usart_using = False
    return S_mov_control_cmdResponse(0)

def Send_Location_Cmd(req):
    global usart_using
    global lower_machine_answer
    rate_wait = rospy.Rate(RESEND_WAIT_TIME)
    Usart_protocol = protocol_process.Protocol_Usart(PROTOCOL_COM)
    # the number of wait the usart
    for i in range(RESEND_TIMES):
        if usart_using == False:
            usart_using = True
            lower_machine_answer = False
            rate_again = rospy.Rate(RECEIVE_WAIT_ANSWER_TIME)
            # number of data resends
            Usart_protocol.Send_Location_Cmd(req.server_type, req.server_state)

            for j in range(RECEIVE_WAIT_ANSWER_TIMES):
                rate_again.sleep()
                if lower_machine_answer == True:
                    usart_using = False
                    return S_location_serverResponse(1)
                else:
                    rospy.logdebug("the lower machine didn't answer.")
        rate_wait.sleep()
        lower_machine_answer = True
        usart_using = False
    return S_location_serverResponse(0)


def Answer_With_Lower_Machine(req):
    if req.service_type == 0:
        global lower_machine_answer
        lower_machine_answer = True
        rospy.logdebug("the lower machine is answer")
        return S_answer_lower_machineResponse(1)
    elif req.service_type == 1:
        global usart_using
        usart_using = False
        Usart_protocol = protocol_process.Protocol_Usart(PROTOCOL_COM)
        Usart_protocol.Send_Answer_To_Lower()
        usart_using = False
        return S_answer_lower_machineResponse(1)
    return S_answer_lower_machineResponse(0)


def Server_Init():
    cmd_server = rospy.Service('send_mov_cmd', S_mov_control_cmd, Send_Moving_Control_Cmd)
    remote_cmd_server = rospy.Service('send_remote_cmd', S_mov_control_cmd, Send_Remote_Cmd)
    answer_server = rospy.Service('answer_with_lower_machine', S_answer_lower_machine, Answer_With_Lower_Machine)
    location_server =  rospy.Service('send_location_server', S_location_server, Send_Location_Cmd)
    rospy.loginfo("the protocol_cmd_server is ready.")

if __name__ == '__main__':
    try:
        rospy.init_node("protocol_cmd_server_node", anonymous=True)
        Server_Init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
