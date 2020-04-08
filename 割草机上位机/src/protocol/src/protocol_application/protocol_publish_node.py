#!/usr/bin/env python
# license removed for brevity
import rospy
from msg_srv_act.msg import M_senser_data
from msg_srv_act.srv import S_answer_lower_machine
try:
    import protocol_process
except:
    from protocol_application import protocol_process

import copy

PROTOCOL_COM = "/dev/ttyUSB0"

def Senser_msg_add(data_list):
    gps_imu_data = M_senser_data()
    gps_imu_data.base_time =copy.deepcopy(data_list[0])
    gps_imu_data.imu_time = copy.deepcopy(data_list[1])
    gps_imu_data.pitch = copy.deepcopy(data_list[2])
    gps_imu_data.roll = copy.deepcopy(data_list[3])
    gps_imu_data.yaw = copy.deepcopy(data_list[4])
    gps_imu_data.acc_y = copy.deepcopy(data_list[5])
    gps_imu_data.gyo_x = copy.deepcopy(data_list[6])
    gps_imu_data.gyo_y = copy.deepcopy(data_list[7])
    gps_imu_data.gyo_z = copy.deepcopy(data_list[8])
    gps_imu_data.gps_time = copy.deepcopy(data_list[9])
    gps_imu_data.e_gps_state = copy.deepcopy(data_list[10])
    gps_imu_data.latitude = copy.deepcopy(data_list[11])
    gps_imu_data.e_latitude = copy.deepcopy(data_list[12])
    gps_imu_data.longitude = copy.deepcopy(data_list[13])
    gps_imu_data.e_longitude = copy.deepcopy(data_list[14])
    gps_imu_data.gps_speed = copy.deepcopy(data_list[15])
    gps_imu_data.e_gps_mode = copy.deepcopy(data_list[16])
    gps_imu_data.gps_rms = copy.deepcopy(data_list[17])
    gps_imu_data.odo_time = copy.deepcopy(data_list[18])
    gps_imu_data.odo_speed = copy.deepcopy(data_list[19])
    return gps_imu_data

def send_answer_client(service_type):
    rospy.wait_for_service('send_mov_cmd')
    try:
        send_mov_cmd = rospy.ServiceProxy('send_mov_cmd', S_answer_lower_machine)
        resp1 = send_mov_cmd(service_type)
        if resp1.answer:
            print(" answer successful")
        else:
            print(" answer false")
        return resp1.answer
    except rospy.ServiceException as exc:
        print("Service call failed!  because:" + str(exc))


def Publisher_Init():
    pub_senser = rospy.Publisher('lower_machine_senser_data', M_senser_data, queue_size=10)
    rospy.loginfo("the protocol_process_publisher is ready")
    try:
        Usart_protocol = protocol_process.Protocol_Usart(PROTOCOL_COM)
    except:
        print("com isn't open")
        exit()

    while not rospy.is_shutdown():
        receive_data = Usart_protocol.Com.read(500)

        if receive_data != b'':
            Usart_protocol.Analyse(receive_data)
            if (Usart_protocol.receive_data_tpye == 2) or (Usart_protocol.receive_data_tpye == 3)or \
                    (Usart_protocol.receive_data_tpye == 4):
                gps_imu_data = Senser_msg_add(Usart_protocol.receive_data)
                rospy.logdebug(Usart_protocol.receive_data)
                pub_senser.publish(gps_imu_data)

                # The lower computer stops sending data and requests the upper computer to stop corresponding actions
                if Usart_protocol.answer == 1:
                    # Heading synchronization stop
                    if Usart_protocol.receive_data_tpye == 2:
                        a = 0
                    # Coordinate synchronization stop
                    elif Usart_protocol.receive_data_tpye == 3:
                        a = 0
                    # Real-time positioning stop
                    elif Usart_protocol.receive_data_tpye == 4:
                        a = 0
            elif Usart_protocol.receive_data_tpye == 6:
                server_on = rospy.wait_for_service('answer_with_lower_machine',timeout=1)
                if server_on == False:
                    rospy.loginfo("the answer server not started")
                    continue
                try:
                    answer = rospy.ServiceProxy('answer_with_lower_machine', S_answer_lower_machine)
                    resp1 = answer(0)# receive the answer
                    if resp1.answer:
                        print("lower machine has answered")
                except rospy.ServiceException as exc:
                    print("Service call failed!  because:" + str(exc))




if __name__ == '__main__':
    try:
        rospy.init_node('protocol_process_publisher_node', anonymous=True)
        Publisher_Init()
        rospy.spin()
    except rospy.ROSInterruptException:
        print(__file__)
        pass
