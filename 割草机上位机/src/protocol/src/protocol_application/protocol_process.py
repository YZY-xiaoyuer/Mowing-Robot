#!/usr/bin/env python
# license removed for brevity
import serial
import struct
import binascii
try:
    import crc
except:
    from protocol_application import crc
import copy
import time

class Protocol_Usart(object):
    def __init__(self, com):
        try:
            self.Com = serial.Serial(com, 460800, timeout=0.00001)
        except:
            if self.Com.is_open == False:
                print("usart con't open")
            else:
                print("usart is open")
        self.receive_data_tpye = 0  #reference communication protocol
        self.receive_data = []
        self.answer = 0 # 0 needless to answer ;1 need to answer

    def Send_Answer_To_Lower(self):
        package_head = 85
        package_tail = 170
        package_answer = 0
        send_head = struct.pack("<1B", package_head)
        send_tail = struct.pack("<1B", package_tail)
        package_len = 9
        package_type = 6
        send_check_crc_data = struct.pack("<1I1B1B", package_len, package_type, package_answer)
        hex_crc_data = binascii.hexlify(send_check_crc_data)
        crc8 = crc.CRCGenerator()
        cal_crc = crc8.create(hex_crc_data)
        send_crc = struct.pack("<1B", cal_crc)

        send_data = send_head + send_check_crc_data + send_crc + send_tail
        if self.Com.write(send_data):
            return 0
        else:
            return 1

    def Send_Miving_Cmd(self, cmd_id,cmd_type, v, w, s_or_angle):
        package_head = 85
        package_tail = 170
        package_answer = 1
        send_head = struct.pack("<1B", package_head)
        send_tail = struct.pack("<1B", package_tail)
        package_len = 21
        package_type = 1
        send_check_crc_data = struct.pack("<1I1B1B1B1h1f1f1B", package_len, package_type, cmd_id, (cmd_type), (v), (w), (s_or_angle), package_answer)
        hex_crc_data = binascii.hexlify(send_check_crc_data)
        crc8 = crc.CRCGenerator()
        cal_crc = crc8.create(hex_crc_data)
        send_crc = struct.pack("<1B", cal_crc)

        send_data = send_head + send_check_crc_data + send_crc + send_tail
        if self.Com.write(send_data):
            return 0
        else:
            return 1

    def Send_Remote_Cmd(self, cmd_id,cmd_type, v, w, s_or_angle):
        package_head = 85
        package_tail = 170
        package_answer = 1
        send_head = struct.pack("<1B", package_head)
        send_tail = struct.pack("<1B", package_tail)
        package_len = 21
        package_type = 7
        send_check_crc_data = struct.pack("<1I1B1B1B1h1f1f1B", package_len, package_type, cmd_id, (cmd_type), (v), (w), (s_or_angle), package_answer)
        hex_crc_data = binascii.hexlify(send_check_crc_data)
        crc8 = crc.CRCGenerator()
        cal_crc = crc8.create(hex_crc_data)
        send_crc = struct.pack("<1B", cal_crc)

        send_data = send_head + send_check_crc_data + send_crc + send_tail
        if self.Com.write(send_data):
            return 0
        else:
            return 1

    def Send_Location_Cmd(self, server_type,server_state):
        package_head = 85
        package_tail = 170
        package_answer = 1
        send_head = struct.pack("<1B", package_head)
        send_tail = struct.pack("<1B", package_tail)
        package_len = 10
        if server_type == 0:
            package_type = 2
        elif server_type == 1:
            package_type = 3
        elif server_type == 2:
            package_type = 4

        package_cmd = server_state
        send_check_crc_data = struct.pack("<1I1B1B1B", package_len, package_type, package_cmd, package_answer)
        hex_crc_data = binascii.hexlify(send_check_crc_data)
        crc8 = crc.CRCGenerator()
        cal_crc = crc8.create(hex_crc_data)
        send_crc = struct.pack("<1B", cal_crc)

        send_data = send_head + send_check_crc_data + send_crc + send_tail
        if self.Com.write(send_data):
            return 0
        else:
            return 1



    def Analyse(self, receive_data):
        self.receive_data_tpye = 0
        receive_package_head = copy.deepcopy(receive_data[:6])
        receive_package_tail = copy.deepcopy(receive_data[-3:])
        receive_crc_data = copy.deepcopy(receive_data[1:-2])
        hex_crc_data = binascii.hexlify(receive_crc_data)
        crc8 = crc.CRCGenerator()
        cal_crc = crc8.create(hex_crc_data)
        if len(receive_package_tail) == 3:
            package_answer_flag, package_crc_data, package_tail = struct.unpack("<1B1B1B", receive_package_tail)
            if cal_crc == package_crc_data:
                if len(receive_package_head) == 6:
                    package_head, package_len, protocol_type = struct.unpack("<1B1I1B", receive_package_head)

                    if protocol_type == 2 or protocol_type == 3 or protocol_type == 4:
                        analyse_data = receive_data[6:-3]
                        if len(analyse_data) == 102:
                            self.receive_data_tpye = protocol_type
                            data_list = [0 for i in range(21)]
                            data_list[0], data_list[1], data_list[2], data_list[3], data_list[4], data_list[5], data_list[6], \
                            data_list[7], data_list[8], data_list[9], data_list[10], data_list[11], data_list[12], data_list[13], \
                            data_list[14], data_list[15], data_list[16], data_list[17], data_list[18], data_list[19]\
                                = struct.unpack("<1I1I1d1d1d1d1d1d1d1I1B1d1B1d1B1f1B1H1I1i", analyse_data)
                            self.receive_data = copy.deepcopy(data_list)
                    if protocol_type == 6:
                        self.receive_data_tpye = protocol_type


def Analyse_data():
    com = "/dev/ttyUSB0"
    protocol = Protocol_Usart(com)
    try:
        serial.Serial(com, 460800, timeout=0.00001)
    except:
        print("usart con't open")

    if protocol.Com.is_open == False:
        protocol.Com.open()
    while 1:
        receive_data = protocol.Com.read(500)
        if receive_data != b'':
            print(receive_data)
            protocol.Analyse(receive_data)

def Send_data():
    com = "/dev/ttyUSB0"
    protocol = Protocol_Usart(com)
    while 1:
        try:
            serial.Serial(com, 460800, timeout=0.00001)
        except:
            print("usart con't open")
        protocol.Send_Miving_Cmd(0, 20, 30, 700)
        time.sleep(0.1)


def test1():
    com = "/dev/ttyUSB0"
    protocol = Protocol_Usart(com)
    start = time.time()
    protocol.Send_Miving_Cmd(0, 20, 30, 700)
    end = time.time()
    print(end-start)

if __name__ == '__main__':
    #test1()
    Send_data()
    #Analyse_data()
