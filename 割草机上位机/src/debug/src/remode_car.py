import cv2
import rospy
import numpy as np
import time
import protocol_client_test
import serial
import socket


SPEED_FRADE = 0
BLUETOOTH_COM = "/dev/ttyUSB1"

def Remote_Mov_Forward():
    protocol_client_test.send_remote_cmd_client(6, 100+30*(SPEED_FRADE+1), 0, 1000)
    return
def Remote_Mov_Backward():
    protocol_client_test.send_remote_cmd_client(6, -(100+30*(SPEED_FRADE+1)), 0, 1000)
    return
def Remote_Turn_Left():
    protocol_client_test.send_remote_cmd_client(7, 0, 0.8, 90)
    return
def Remote_Turn_Right():
    protocol_client_test.send_remote_cmd_client(7, 0, -0.8, 90)
    return
def Remote_Stop():
    protocol_client_test.send_remote_cmd_client(4,0,0,0)
    return

def Remote_Brake():
    protocol_client_test.send_remote_cmd_client(5,0,0,0)
    return

def PC_Remode_Car():
    cv2.namedWindow("Remote the car",cv2.WINDOW_AUTOSIZE)
    canvas = np.full((240,480),int(255),dtype=np.uint8)
    cv2.imshow("Remote the car",canvas)
    while 1:
        key_value = cv2.waitKey()
        if key_value == 119: # w
            Remote_Mov_Forward()
        elif key_value == 115:# s
            Remote_Mov_Backward()
        elif key_value == 97: # a
            Remote_Turn_Left()
        elif key_value == 100: # d
            Remote_Turn_Right()
        elif key_value == 32:# space
            Remote_Stop()
        elif key_value == 27:
            break
        time.sleep(0.1)

def Bluetooth_Remote_Car():
    try:
        Com = serial.Serial(BLUETOOTH_COM, 115200, timeout=0.00001)
    except:
        if Com.is_open == False:
            print("usart con't open")
        else:
            print("usart is open")
    while 1:
        recv_data = Com.read(1)
        if recv_data == b'w':
            Remote_Mov_Forward()
        elif recv_data == b's':
            Remote_Mov_Backward()
        elif recv_data == b'a':
            Remote_Turn_Left()
        elif recv_data == b'd':
            Remote_Turn_Right()
        elif recv_data == b'p':
            Remote_Stop()
        elif recv_data == b'q':
            break
        time.sleep(0.1)

def WIFI_Remote_Car():
    global SPEED_FRADE
    port = 12345
    buff_size = 1024

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server.bind((ip,port))
    print("ip:",ip,"port:",port)
    server.listen(1)

    clientsock,clientaddress = server.accept()
    print("connect from",clientaddress)
    while 1:
        recv_data = clientsock.recv(buff_size).decode('utf-8')
        #print(recv_data)
        if recv_data == 'US':
            Remote_Mov_Forward()
        elif recv_data == 'DS':
            Remote_Mov_Backward()
        elif recv_data == 'LS':
            Remote_Turn_Left()
        elif recv_data == 'RS':
            Remote_Turn_Right()
        elif recv_data == 'AA':
            Remote_Stop()
        elif recv_data == 'DD':
            Remote_Brake()
        elif recv_data == 'SP0':
            SPEED_FRADE = 0
        elif recv_data == 'SP1':
            SPEED_FRADE = 1
        elif recv_data == 'SP2':
            SPEED_FRADE = 2
        elif recv_data == 'SP3':
            SPEED_FRADE = 3
        elif recv_data == 'SP4':
            SPEED_FRADE = 4
        elif recv_data == 'SP5':
            SPEED_FRADE = 5
        elif recv_data == 'SP6':
            SPEED_FRADE = 6
        elif recv_data == 'SP7':
            SPEED_FRADE = 7
        elif recv_data == 'SP8':
            SPEED_FRADE = 8
        elif recv_data == 'SP9':
            SPEED_FRADE = 9
        elif recv_data == 'BB':
            clientsock.close()
            server.close()
            break
        elif len(recv_data) == 0:
            clientsock.close()
            server.close()
            print(clientaddress,": disconnect!")
            break
        #time.sleep(0.1)
    clientsock.close()
    server.close()



if __name__ == "__main__":
    #PC_Remode_Car()
    #Bluetooth_Remote_Car()
    WIFI_Remote_Car()