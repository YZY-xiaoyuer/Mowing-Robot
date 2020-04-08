#!/usr/bin/env python
# license removed for brevity

# 高斯坐标正反算算法

import math

P = 180 / math.pi * 3600  # 1弧度的秒数常数值，1弧度=180/pi=57.29578*3600=206264.80624709636

#def Get_WGS84(B, L):


def GetGSZS54(B, L):  # 北京54克拉索夫斯基椭球参数  B,L 为十进制度，非度分秒格式，python中math.sin 和.cos函数仅支持弧度，需要用math.radians将度数转换为弧度。
    # 北京54椭球高斯正算
    l = (L - Get6DL0(L)[0]) * 3600 / P
    COS2B = math.pow(math.cos(math.radians(B)), 2)
    N = 6399698.902 - (21562.267 - (108.973 - 0.612 * COS2B) * COS2B) * COS2B
    A0 = 32140.404 - (135.3302 - (0.7092 - 0.0040 * COS2B) * COS2B) * COS2B
    A4 = (0.25 + 0.00252 * COS2B) * COS2B - 0.04166
    A6 = (0.166 * COS2B - 0.084) * COS2B
    A3 = (0.3333333 + 0.001123 * COS2B) * COS2B - 0.1666667
    A5 = 0.0083 - (0.1667 - (0.1968 + 0.0040 * COS2B) * COS2B) * COS2B

    x = 6367558.4969 * B * 3600 / P - (
                A0 - (0.5 + (A4 + A6 * math.pow(l, 2)) * math.pow(l, 2)) * math.pow(l, 2) * N) * math.sin(
        math.radians(B)) * math.cos(math.radians(B))
    y = (1 + (A3 + A5 * math.pow(l, 2)) * math.pow(l, 2)) * l * N * math.cos(math.radians(B)) + Get3DL0(L)[
        1] * 1000000 + 500000  # 坐标加3度带带号，y坐标西移500KM
    '''
    x_str = str(x)
    y_str = str(y)
    x_str = x_str[2:]
    y_str = y_str[2:]
    x = float(x_str)
    y = float(y_str)
    '''
    return (x, y)

def LatLon2XY(latitude, longitude, H):
    f = 1 / 298.257223563
    a = 6378137
    b = a * (1 - f)
    e = math.sqrt(a * a - b * b) / a
    N = a / math.sqrt(1 - e * e * math.sin(latitude * math.pi / 180) * math.sin(latitude * math.pi / 180))
    WGS84_X = (N + H ) * math.cos(latitude * math.pi / 180) * math.cos(longitude * math.pi / 180)
    WGS84_Y = (N + H ) * math.cos(latitude * math.pi / 180) * math.sin(longitude * math.pi / 180)
    return WGS84_X, WGS84_Y

def GetWGS84(B,L):

    Datum = 84# 投影基准面类型：北京54基准面为54，西安80基准面为80，WGS84基准面为84
    prjno = 0# 投影带号
    zonewide = 3
    IPI = 0.0174532925199433333333# 3.1415926535898 / 180.0
    if zonewide == 6:
        prjno = int(L/zonewide)+1
        L0 = prjno*zonewide-3
    else:
        prjno = int((L-1.5)/3) + 1
        L0 = prjno*3
    if Datum == 54:
        a = 6378245
        f = 1/298.3
    elif Datum == 84:
        a = 6378137
        f = 1/298.257223563
    L0 = L0*IPI
    L = L*IPI
    B = B*IPI
    e2 = 2*f - f*f
    l = L-L0
    t = math.tan(B)
    m = l*math.cos(B)
    N = a/math.sqrt(1-e2*math.cos(B)*math.sin(B))
    q2=e2/(1-e2)* math.cos(B)* math.cos(B)
    a1=1+ 3/4*e2+45/64*e2*e2+175/256*e2*e2*e2+11025/16384*e2*e2*e2*e2+43659/65536*e2*e2*e2*e2*e2
    a2=3/4*e2+5/16*e2*e2+25/512*e2*e2*e2+2205/2048*e2*e2*e2*e2+72765/65536*e2*e2*e2*e2*e2
    a3=15/64*e2*e2+105/256*e2*e2*e2+2205/4096*e2*e2*e2*e2+10359/16384*e2*e2*e2*e2*e2
    a4=35/512*e2*e2*e2+315/2048*e2*e2*e2*e2+31185/13072*e2*e2*e2*e2*e2
    b1=a1*a*(1-e2)
    b2=-1/2*a2*a*(1-e2)
    b3=1/4*a3*a*(1-e2)
    b4=-1/6*a4*a*(1-e2)
    c0=b1
    c1=2*b2+4*b3+6*b4
    c2=-(8*b3+32*b4)
    c3=32*b4
    s=c0*B+math.cos(B)*(c1*math.sin(B)+c2*math.sin(B)*math.sin(B)*math.sin(B)+c3*math.sin(B)*math.sin(B)*math.sin(B)*math.sin(B)*math.sin(B))
    x =s+1/2*N*t*m*m+1/24*(5-t*t+9*q2+4*q2*q2)*N*t*m*m*m*m+1/720*(61-58*t*t+t*t*t*t)*N*t*m*m*m*m*m*m
    y =N*m+1/6*(1-t*t+q2)*N*m*m*m+1/120*(5-18*t*t+t*t*t*t-14*q2-58*q2*t*t)*N*m*m*m*m*m
    y =y+1000000*prjno+500000
    x_str = str(x)
    y_str = str(y)
    x_str = x_str[2:]
    y_str = y_str[2:]
    x = float(x_str)
    y = float(y_str)
    return x,y



def GetGSZS80(B, L):  # 西安80  国际1975国际椭球参数
    # 西安80国际1975椭球高斯正算
    l = (L - Get3DL0(L)[0]) * 3600 / P
    COS2B = math.pow(math.cos(math.radians(B)), 2)
    N = 6399596.652 - (21565.045 - (108.996 - 0.603 * COS2B) * COS2B) * COS2B
    A0 = 32144.5189 - (135.3646 - (0.7034 - 0.0041 * COS2B) * COS2B) * COS2B
    A4 = (0.25 + 0.00253 * COS2B) * COS2B - 0.04167
    A6 = (0.167 * COS2B - 0.083) * COS2B
    A3 = (0.3333333 + 0.001123 * COS2B) * COS2B - 0.1666667
    A5 = 0.00878 - (0.1702 - 0.20382 * COS2B) * COS2B

    x = 6367452.1328 * B * 3600 / P - (
                A0 - (0.5 + (A4 + A6 * math.pow(l, 2)) * math.pow(l, 2)) * math.pow(l, 2) * N) * math.cos(
        math.radians(B)) * math.sin(math.radians(B))
    y = (1 + (A3 + A5 * math.pow(l, 2)) * math.pow(l, 2)) * l * N * math.cos(math.radians(B)) + Get3DL0(L)[
        1] * 1000000 + 500000  # 坐标加3度带带号，y坐标西移500KM

    return (x, y)


def GetGSFS54(x, y, L0):
    # 北京54椭球高斯反算
    y = y - L0 / 3 * 1000000 - 500000  # 去掉带号和y坐标恢复东移500KM
    BT = x / 6367558.4969  # BT=x/6367558.4969*P 得到是秒数，不乘P直接得到弧度数值
    bt = x / 6367558.4969 * P  # 得到是秒数
    COS2BT = math.pow(math.cos(BT), 2)
    BF = (bt + (50221746 + (293622 + (2350 + 22 * COS2BT) * COS2BT) * COS2BT) * math.pow(10, -10) * math.cos(
        BT) * math.sin(BT) * P) / P
    COS2BF = math.pow(math.cos(BF), 2)
    NF = 6399698.902 - (21562.267 - (108.973 - 0.612 * COS2BF) * COS2BF) * COS2BF
    Z = y / (NF * math.cos(BF))
    B2 = (0.5 + 0.003369 * COS2BF) * math.sin(BF) * math.cos(BF)
    B3 = 0.333333 - (0.166667 - 0.001123 * COS2BF) * COS2BF
    B4 = 0.25 + (0.16161 + 0.00562 * COS2BF) * COS2BF
    B5 = 0.2 - (0.1667 - 0.0088 * COS2BF) * COS2BF

    B = (BF * P - (1 - (B4 - 0.12 * math.pow(Z, 2)) * math.pow(Z, 2)) * math.pow(Z, 2) * B2 * P) / 3600.0
    l = ((1 - (B3 - B5 * math.pow(Z, 2)) * math.pow(Z, 2)) * Z * P) / 3600.0
    L = L0 + l

    return (B, L)


def GetGSFS80(x, y, L0):
    y = y - L0 / 3 * 1000000 - 500000  # 去掉带号和y坐标恢复东移500KM
    BT = x / 6367452.133  # BT=x/6367558.4969*P 得到是秒数，不乘P直接得到弧度数值
    bt = x / 6367452.133 * P  # 得到是秒数
    COS2BT = math.pow(math.cos(BT), 2)
    BF = (bt + (50228976 + (293697 + (2383 + 22 * COS2BT) * COS2BT) * COS2BT) * math.pow(10, -10) * math.cos(
        BT) * math.sin(BT) * P) / P
    COS2BF = math.pow(math.cos(BF), 2)

    NF = 6399596.652 - (21565.045 - (108.996 - 0.603 * COS2BF) * COS2BF) * COS2BF

    Z = y / (NF * math.cos(BF))
    B2 = (0.5 + 0.00336975 * COS2BF) * math.sin(BF) * math.cos(BF)
    B3 = 0.333333 - (0.1666667 - 0.001123 * COS2BF) * COS2BF
    B4 = 0.25 + (0.161612 + 0.005617 * COS2BF) * COS2BF
    B5 = 0.2 - (0.16667 - 0.00878 * COS2BF) * COS2BF

    B = (BF * P - (1 - (B4 - 0.147 * math.pow(Z, 2)) * math.pow(Z, 2)) * math.pow(Z, 2) * B2 * P) / 3600.0
    l = ((1 - (B3 - B5 * math.pow(Z, 2)) * math.pow(Z, 2)) * Z * P) / 3600.0
    L = L0 + l
    return (B, L)


def Get3DL0(L):  # 计算已知经度的3度分带中央经线和带号
    H = math.trunc((L + 1.5) / 3)  # 带号
    L0 = H * 3  # 中央经线
    return (L0, H)


def Get6DL0(L):  # 计算已知经度的6度分带中央经线和带号
    H = math.trunc((L + 6) / 6)  # 带号
    L0 = H * 6 - 3
    return (L0, H)

if __name__ == '__main__':
    # 试验数据
    B1 = 22.625580
    L1 = 144.0391625
    B2 = 22.6255467
    L2 = 144.03913133

    x1,y1 = GetGSZS54(B1, L1)
    x2, y2 = GetGSZS54(B2, L2)
    print(x1,y1)
    print(x2,y2 )
    #print(math.sqrt((x1-x2)**2+(y1-y2)**2))
