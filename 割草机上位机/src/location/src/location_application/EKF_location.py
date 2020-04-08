#!/usr/bin/env python3
# license removed for brevity

import numpy as np
import math
try:
    import EKF
except:
    from location_application import EKF
try:
    import gauss_projection as gp
except:
    from location_application import gauss_projection as gp

GPS_AND_LOCATION_ERROR = 20 # Maximum error of gps and ekf positioning (m)




def motion_model_v_yaw (step_time,X , u_now):
    # x = [x,y,yaw,v]
    # u = [v,yaw]
    yaw = math.radians(X[2, 0])

    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[math.cos(yaw)*step_time, 0],
                  [math.sin(yaw)*step_time, 0],
                  [0, 1],
                  [1, 0]])

    x = F @ X + B @ u_now
    x[2, 0] += 360
    x[2, 0] = x[2, 0] % 360
    return x

def motion_model_v_w1_w2 (step_time,X , u_now):
    # x = [x,y,yaw,pitch,v]
    # u = [v,w_yaw,w_pitch]

    yaw = math.radians(X[2, 0])
    pitch = math.radians(X[3, 0])

    F = np.array([[1, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0],
                  [0, 0, 1, 0, 0],
                  [0, 0, 0, 1, 0],
                  [0, 0, 0, 0, 0]
                  ])

    B = np.array([[math.cos(yaw)*step_time*math.cos(pitch), 0, 0],
                  [math.sin(yaw)*step_time*math.cos(pitch), 0, 0],
                  [0, step_time, 0],
                  [0, 0, step_time],
                  [1, 0, 0]
                  ])

    x = F @ X + B @ u_now
    if x[3, 0] >= 180:
        x[3, 0] = x[3, 0] - 360
    elif x[3, 0] < -180:
        x[3, 0] = x[3, 0] + 360
    return x

def motion_model_v_w (step_time,X , u_now):
    # x = [x,y,yaw,v]
    # u = [v,w]
    yaw = np.deg2rad(X[2, 0])

    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 0]])

    B = np.array([[math.cos(yaw)*step_time, 0],
                  [math.sin(yaw)*step_time, 0],
                  [0, step_time],
                  [1, 0]])

    x = F @ X + B @ u_now
    x[2, 0] %= 360
    return x

def motion_model_a_w(step_time,X , u_now):
    # x = [x,y,yaw,v]
    # u = [a,w]
    yaw = math.radians(X[2, 0])

    F = np.array([[1.0, 0, 0, step_time * math.cos(yaw)],
                  [0, 1.0, 0, step_time * math.sin(yaw)],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 1]])

    B = np.array([[1 / 2 * (step_time ** 2) * math.cos(yaw), 0],
                  [1 / 2 * (step_time ** 2) * math.sin(yaw), 0],
                  [0, step_time],
                  [step_time, 0]])

    x = F @ X + B @ u_now
    x[2, 0] = x[2, 0] % 360
    return x

def motion_model_a_yaw(step_time, X, u_now):
    yaw = math.radians(X[2, 0])

    F = np.array([[1.0, 0, 0, math.cos(yaw)*step_time],
                  [0, 1.0, 0, math.sin(yaw)*step_time],
                  [0, 0, 0, 0],
                  [0, 0, 0, 1]])

    B = np.array([[1/2*(step_time**2)*math.cos(yaw), 0],
                  [1/2*(step_time**2)*math.sin(yaw), 0],
                  [0.0, 1],
                  [step_time, 0.0]])

    x = F @ X + B @ u_now
    x[2, 0] = x[2, 0]%360
    return x

def observation_model_gps(X):
    # Observation Model
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])
    z = H @ X
    return z

def observation_model_imu_yaw(X):
    # Observation Model
    H = np.array([
        [0, 0, 1, 0],
    ])
    z = H @ X
    return z

def observation_model_imu_yaw_pitch(X):
    # Observation Model
    H = np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
    ])
    z = H @ X
    return z

def observation_model_odo_v(X):
    # Observation Model
    H = np.array([
        [0, 0, 0, 1],
    ])
    z = H @ X
    return z
def jacobF(step_time, X, U = []):
    """
    Jacobian of Motion Model
    """
    yaw = math.radians(X[2, 0])
    if U != []:
        v = U[0, 0]
    else:
        v = X[3, 0]
    jF = np.array([
        [1.0, 0.0, -step_time * v * math.sin(yaw), step_time * math.cos(yaw)],
        [0.0, 1.0, step_time * v * math.cos(yaw), step_time * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF

def jacobF_yaw_pitch(step_time, X, U = []):
    """
    Jacobian of Motion Model
    """
    yaw = math.radians(X[2, 0])
    pitch = math.radians(X[3, 0])
    if U != []:
        v = U[0, 0]
    else:
        v = X[3, 0]
    jF = np.array([
        [1, 0, -step_time * v * math.sin(yaw)*math.cos(pitch), -v*step_time*math.cos(yaw)*math.sin(pitch), step_time * math.cos(yaw)*math.cos(pitch)],
        [0, 1, step_time * v * math.cos(yaw)*math.cos(pitch), -v*step_time*math.sin(yaw)*math.sin(pitch), step_time * math.sin(yaw)*math.cos(pitch)],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1]
    ])

    return jF

def jacobH_x_y_v(input = []):
    """
    Jacobian of observation Model
    """
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1]
    ])

    return jH


def jacobH_yaw(input = []):
    """
    Jacobian of observation Model
    """
    jH = np.array([
        [0, 0, 1, 0],
    ])

    return jH

def jacobH_yaw_pitch(input = []):
    """
    Jacobian of observation Model
    """
    jH = np.array([
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
    ])

    return jH

def jacobH_v(input = []):
    """
    Jacobian of observation Model
    """
    jH = np.array([
        [0, 0, 0, 1],
    ])

    return jH


def Caculate_ACC_Level(level, now_data, variance):
    p = 0.7
    q = 1 - p
    if abs(now_data-level) < variance:
        level = p*level + q*now_data
    out_put = now_data-level
    return level, out_put

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

def Get_Var(a,w):
    a_var = np.var(a)
    w_var = np.var(w)
    return a_var, w_var

def Get_Avg(a,w):
    a_avg = np.average(a)
    w_avg = np.average(w)
    return a_avg, w_avg

class EKF_Location(object):
    def __init__(self):
        self.location_module = 0  # 0 is none do ;  1 is Coordinate Alignment ;2 is heading coordinate ;3 is location
        self.x_offset = 0
        self.y_offset = 0
        self.yaw_offset = 0
        self.pitch_offset = 0
        self.origin_gps_yaw = 0
        self.a_static_var = 0
        self.w_static_var = 0
        self.a_static_avg = 0
        self.w_static_avg = 0
        self.last_time = 0
        self.a_level = 0
        self.w_yaw_level = 0
        self.w_level = 0
        self.gps_observation_frequency = 1
        self.gps_position_offset = 0.2  # the distance from the center of mahine to GPS position  (m)
        self.pridict_module = 4 # 0:[a, w ]; 1: [v, w] ;2:[v, yaw] ;3: [a ,yaw] 4: [v, w_yaw, wddddd_pitch]
        if self.pridict_module == 0:
            self.ekf_location = EKF.EKF(4, np.array([[0, 0, 0, 0]]).T, np.diag([0.01, 0.01, np.deg2rad(1), 0.01])**2,
                                        motion_model_a_w)
        elif self.pridict_module == 1:
            self.ekf_location = EKF.EKF(4, np.array([[0, 0, 0, 0]]).T, np.diag([0.01, 0.01, np.deg2rad(1), 0.01]) ** 2,
                                        motion_model_v_w)
        elif self.pridict_module == 2:
            self.ekf_location = EKF.EKF(4, np.array([[0, 0, 0, 0]]).T, np.diag([0.01, 0.01, np.deg2rad(1), 0.01]) ** 2,
                                        motion_model_v_yaw)
        elif self.pridict_module == 3:
            self.ekf_location = EKF.EKF(4, np.array([[0, 0, 0, 0]]).T, np.diag([0.01, 0.01, np.deg2rad(1), 0.01]) ** 2,
                                        motion_model_a_yaw)
        elif self.pridict_module == 4:
            self.ekf_location = EKF.EKF(4, np.array([[0, 0, 0, 0, 0]]).T, np.diag([0.01, 0.01, np.deg2rad(1), np.deg2rad(1), 0.01]) ** 2,
                                        motion_model_v_w1_w2)


    def Calculate_x_y(self, B, L):


        b_split = math.modf(B / 100)
        l_split = math.modf(L / 100)
        B = float(b_split[1]) + float(b_split[0] * 100) / 60
        L = float(l_split[1]) + float(l_split[0] * 100) / 60
        x, y = gp.GetGSZS54(B, L)
        return x, y


    def Location_Realtime(self, senser_data):
        base_time = senser_data.base_time
        imu_time = senser_data.imu_time
        imu_pitch = senser_data.pitch
        imu_roll = senser_data.roll
        imu_yaw = senser_data.yaw
        imu_a = senser_data.acc_y
        imu_w_pitch = -senser_data.gyo_x
        imu_w_roll = senser_data.gyo_y
        imu_w_yaw = -senser_data.gyo_z
        gps_time = senser_data.gps_time
        gps_state = senser_data.e_gps_state
        gps_B = senser_data.latitude
        gps_B_state = senser_data.e_latitude
        gps_L = senser_data.longitude
        gps_L_state = senser_data.e_longitude
        gps_speed = senser_data.gps_speed
        gps_mode = senser_data.e_gps_mode
        gps_rms = senser_data.gps_rms
        odo_time = senser_data.odo_time
        odo_speed = senser_data.odo_speed
        #print(imu_pitch,imu_yaw,imu_w_pitch,imu_w_yaw)

        if self.last_time == 0:
            self.last_time = base_time
            X_now = self.ekf_location.get_state()
            X_now[2, 0] = imu_yaw
            X_now[3, 0] = imu_pitch
            self.ekf_location.set_state(X_now)
            return
        else:
            if self.pridict_module == 0:
                if imu_time >= base_time:
                    X_now = self.ekf_location.get_state()
                    step_time = (imu_time - self.last_time) / 1000
                    self.last_time = imu_time
                    now_yaw = X_now[2, 0]

                    # x y w a module parameter
                    self.w_yaw_level, w_yaw = Caculate_ACC_Level(self.w_level, imu_w_yaw, self.w_static_var*100)
                    if abs(w_yaw) > 30:
                        a = 0
                    else:
                        self.a_level, a = Caculate_ACC_Level(self.a_level, imu_a, self.a_static_var*1000)
                    w = imu_w_yaw
                    SD_yaw = 1#self.w_static_var * step_time * 10000
                    SD_v = 1 #self.a_static_var * step_time * 100000
                    SD_x = SD_v * step_time * math.cos(np.deg2rad(now_yaw))
                    SD_y = SD_v * step_time * math.sin(np.deg2rad(now_yaw))
                    jF = jacobF(step_time, self.ekf_location.get_state())
                    Q = np.diag([SD_x, SD_y, np.deg2rad(SD_yaw), SD_v]) ** 2
                    u = np.array([[a, np.deg2rad(w)]]).T
                    self.ekf_location.predict(step_time, u, Q, jF)
            elif self.pridict_module == 1:
                if imu_time >= base_time:
                    X_now = self.ekf_location.get_state()
                    step_time = (imu_time - self.last_time) / 1000
                    self.last_time = imu_time
                    now_yaw = X_now[2, 0]

                    w = imu_w_yaw
                    v = odo_speed/1000
                    u = np.array([[v, w]]).T
                    SD_yaw = np.deg2rad(5)
                    SD_v = v * 0.01
                    SD_x = SD_v * step_time * math.cos(np.deg2rad(now_yaw - SD_yaw))
                    SD_y = SD_v * step_time * math.sin(np.deg2rad(now_yaw - SD_yaw))
                    jF = jacobF(step_time, self.ekf_location.get_state(), U=u)
                    Q = np.diag([SD_x, SD_y, SD_yaw, SD_v]) ** 2
                    self.ekf_location.predict(step_time, u, Q, jF)
            elif self.pridict_module == 2:
                a = 0
            elif self.pridict_module == 3:
                a = 0
            elif self.pridict_module == 4:
                if imu_time >= base_time:
                    X_now = self.ekf_location.get_state()
                    step_time = (imu_time - self.last_time) / 1000
                    self.last_time = imu_time
                    now_yaw = X_now[2, 0]
                    now_pitch = X_now[3, 0]

                    w_yaw = imu_w_yaw
                    w_pitch = imu_w_pitch
                    v = odo_speed / 1000
                    u = np.array([[v, w_yaw, w_pitch]]).T
                    SD_yaw = np.deg2rad(1)
                    SD_pitch = np.deg2rad(1)
                    SD_v = v * 0.01
                    SD_x = SD_v * step_time * math.cos(np.deg2rad(now_yaw)) * \
                           math.cos(np.deg2rad(now_pitch))
                    SD_y = SD_v * step_time * math.sin(np.deg2rad(now_yaw )) * \
                           math.cos(np.deg2rad(now_pitch))
                    jF = jacobF_yaw_pitch(step_time, self.ekf_location.get_state(), U=u)
                    Q = np.diag([SD_x, SD_y, SD_yaw, SD_pitch, SD_v]) ** 2
                    self.ekf_location.predict(step_time, u, Q, jF)
                    X_now = self.ekf_location.get_state()
                    X_predict = np.array([X_now[0, 0], X_now[1, 0], X_now[2, 0], X_now[3, 0], X_now[4, 0]])

            # IMU  heading  update
            if 1:
                if base_time >= imu_time:
                    if self.pridict_module == 1:
                        X_now = self.ekf_location.get_state()
                        Residual = np.array([[0, 0]]).T
                        Z_yaw = np.array([[(imu_yaw + self.yaw_offset) % 360]]).T
                        R_yaw = np.diag([np.deg2rad(0.1)]) ** 2
                        jH_yaw = jacobH_yaw()
                        zPred = observation_model_imu_yaw(X_now)
                        z_yaw_180 = Z_yaw[0, 0] - 180
                        z_pred_180 = zPred[0, 0] - 180
                        if z_yaw_180 * z_pred_180 < 0:
                            if abs(Z_yaw[0, 0] - zPred[0, 0]) > 180:
                                if Z_yaw[0, 0] > zPred[0, 0]:
                                    D_vaule = -((360 - Z_yaw[0, 0]) + zPred[0, 0] - 0)
                                    X_now[2, 0] += 360
                                    self.ekf_location.set_state(X_now)
                                else:
                                    D_vaule = ((360 - zPred[0, 0]) + Z_yaw[0, 0] - 0)
                                    X_now[2, 0] -= 360
                                    self.ekf_location.set_state(X_now)
                                Residual = np.array([[D_vaule]]).T
                        else:
                            Residual = Z_yaw - zPred
                        self.ekf_location.update(R_yaw, jH_yaw, Residual)
                    elif self.pridict_module == 4:
                        X_now = self.ekf_location.get_state()
                        Z_yaw_pitch = np.array([[(imu_yaw + self.yaw_offset) % 360, imu_pitch]]).T
                        R_yaw_pitch = np.diag([np.deg2rad(0.1), np.deg2rad(0.1)]) ** 2
                        jH_yaw_pitch = jacobH_yaw_pitch()
                        zPred = observation_model_imu_yaw_pitch(self.ekf_location.get_state())

                        Residual = np.array([[0, 0]]).T

                        D_vaule_pitch = Z_yaw_pitch[1, 0] - zPred[1, 0]
                        if abs(D_vaule_pitch)>180:
                            if Z_yaw_pitch[1, 0] >= 0:
                                D_vaule_pitch = Z_yaw_pitch[1, 0] - (zPred[1, 0] + 360)
                            else:
                                D_vaule_pitch = Z_yaw_pitch[1, 0] - (zPred[1, 0] - 360)
                        else:
                            D_vaule_pitch = Z_yaw_pitch[1, 0] - zPred[1, 0]


                        z_yaw_180 = Z_yaw_pitch[0, 0] - 180
                        z_pred_180 = zPred[0, 0] - 180
                        if z_yaw_180 * z_pred_180 < 0:
                            if abs(Z_yaw_pitch[0, 0] - zPred[0, 0]) > 180:
                                if Z_yaw_pitch[0, 0] > zPred[0, 0]:
                                    D_vaule_yaw = -((360 - Z_yaw_pitch[0, 0]) + (zPred[0, 0] - 0))
                                    X_now[2, 0] += 360
                                    self.ekf_location.set_state(X_now)
                                else:
                                    D_vaule_yaw = ((360 - zPred[0, 0]) + (Z_yaw_pitch[0, 0] - 0))
                                    X_now[2, 0] -= 360
                                    self.ekf_location.set_state(X_now)
                                Residual = np.array([[D_vaule_yaw, D_vaule_pitch]]).T
                        else:
                            Residual = Z_yaw_pitch - zPred
                        self.ekf_location.update(R_yaw_pitch, jH_yaw_pitch, Residual)

            # gps x y v update on/off
            if 1:
                if gps_state == 0:
                    if gps_time >= base_time:
                        self.gps_observation_frequency += 1
                        # gps observation frequency
                        if self.gps_observation_frequency >= 1:
                            self.gps_observation_frequency = 0
                            if (gps_mode == 4) or (gps_mode == 5):
                                X_now = self.ekf_location.get_state()
                                x, y = self.Calculate_x_y(gps_B, gps_L)
                                # caculate the gps position
                                x = x -self.gps_position_offset*math.cos(np.deg2rad(X_now[2,0]))
                                y = y -self.gps_position_offset*math.sin(np.deg2rad(X_now[2,0]))

                                Z_x_y_v = np.array([[x, y, gps_speed]]).T
                                R_x_y_v = np.diag([0.02, 0.02, 0.3]) ** 2
                                jH_x_y_v = jacobH_x_y_v()
                                zPred = observation_model_gps(self.ekf_location.get_state())
                                Residual = Z_x_y_v - zPred

                                # When the deviation between gps and ekf is too large, re-initialize the co-difference matrix and pose matrix
                                if (abs(Residual[0,0]) > GPS_AND_LOCATION_ERROR) or (abs(Residual[1,0]) > GPS_AND_LOCATION_ERROR):
                                    X_now[0, 0] = x
                                    X_now[1, 0] = y
                                    self.ekf_location.reset(X_now, np.diag([0.01, 0.01, np.deg2rad(1), np.deg2rad(1), 0.01]) ** 2)
                                else:
                                    self.ekf_location.update(R_x_y_v, jH_x_y_v, Residual)


def Location_Map():
    test = EKF.EKF(4, np.array([[0, 0, 0, 0, 0]]).T, np.diag([0.01, 0.01, np.deg2rad(1), np.deg2rad(1), 0.01]) ** 2,
                   motion_model_v_w1_w2)
    return


ekf_location_v_w_pitch_yaw = EKF_Location()

if __name__ == '__main__':
    Location_Map()