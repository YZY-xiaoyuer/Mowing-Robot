#!/usr/bin/env python
import rospy
from msg_srv_act.msg import M_location_pose
import matplotlib.pyplot as plt
import copy
import numpy as np
import math


class Display_Location(object):
    def __init__(self):
        self.display_list_x = []
        self.display_list_y = []
        self.display_list_yaw = []
        self.display_list_pitch = []
        self.display_list_speed = []
        self.draw_frequency = 1
        self.draw_covariance_on = 0
        self.draw_in_one_picture = True
        self.covariance = []

    def plot_covariance_ellipse(self, position_x, position_y, p_covariance_xy):
        eigval, eigvec = np.linalg.eig(p_covariance_xy)

        if eigval[0] >= eigval[1]:
            bigind = 0
            smallind = 1
        else:
            bigind = 1
            smallind = 0

        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        a = math.sqrt(eigval[bigind])
        b = math.sqrt(eigval[smallind])
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        angle = math.atan2(eigvec[bigind, 1], eigvec[bigind, 0])
        R = np.array([[math.cos(angle), math.sin(angle)],
                      [-math.sin(angle), math.cos(angle)]])
        fx = R @ (np.array([x, y]))
        px = np.array(fx[0, :] + position_x).flatten()
        py = np.array(fx[1, :] + position_y).flatten()
        plt.plot(px, py, "--r")

    def draw(self):
        self.draw_frequency += 1
        if self.draw_frequency % 5 == 0:
            if self.draw_in_one_picture:
                figure1 = plt.subplot(131)
            else:
                plt.figure(1)
            plt.cla()
            plt.plot(self.display_list_x, self.display_list_y, "-g", linewidth=1)
            plt.title("location ")
            x_now = self.display_list_x[-1]
            y_now = self.display_list_y[-1]
            plt.plot(x_now, y_now, ".b", linewidth=3)
            x_start = self.display_list_x[0]
            y_start = self.display_list_y[0]
            plt.plot(x_start, y_start, ".r", linewidth=3)

            if self.draw_covariance_on:
                P_now = self.covariance
                p_xy = P_now[0:2, 0:2]
                self.plot_covariance_ellipse(x_now, y_now, p_xy)
            if self.draw_in_one_picture:
                figure1 = plt.subplot(132)
            else:
                plt.figure(2)
            plt.cla()
            yaw_list = self.display_list_yaw
            pitch_list = self.display_list_pitch
            plt.title("heading,pitch")
            if yaw_list != []:
                plt.plot(np.linspace(0, 0.01 * len(yaw_list), len(yaw_list)), yaw_list, "-b", linewidth=1)
            if pitch_list != []:
                plt.plot(np.linspace(0, 0.01 * len(pitch_list), len(pitch_list)), pitch_list, "-r", linewidth=1)

            if self.draw_in_one_picture:
                figure1 = plt.subplot(133)
            else:
                plt.figure(3)
            plt.cla()
            speed_list = self.display_list_speed
            plt.title("speed")
            plt.plot(np.linspace(0, 0.01 * len(speed_list), len(speed_list)), speed_list, "-r", linewidth=1)
            plt.pause(0.001)



display_location = Display_Location()

def Display(pose_data):
    display_location.covariance = copy.deepcopy(pose_data.covariance_pose)
    display_location.display_list_x.append(pose_data.location_pose[0])
    display_location.display_list_y.append(pose_data.location_pose[1])
    display_location.display_list_yaw.append(pose_data.location_pose[2])
    display_location.display_list_pitch.append(pose_data.location_pose[3])
    display_location.display_list_speed.append(pose_data.location_pose[4])
    display_location.draw()
    return

def subscription():

    rospy.init_node('display_location', anonymous=True)

    rospy.Subscriber("EKF_location_pose", M_location_pose, Display)

    rospy.loginfo("display location node is Ready")


    rospy.spin()

if __name__ == '__main__':
    try:
        subscription()
    except rospy.ROSInterruptException:
        print(__file__)
        pass