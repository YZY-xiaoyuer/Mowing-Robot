import math
import numpy
import copy
import rospy

ANGLE_ERROR = 2
D_POINT_2_POINT = 0.3 # m
HEADING_360 = 360

def Get_D_Point_2_Point(point1, point2):
    #point is [x,y...]
    point1 = point1[:2]
    point2 = point2[:2]
    D_point = point1 - point2
    D_point = math.hypot(D_point[0], D_point[1])
    return D_point

def Get_D_Point_2_Line(point,line):
    # point [x,y]
    #line[[x1,y1],[x2,y2]]
    point_x = point[0]
    point_y = point[1]
    line_s_x = line[0][0]
    line_s_y = line[0][1]
    line_e_x = line[1][0]
    line_e_y = line[1][1]
    # 若直线与y轴平行，则距离为点的x坐标与直线上任意一点的x坐标差值的绝对值
    if line_e_x - line_s_x == 0:
        return math.fabs(point_x - line_s_x)
    # 若直线与x轴平行，则距离为点的y坐标与直线上任意一点的y坐标差值的绝对值
    if line_e_y - line_s_y == 0:
        return math.fabs(point_y - line_s_y)
    # 斜率
    k = (line_e_y - line_s_y) / (line_e_x - line_s_x)
    # 截距
    b = line_s_y - k * line_s_x
    # 带入公式得到距离dis
    dis = math.fabs(k * point_x - point_y + b) / math.pow(k * k + 1, 0.5)

    return dis


def Get_delta_theta(ring_data,yaw1,yaw2):
    data1 = ring_data-yaw1
    data2 = ring_data-yaw2
    data = data1-data2
    if data > ring_data/2:
        data -= ring_data
    elif data < -(ring_data/2):
        data +=ring_data
    return data


def Get_Line_2_Goal(now_location,goal_pint):
    # point is [x,y...]
    D_2_goal = Get_D_Point_2_Point(now_location, goal_pint)
    theta = math.atan2(goal_pint[1] - now_location[1], goal_pint[0] - now_location[0])
    theta = math.degrees(theta)
    return numpy.array([D_2_goal, theta])

class Track(object):
    def __init__(self):
        # path point [x,y]
        self.path_list = []
        self.track_list = []
        # now point [x,y,θ]
        self.now_point = []
        # goal point [x,y,θ]
        self.goal_point = []
        self.destination = []
        self.line_num = 0
        self.path_line_count = 1

    def create_track(self,path):
        self.path_list = path
        self.track_list = []
        self.line_num = 0
        self.path_line_count = len(path)
        if len(path) == 0:
            return

        now_point = [0, 0, 0]
        now_point[0] = self.path_list[0][0]
        now_point[1] = self.path_list[0][1]
        next_point = [0, 0, 0]
        next_point[0] = self.path_list[1][0]
        next_point[1] = self.path_list[1][1]
        yaw = math.atan2(next_point[1] - now_point[1], next_point[0] - now_point[0])
        yaw = math.degrees(yaw)
        yaw = (yaw + 360) % 360
        now_point[2] = yaw
        self.track_list.append(now_point)
        last_point = copy.deepcopy(now_point)

        for i in range(1, len(self.path_list)-1):
            now_point = [0, 0, 0]
            now_point[0] = self.path_list[i][0]
            now_point[1] = self.path_list[i][1]
            next_point = [0, 0, 0]
            next_point[0] = self.path_list[i + 1][0]
            next_point[1] = self.path_list[i + 1][1]
            yaw = math.atan2(next_point[1] - now_point[1], next_point[0] - now_point[0])
            yaw = math.degrees(yaw)
            yaw = (yaw + 360) % 360
            now_point[2] = yaw
            if abs(last_point[2] - now_point[2]) >= ANGLE_ERROR:
                self.track_list.append(now_point)
                last_point = copy.deepcopy(now_point)

        self.destination = copy.deepcopy([path[-1][0], path[-1][1],0])
        self.track_list.append(self.destination)
        self.goal_point = self.track_list[0]
        print(self.track_list)
        return self.track_list

    # return [s, θ] None is not action
    def update(self, now_location):
        # now location [x,y,yaw]
        now_point = numpy.array([now_location[0], now_location[1]])
        goal_point = numpy.array([self.goal_point[0], self.goal_point[1]])

        if Get_D_Point_2_Point(now_point, goal_point) <= D_POINT_2_POINT:
            if goal_point == self.destination:
                return [0, 0]
            else:
                delta_yaw = Get_delta_theta(HEADING_360, now_location[2], self.goal_point[2])
                if abs(delta_yaw) <= ANGLE_ERROR:
                    self.line_num += 1
                    self.goal_point = self.track_list[self.line_num]
                    D_point = Get_D_Point_2_Point(now_point, self.goal_point)
                    return [D_point, 0]
                else:
                    return [0, delta_yaw]

        else:
            line = numpy.array([[self.track_list[self.line_num][0], self.track_list[self.line_num][1]],
                    [self.track_list[self.line_num + 1][0], self.track_list[self.line_num + 1][1]]])
            if Get_D_Point_2_Line(now_point, line) <= D_POINT_2_POINT:
                D_point = Get_D_Point_2_Point(now_point, goal_point)
                return [D_point, 0]
            else:
                D_theta = Get_Line_2_Goal(now_point, goal_point)
                delta_yaw = Get_delta_theta(HEADING_360, now_location[2], D_theta[1])
                if abs(delta_yaw) <= ANGLE_ERROR:
                    return [D_theta[0], 0]
                else:
                    return [0, delta_yaw]


