import rospy
import numpy as np
import copy
import matplotlib.pyplot as plt
try:
    import A_star
except:
    from planning_application import A_star
import cv2

class Along_side(object):
    def __init__(self, map):
        self.map = map
        self.no_clean = 255
        self.is_clean = 150
        self.barrier = 0
        self.goal_point = []
        self.x_max = map.shape[0]
        self.y_max = map.shape[1]
        self.along_origin_point = []
        self.now_point = []
        self.path_list = []

    # return 0 is left side ;1 is up side;2 is right side;3 is under side
    def find_side(self):
        dis_side_list = []
        dis_side_list.append(self.now_point[0])
        dis_side_list.append(self.y_max - 1 - self.now_point[1])
        dis_side_list.append(self.x_max - 1 - self.now_point[0])
        dis_side_list.append(self.now_point[1])
        return dis_side_list.index(min(dis_side_list))

    def find_side_point(self):
        if self.map[self.now_point[0],self.now_point[1]] != self.no_clean:
            print("start point is error")
            return 1
        recently_side = self.find_side()
        x = self.now_point[0]
        y = self.now_point[1]
        goal_point = [x, y]
        if recently_side == 0:
            while 1:
                x -= 1
                if x < 0 or x > self.x_max - 1 or \
                        y < 0 or y > self.y_max - 1:
                    break
                if self.map[x, y] == self.no_clean:
                    goal_point = [x, y]
        elif recently_side == 1:
            while 1:
                y += 1
                if x < 0 or x > self.x_max - 1 or \
                        y < 0 or y > self.y_max - 1:
                    break
                if self.map[x, y] == self.no_clean:
                    goal_point = [x, y]
        elif recently_side == 2:
            while 1:
                x += 1
                if x < 0 or x > self.x_max - 1 or \
                        y < 0 or y > self.y_max - 1:
                    break
                if self.map[x, y] == self.no_clean:
                    goal_point = [x, y]
        elif recently_side == 3:
            while 1:
                y -= 1
                if x < 0 or x > self.x_max - 1 or \
                        y < 0 or y > self.y_max - 1:
                    break
                if self.map[x, y] == self.no_clean:
                    goal_point = [x, y]

        return goal_point
    # return 0 is move ;return  1 is error ; return 2 is finish
    def along_side_move(self):

        # Clockwise from 1 to 8
        around_list = []
        xs = (-1, -1, -1, 0, 1, 1, 1, 0)
        ys = (-1, 0, 1, 1, 1, 0, -1, -1)
        for around_x, around_y in zip(xs, ys):
            x = self.now_point[0] + around_x
            y = self.now_point[1] + around_y
            if x < 0 or x > self.x_max - 1 or \
                    y < 0 or y > self.y_max - 1:
                around_list.append(1)
            elif self.map[x, y] == self.no_clean:
                around_list.append(0)
            else:
                around_list.append(1)

        if around_list.count(0) == 0:
            return 1

        if around_list[0] + around_list[1] + around_list[2] > 1:
            xs = (-1, -1, 0, 1, 1, 1, 0, -1)
            ys = (0, 1, 1, 1, 0, -1, -1, -1)
            for around_x, around_y in zip(xs, ys):
                x = self.now_point[0] + around_x
                y = self.now_point[1] + around_y
                if x < 0 or x > self.x_max - 1 or \
                        y < 0 or y > self.y_max - 1:
                    continue
                elif self.map[x, y] == self.no_clean:
                    self.now_point = [x, y]
                    break
        elif around_list[2] + around_list[3] + around_list[4] > 1:
            xs = (0, 1, 1, 1, 0, -1, -1, -1)
            ys = (1, 1, 0, -1, -1, -1, 0, 1)
            for around_x, around_y in zip(xs, ys):
                x = self.now_point[0] + around_x
                y = self.now_point[1] + around_y
                if x < 0 or x > self.x_max - 1 or \
                        y < 0 or y > self.y_max - 1:
                    continue
                elif self.map[x, y] == self.no_clean:
                    self.now_point = [x, y]
                    break
        elif around_list[4] + around_list[5] + around_list[6] > 1:
            xs = (1, 1, 0, -1, -1, -1, 0, 1)
            ys = (0, -1, -1, -1, 0, 1, 1, 1)
            for around_x, around_y in zip(xs, ys):
                x = self.now_point[0] + around_x
                y = self.now_point[1] + around_y
                if x < 0 or x > self.x_max - 1 or \
                        y < 0 or y > self.y_max - 1:
                    continue
                elif self.map[x, y] == self.no_clean:
                    self.now_point = [x, y]
                    break
        elif around_list[6] + around_list[7] + around_list[0] > 1:
            xs = (0, -1, -1, -1, 0, 1, 1, 1)
            ys = (-1, -1, 0, 1, 1, 1, 0, -1)
            for around_x, around_y in zip(xs, ys):
                x = self.now_point[0] + around_x
                y = self.now_point[1] + around_y
                if x < 0 or x > self.x_max - 1 or \
                        y < 0 or y > self.y_max - 1:
                    continue
                elif self.map[x, y] == self.no_clean:
                    self.now_point = [x, y]
                    break

        if self.now_point == self.along_origin_point:
            return 2

        self.map[self.now_point[0], self.now_point[1]] = self.is_clean
        self.path_list.append([self.now_point[0], self.now_point[1]])

        return 0

    def along_coverage(self, start_point):
        self.now_point = start_point
        temp_goal_point = self.find_side_point()
        if temp_goal_point == 1:
           return 1
        a1 = A_star.AStar(self.map,self.map.shape[0],self.map.shape[1])
        temp_search_list = a1.get_path(self.now_point, temp_goal_point)
        self.path_list += temp_search_list
        self.now_point = temp_goal_point
        self.along_origin_point = temp_goal_point
        #for i in range(len(self.path_list)):
        #    self.map[self.path_list[i][0], self.path_list[i][1]] = self.is_clean

        while 1:
            along_state = self.along_side_move()
            if along_state == 1:
                print("the rebot is Siege")
                break
            elif along_state == 2:
                print("the robot is along the edge")
                break

            #plt.clf()
            #plt.imshow(self.map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=255)
            #plt.pause(0.001)


    def get_coverage_path(self, start_point):
        if self.along_coverage(start_point) == 1:
            return 1
        else:
            return self.path_list

    def get_now_point(self):
        return self.now_point