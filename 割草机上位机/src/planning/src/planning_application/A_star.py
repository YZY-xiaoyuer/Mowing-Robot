import numpy
from pylab import *
import matplotlib.pyplot as plt

START_POINT_COLOR = 100
GOAL_POINT_COLOR = 150
SEARCH_COLOR = 130
PATH_COLOR = 200

# 定义一个含有障碍物的20×20的栅格地图
# 10表示可通行点
# 0表示障碍物
# 7表示起
# 5表示终点
map_grid = numpy.full((100, 100), int(255), dtype=numpy.int16)
#print(map_grid)
map_grid[30:50, 25:40] = 0
map_grid[30:45, 27:38] = 255
map_grid[40:80, 70:80] = 0
map_grid[55:65, 60:90] = 0
map_grid[20:50, 90:95] = 0


# 画出定义的栅格地图

class AStart_point(object):
    def __init__(self):
        self.f = 0
        self.g = 0
        self.h = 0

        self.position = numpy.array([0, 0])
        self.parent = 0 # 指向父节点

    def set_position(self, position):
        self.position = position

    def set_parent(self, parent):
        self.parent = parent

    def get_parent(self):
        return self.parent

    def get_posiation(self):
        return self.position

    def cal_h(self, goal_point):
        #使用欧式距离 (暂时)
        h = (goal_point.position[0] - self.position[0])**2 + (goal_point.position[1] - self.position[1])**2
        h = numpy.sqrt(h)
        return h

    def cal_g(self, last_point):
        extra_g = numpy.sqrt((self.position[0] - last_point.position[0])**2 + (self.position[1] - last_point.position[1])**2)
        #extra_g = (abs(self.position[0] - last_point.position[0]) + abs(self.position[1] - last_point.position[1]))
        if last_point.parent == 0:
            parent_g = 0
        else:
            parent_g = last_point.g
        return (extra_g + parent_g)
        #return 0

    def cal_f(self, last_point, goal_point):
        temp_g = self.cal_g(last_point)
        temp_h = self.cal_h(goal_point)
        return temp_g + temp_h

    def update_f(self, last_point, goal_point):
        self.g = self.cal_g(last_point)
        self.h = self.cal_h(goal_point)
        self.f = self.g + self.h



class AStar(AStart_point):
    """
    创建一个A*算法类
    """

    def __init__(self, map,shape0,shape1):
        """
        初始化
        """
        self.obstacle = 0
        self.outside_color = 200
        self.map = map
        self.x_max = shape0
        self.y_max = shape1
        self.open_list = []
        self.closed_list = []
        self.search_list = []
        self.start_point = AStart_point()
        self.goal_point = AStart_point()
        self.current_point = AStart_point()
        self.path_list = []

    def get_path(self, start_point, goal_point):
        result = self.search(start_point, goal_point)
        if result == 1:
            return self.path_list
        else:
            current_point = self.goal_point
            while 1:
                if current_point.parent == 0:
                    self.path_list.append([current_point.position[0], current_point.position[1]])
                    break
                self.path_list.append([current_point.position[0], current_point.position[1]])
                current_point = current_point.parent
            self.path_list.reverse()
            return self.path_list


    def get_min_f_point(self):
        min_point = AStart_point()
        min_f = 0
        if len(self.search_list) > 0:
            min_f = self.search_list[0].f
            min_point = self.search_list[0]
        elif len(self.open_list) > 0:
            min_f = self.open_list[0].f + self.current_point.g
            min_point = self.open_list[0]


        for i in range(len(self.search_list)):
            if self.search_list[i].f < min_f:
                min_f = self.search_list[i].f
                min_point = self.search_list[i]

        for i in range(len(self.open_list)):
            if self.open_list[i].f + self.current_point.g < min_f:
                min_f = self.open_list[i].f + self.current_point.g
                min_point = self.open_list[i]
        return min_point

    def judge_location(self, point, list):
        jud = 0
        piont_num = 0
        for i in range(len(list)):
            if point.position == list[i].position:
                jud += 1
                piont_num = i
                break
            else:
                jud = jud

        return jud, piont_num

    def creat_child_point(self, point):
        xs = (-1, 0, 1, -1, 1, -1, 0, 1)
        ys = (-1, -1, -1, 0, 0, 1, 1, 1)
        # Start iterating through the eight surrounding nodes
        #for x in range(-1, 2, 1):
        #for y in range(-1, 2, 1):
        for x, y in zip(xs, ys):
            child_point = AStart_point()
            child_point.set_position([point.position[0] + x, point.position[1] + y])
            # 去掉搜索边界
            if child_point.position[0] < 0 or child_point.position[0] > self.x_max - 1 or \
                    child_point.position[1] < 0 or child_point.position[1] > self.y_max - 1:  # 搜索点出了边界去掉
                continue
            # 搜索到障碍物去掉
            elif self.map[int(child_point.position[0]), int(child_point.position[1])] == self.obstacle:
                continue
            # 搜索到边界外去掉
            elif self.map[int(child_point.position[0]), int(child_point.position[1])] == self.outside_color:
                continue
            # if the child point is goal point return 1
            if child_point.position[0] == self.goal_point.position[0] \
                    and child_point.position[1] == self.goal_point.position[1]:
                self.goal_point.parent = point
                return 1
            jud_open, point_num = self.judge_location(child_point, self.open_list)
            # 在open表中，重新计算F G 值,如果重新计算的值小,则设A为当前节点的父节点
            if jud_open == 1:
                temp_cal_f = child_point.cal_f(point, self.goal_point)
                if temp_cal_f < self.open_list[point_num].f:
                    del self.open_list[point_num]
                    child_point.update_f(point, self.goal_point)
                    child_point.parent = point
                    self.search_list.append(child_point)
                continue
            # 在closed表中,则不搜索点
            jud_close, point_num = self.judge_location(child_point, self.closed_list)
            if jud_close == 1:
                continue
            child_point.parent = point
            child_point.update_f(point, self.goal_point)
            # The searched child nodes are added to the open list
            self.search_list.append(child_point)
            #self.map[child_point.position[0], child_point.position[1]] = SEARCH_COLOR


    def move_point_to_close_list(self, point):
        #delet the point from the open list
        if len(self.open_list) > 0:
            self.open_list.remove(point)
        #add the point to the close list
        self.closed_list.append(point)


    def search(self, start, goal):
        if start == goal:
            print("起点终点重合!")
            return 1
        elif self.map[start[0], start[1]] == self.obstacle:
            print("起点在障碍物中!")
            return 1
        elif self.map[goal[0], goal[1]] == self.obstacle:
            print("终点在障碍物中!")
            return 1
        elif self.map[start[0], start[1]] == self.outside_color:
            print("起点在边界外!")
            return 1
        elif self.map[goal[0], goal[1]] == self.outside_color:
            print("终点在边界外!")
            return 1

        self.start_point.position = start
        self.goal_point.position = goal
        self.current_point = self.start_point
        self.current_point.update_f(self.start_point, self.goal_point)
        self.open_list.append(self.current_point)
        for i in range(1500):
            if self.creat_child_point(self.current_point) == 1:
                #print(i)
                #self.draw_map()
                return 0
            #self.map[self.current_point.position[0], self.current_point.position[1]] = 100
            #self.draw_map()
            self.move_point_to_close_list(self.current_point)
            mix_f_point = self.get_min_f_point()
            self.open_list = self.open_list + self.search_list
            self.search_list.clear()
            self.current_point = mix_f_point


    def draw_map(self):
        plt.imshow(self.map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=255)
        plt.colorbar()
        xlim(-1, self.x_max)  # 设置x轴范围
        ylim(-1, self.y_max)  # 设置y轴范围
        my_x_ticks = numpy.arange(0, self.x_max, self.x_max/20)
        my_y_ticks = numpy.arange(0, self.y_max, self.y_max/20)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        plt.show()
        #plt.ion()
        #plt.pause(0.1)

    def draw_search(self):

        for i in range(len(self.closed_list)):
            self.map[self.closed_list[i].position[0], self.closed_list[i].position[1]] = SEARCH_COLOR

        for i in range(len(self.open_list)):
            self.map[self.open_list[i].position[0], self.open_list[i].position[1]] = SEARCH_COLOR
        self.map[self.start_point.position[0], self.start_point.position[1]] = START_POINT_COLOR
        self.map[self.goal_point.position[0], self.goal_point.position[1]] = GOAL_POINT_COLOR

        plt.imshow(self.map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=255)
        plt.colorbar()
        plt.grid(True)
        plt.show()

    def draw_path(self):
        current_point = self.goal_point
        while 1:
            if current_point.parent == 0:
                break
            self.map[current_point.parent.position[0], current_point.parent.position[1]] = PATH_COLOR
            current_point = current_point.parent
        self.map[self.start_point.position[0], self.start_point.position[1]] = START_POINT_COLOR
        self.map[self.goal_point.position[0], self.goal_point.position[1]] = GOAL_POINT_COLOR
        plt.imshow(self.map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=255)
        plt.colorbar()
        '''
        xlim(-1, self.x_max)  # 设置x轴范围
        ylim(-1, self.y_max)  # 设置y轴范围
        my_x_ticks = numpy.arange(0, self.x_max, self.x_max / 20)
        my_y_ticks = numpy.arange(0, self.y_max, self.x_max / 20)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        '''
        plt.show()

if __name__ == '__main__':
    a1 = AStar(map_grid)
    a1.draw_map()
    a1.search([0, 0], [68, 90])
    a1.draw_path()

