import numpy
from pylab import *
import matplotlib.pyplot as plt
import A_star
import random
import matplotlib
matplotlib.use('TkAgg')


START_POINT_COLOR = 100
GOAL_POINT_COLOR = 150
SEARCH_PATH = 180
PATH_COLOR = 190


# 定义一个含有障碍物的20×20的栅格地图
# 255表示可通行点,未清扫点
# 0表示障碍物
# 250表示已经清扫
#
map_grid = numpy.full((100, 100), int(255), dtype=numpy.int16)

#print(map_grid)
map_grid[30:50, 25:40] = 0
map_grid[30:45, 27:38] = 255
map_grid[40:80, 70:80] = 0
map_grid[55:65, 60:90] = 0
map_grid[20:50, 90:95] = 0
map_grid[65:80, 20:45] = 0



global_map = numpy.full((100, 100), int(255), dtype=numpy.int16)
#print(map_grid)
global_map[30:50, 25:40] = 0
global_map[30:45, 27:38] = 255
global_map[40:80, 70:80] = 0
global_map[55:65, 60:90] = 0
global_map[20:50, 90:95] = 0
global_map[65:80, 20:45] = 0

draw_map = numpy.full((100, 100), int(255), dtype=numpy.int16)
#print(map_grid)
draw_map[30:50, 25:40] = 0
draw_map[30:45, 27:38] = 255
draw_map[40:80, 70:80] = 0
draw_map[55:65, 60:90] = 0
draw_map[20:50, 90:95] = 0
draw_map[65:80, 20:45] = 0

class Subimage_coverage(object):

    def __init__(self, subimage_map, origin_point):
        self.subimage_is_clean = 0
        self.reachable = 0
        self.no_clean = 255
        self.is_clean = 150
        self.barrier = 0
        self.subimage_map = subimage_map
        self.x_max = subimage_map.shape[0]
        self.y_max = subimage_map.shape[1]
        self.origin_x = origin_point[0]
        self.origin_y = origin_point[1]
        self.now_point = []
        #0 is the Y-axis direction,1 is the X-axis direction,2 is the X-axis direction, 45° is the Y-axis direction,
        # 3 is the X-axis direction, negative 45° is the Y-axis direction
        self.covering_method = 0
        #Cover direction 0 is square direction 1 is negative direction
        self.coverage_direction = 0
        # direction of reciprocating motion,0 is to go back,1 is back
        self.reciprocate_direction = 0
        self.coverage_path = []
        self.search_path = []
        self.covering_width = 3


    def draw_map(self):
        #plt.figure()
        plt.imshow(self.subimage_map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=255)
        plt.colorbar()
        xlim(-1, self.x_max)  # 设置x轴范围
        ylim(-1, self.y_max)  # 设置y轴范围
        my_x_ticks = numpy.arange(0, self.x_max, 2)
        my_y_ticks = numpy.arange(0, self.y_max, 2)
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        plt.show()


    #self.reachable 为0 不可到达,self.reachable ==1 顺时针,第一条;self.reachable ==2,顺时针,第二条;
    #self.reachable ==3,顺时针,第三条,self.reachable ==4,顺时针,第四条
    def judge_reachable(self):
        if (self.subimage_map[0, :] == self.no_clean).any():
            self.reachable |= 1
        if(self.subimage_map[:, self.y_max-1] == self.no_clean).any():
            self.reachable |= 1 << 1
        if(self.subimage_map[self.x_max - 1, :] == self.no_clean).any():
            self.reachable |= 1 << 2
        if(self.subimage_map[:, 0] == self.no_clean).any():
            self.reachable |= 1 << 3

    def move_up(self):
        # 当前格设标记已经覆盖
        self.subimage_map[self.now_point[0], self.now_point[1]] = self.is_clean
        #往前移动
        x = self.now_point[0]
        if self.reciprocate_direction == 0:
            y = self.now_point[1]+1
        else:
            y = self.now_point[1]+1
        if x < 0 or x > self.x_max - 1 or \
                y < 0 or y > self.y_max - 1:
            return 1
        if self.subimage_map[x, y] == self.barrier:
            return 2
        if self.subimage_map[x, y] == self.is_clean:
            return 3
        else:
            self.coverage_path.append(self.get_now_point())
            #将当前格移动一格
            self.now_point = [x, y]
            return 0

    def move_down(self):
        # 当前格设标记已经覆盖
        self.subimage_map[self.now_point[0], self.now_point[1]] = self.is_clean
        # 往后移动
        x = self.now_point[0]
        y = self.now_point[1]-1

        if x < 0 or x > self.x_max - 1 or \
                y < 0 or y > self.y_max - 1:
            return 1
        if self.subimage_map[x, y] == self.barrier:
            return 2

        if self.subimage_map[x, y] == self.is_clean:
            return 3
        else:
            self.coverage_path.append(self.get_now_point())
            #将当前格移动一格
            self.now_point = [x, y]
            return 0

    def move_left(self):
        # 当前格设标记已经覆盖
        self.subimage_map[self.now_point[0], self.now_point[1]] = self.is_clean
        # 往左移动
        x = self.now_point[0] - 1
        y = self.now_point[1]

        if x < 0 or x > self.x_max - 1 or \
                y < 0 or y > self.y_max - 1:
            return 1
        if self.subimage_map[x, y] == self.barrier:
            return 2
        if self.subimage_map[x, y] == self.is_clean:
            return 3
        else:
            self.coverage_path.append(self.get_now_point())
            # 将当前格移动一格
            self.now_point = [x, y]
            return 0

    def move_right(self):
        # 当前格设标记已经覆盖
        self.subimage_map[self.now_point[0], self.now_point[1]] = self.is_clean

        # 往右移动
        x = self.now_point[0] + 1
        y = self.now_point[1]

        if x < 0 or x > self.x_max - 1 or \
                y < 0 or y > self.y_max - 1:
            return 1
        if self.subimage_map[x, y] == self.barrier:
            return 2
        if self.subimage_map[x, y] == self.is_clean:
            return 3
        else:
            self.coverage_path.append(self.get_now_point())
            # 将当前格移动一格
            self.now_point = [x, y]
            return 0

    def move_upper_left(self):
        # 当前格设标记已经覆盖
        self.subimage_map[self.now_point[0], self.now_point[1]] = self.is_clean
        # 往左上移动
        x = self.now_point[0] - 1
        y = self.now_point[1] + 1

        if x < 0 or x > self.x_max - 1 or \
                y < 0 or y > self.y_max - 1:
            return 1
        if self.subimage_map[x, y] == self.barrier:
            return 2
        if self.subimage_map[x, y] == self.is_clean:
            return 3
        else:
            self.coverage_path.append(self.get_now_point())
            # 将当前格移动一格
            self.now_point = [x, y]
            return 0

    def move_upper_right(self):
        # 当前格设标记已经覆盖
        self.subimage_map[self.now_point[0], self.now_point[1]] = self.is_clean
        # 往右上移动
        x = self.now_point[0] + 1
        y = self.now_point[1] + 1

        if x < 0 or x > self.x_max - 1 or \
                y < 0 or y > self.y_max - 1:
            return 1
        if self.subimage_map[x, y] == self.barrier:
            return 2
        if self.subimage_map[x, y] == self.is_clean:
            return 3
        else:
            self.coverage_path.append(self.get_now_point())
            #将当前格移动一格
            self.now_point = [x, y]
            return 0


    def move_under_left(self):
        # 当前格设标记已经覆盖
        self.subimage_map[self.now_point[0], self.now_point[1]] = self.is_clean
        # 往左下移动
        x = self.now_point[0] - 1
        y = self.now_point[1] - 1

        if x < 0 or x > self.x_max - 1 or \
                y < 0 or y > self.y_max - 1:
            return 1
        if self.subimage_map[x, y] == self.barrier:
            return 2
        if self.subimage_map[x, y] == self.is_clean:
            return 3
        else:
            self.coverage_path.append(self.get_now_point())
            # 将当前格移动一格
            self.now_point = [x, y]
            return 0


    def move_under_right(self):
        # 当前格设标记已经覆盖
        self.subimage_map[self.now_point[0], self.now_point[1]] = self.is_clean
        # 往右下移动
        x = self.now_point[0] + 1
        y = self.now_point[1] - 1

        if x < 0 or x > self.x_max - 1 or \
                y < 0 or y > self.y_max - 1:
            return 1
        if self.subimage_map[x, y] == self.barrier:
            return 2
        if self.subimage_map[x, y] == self.is_clean:
            return 3
        else:
            self.coverage_path.append(self.get_now_point())
            #将当前格移动一格
            self.now_point = [x, y]
            return 0

    def coverage_section_lengthways(self):
        if self.reciprocate_direction == 0:
            while 1:
                if self.move_up() != 0:
                    break
        else:
            while 1:
                if self.move_down() != 0:
                    break

        for i in range(1):
            if self.coverage_direction == 0:
                if self.move_right() != 0:
                    if self.reciprocate_direction == 0:
                        if self.move_under_right() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
                    else:
                        if self.move_upper_right() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
            else:
                if self.move_left() != 0:
                    if self.reciprocate_direction == 0:
                        if self.move_under_left() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
                    else:
                        if self.move_upper_left() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
        self.reciprocate_direction ^= 1
        #self.draw_map()
        return 0

    def coverage_section_crosswise(self):
        if self.reciprocate_direction == 0:
            while 1:
                if self.move_right() != 0:
                    break
        else:
            while 1:
                if self.move_left() != 0:
                    break

        for i in range(1):
            if self.coverage_direction == 0:
                if self.move_up() != 0:
                    if self.reciprocate_direction == 0:
                        if self.move_upper_left() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
                    else:
                        if self.move_upper_right() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
            else:
                if self.move_down() != 0:
                    if self.reciprocate_direction == 0:
                        if self.move_under_left() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
                    else:
                        if self.move_under_right() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
        self.reciprocate_direction ^= 1
        return 0

    def coverage_section_inclined_left(self):
        if self.reciprocate_direction == 0:
            while 1:
                if self.move_under_right() != 0:
                    break
        else:
            while 1:
                if self.move_upper_left() != 0:
                    break

        for i in range(1):
            if self.coverage_direction == 0:
                if self.reciprocate_direction == 0:
                    if self.move_right() != 0:
                        if self.move_up() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1

                else:
                    if self.move_up() != 0:
                        if self.move_right() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1

            else:
                if self.reciprocate_direction == 0:
                    if self.move_down() != 0:
                        if self.move_left() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            #self.draw_map()
                            return 1
                else:
                    if self.move_left() != 0:
                        if self.move_down() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            #self.draw_map()
                            return 1

        self.reciprocate_direction ^= 1
        return 0

    def coverage_section_inclined_right(self):
        if self.reciprocate_direction == 0:
            while 1:
                if self.move_upper_right() != 0:
                    break
        else:
            while 1:
                if self.move_under_left() != 0:
                    break

        for i in range(1):
            if self.coverage_direction == 0:
                if self.reciprocate_direction == 0:
                    if self.move_up() != 0:
                        if self.move_left() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
                else:
                    if self.move_left() != 0:
                        if self.move_up() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
            else:
                if self.reciprocate_direction == 0:
                    if self.move_right() != 0:
                        if self.move_down() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
                else:
                    if self.move_down() != 0:
                        if self.move_right() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                            return 1
        self.reciprocate_direction ^= 1
        return 0

    def judge_full_coverage(self):
        return (self.subimage_map == self.no_clean).any()


    def coverage(self, now_point):
        self.now_point = now_point
        self.now_point[0] -= self.origin_x
        self.now_point[1] -= self.origin_y

        # Set the coverage direction and roundtrip direction
        if self.covering_method == 0:
            if self.now_point[0] > self.x_max / 2:
                self.coverage_direction = 1
            else:
                self.coverage_direction = 0
            if self.now_point[1] > self.y_max/2:
                self.reciprocate_direction = 1
            else:
                self.reciprocate_direction = 0

        elif self.covering_method == 1:
            if self.now_point[0] > self.x_max / 2:
                self.reciprocate_direction = 1
            else:
                self.reciprocate_direction = 0
            if self.now_point[1] > self.y_max/2:
                self.coverage_direction = 1
            else:
                self.coverage_direction = 0
        elif self.covering_method == 2:
            min_axis = min(self.x_max, self.y_max)
            if self.now_point[0] + self.now_point[1] > min_axis:
                self.coverage_direction = 1
            else:
                self.coverage_direction = 0
            if self.now_point[0] > self.now_point[1]:
                self.reciprocate_direction = 0
            else:
                self.reciprocate_direction = 1
        elif self.covering_method == 3:
            min_axis = min(self.x_max, self.y_max)
            if self.now_point[0] + self.now_point[1] > min_axis:
                self.reciprocate_direction = 1
            else:
                self.reciprocate_direction = 0
            if self.now_point[0] > self.now_point[1]:
                self.coverage_direction = 0
            else:
                self.coverage_direction = 1

        while 1:
            besiege_flag = 0
            if self.covering_method == 0:
                besiege_flag = self.coverage_section_lengthways()
            elif self.covering_method == 1:
                besiege_flag = self.coverage_section_crosswise()
            elif self.covering_method == 2:
                besiege_flag = self.coverage_section_inclined_left()
            elif self.covering_method == 3:
                besiege_flag = self.coverage_section_inclined_right()
            if besiege_flag == 1:
                if self.judge_full_coverage() == 1:
                    if self.covering_method == 3:
                        if self.coverage_direction == 0:
                            # 正方向覆盖,则先返再往,
                            self.reciprocate_direction = 1
                            # 正向,找 x最大,y最小的点
                            no_clean_list = np.where(self.subimage_map == self.no_clean)
                            goal_point_num = -1
                            for i in range(len(no_clean_list[0])):
                                if no_clean_list[0][goal_point_num] != no_clean_list[0][goal_point_num-i]:
                                    goal_point_num = goal_point_num - i + 1
                                    break
                            goal_point = [no_clean_list[0][goal_point_num], no_clean_list[1][goal_point_num]]
                        else:
                            # 负方向, 则先往在返
                            self.reciprocate_direction = 0
                            # 反向,找 x最小,y最大的点
                            no_clean_list = np.where(self.subimage_map == self.no_clean)
                            goal_point_num = 0
                            for i in range(len(no_clean_list[0])):
                                if no_clean_list[0][goal_point_num] != no_clean_list[0][goal_point_num+i]:
                                    goal_point_num = goal_point_num + i - 1
                                    break
                            goal_point = [no_clean_list[0][goal_point_num], no_clean_list[1][goal_point_num]]
                    else:
                        if self.coverage_direction == 0:
                            # 正方向覆盖,则先往再返,
                            self.reciprocate_direction = 0
                            # 正向,找 x,y 最小的点
                            no_clean_list = np.where(self.subimage_map == self.no_clean)
                            goal_point = [no_clean_list[0][0], no_clean_list[1][0]]
                        else:
                            #返方向, 则先返在往
                            self.reciprocate_direction = 1
                            # 负方向,找 x,y 最大的点
                            no_clean_list = np.where(self.subimage_map == self.no_clean)
                            goal_point = [no_clean_list[0][-1], no_clean_list[1][-1]]

                    temp_a_star = A_star.AStar(global_map,global_map.shape[0],global_map.shape[1])
                    temp_path = temp_a_star.get_path([self.now_point[0]+self.origin_x, self.now_point[1]+self.origin_y], [goal_point[0]+self.origin_x, goal_point[1]+ self.origin_y])
                    self.search_path += temp_path
                    self.coverage_path += temp_path
                    self.now_point = goal_point
                    #for i in range(len(temp_path)):
                    #    self.subimage_map[temp_path[i][0], temp_path[i][1]] = PATH_COLOR
                    #self.draw_map()
                else:
                    # 子图全覆盖
                    #for i in range(len(self.search_path)):
                    #    self.subimage_map[self.search_path[i][0], self.search_path[i][1]] = PATH_COLOR
                    #self.draw_map()
                    self.subimage_is_clean = 1
                    return 1

    def get_coverage_path(self, now_point):
        self.coverage(now_point)
        return self.coverage_path

    def get_now_point(self):
        return [self.now_point[0]+self.origin_x, self.now_point[1]+self.origin_y]


class Along_side(object):
    def __init__(self, map):
        self.map = map
        self.no_clean = 255
        self.is_clean = 150
        self.barrier = 0
        self.x_max = map.shape[0]
        self.y_max = map.shape[1]
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

    def along_side_move(self):
        self.map[self.now_point[0], self.now_point[1]] = self.is_clean
        self.path_list.append([self.now_point[0], self.now_point[1]])
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

        return 0

    def along_coverage(self, start_point):

        self.now_point = start_point
        temp_goal_point = self.find_side_point()
        a1 = A_star.AStar(self.map,self.map.shape[0],self.map.shape[1])
        temp_search_list = a1.get_path(self.now_point, temp_goal_point)
        self.path_list += temp_search_list
        self.now_point = temp_goal_point
        for i in range(len(self.path_list)):
            self.map[self.path_list[i][0], self.path_list[i][1]] = self.is_clean
        while 1:
            if self.along_side_move() == 1:
                if (self.map == self.no_clean).any():
                    no_clean_list = np.where(self.map == self.no_clean)
                    if len(no_clean_list) > 0:
                        self.map[no_clean_list[0][0], no_clean_list[1][0]] = self.is_clean
                        self.path_list.append([no_clean_list[0][0], no_clean_list[1][0]])
                else:
                    break




    def get_coverage_path(self, start_point):
        self.along_coverage(start_point)
        temp_list = []
        for one in self.path_list:
            if one not in temp_list:
                temp_list.append(one)
        return temp_list

    def get_now_point(self):
        return self.now_point


class Planning(object):
    def __init__(self, map):
        self.map = map
        self.no_clean = 255
        self.is_clean = 150
        self.barrier = 0
        self.subimage_plan_map = []
        self.now_point = []
        self.x_max = map.shape[0]
        self.y_max = map.shape[1]
        # Horizontal split
        self.hsplit = 5
        # Longitudinal partitioning
        self.vsplit = 5
        self.subimage_list = []
        self.subimage_path = []
        self.coverage_path = []

    def draw_map(self):
        plt.imshow(self.map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=255)
        plt.colorbar()
        xlim(-1, self.x_max)  # 设置x轴范围
        ylim(-1, self.y_max)  # 设置y轴范围
        my_x_ticks = numpy.arange(0, self.x_max, int(self.x_max/20))
        my_y_ticks = numpy.arange(0, self.y_max, int(self.y_max/20))
        plt.xticks(my_x_ticks)
        plt.yticks(my_y_ticks)
        plt.grid(True)
        plt.show()

    def draw_ion(self):
        for i in range(len(self.coverage_path)):
            plt.clf()
            now_point_data = draw_map[self.coverage_path[i][0], self.coverage_path[i][1]]
            draw_map[self.coverage_path[i][0], self.coverage_path[i][1]] = 100
            plt.imshow(draw_map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=255)
            plt.colorbar()
            xlim(-1, self.x_max)  # 设置x轴范围
            ylim(-1, self.y_max)  # 设置y轴范围
            my_x_ticks = numpy.arange(0, self.x_max, int(self.x_max / 20))
            my_y_ticks = numpy.arange(0, self.y_max, int(self.y_max / 20))
            plt.xticks(my_x_ticks)
            plt.yticks(my_y_ticks)
            plt.grid(True)
            now_point_data -= 50
            draw_map[self.coverage_path[i][0], self.coverage_path[i][1]] = now_point_data
            plt.pause(0.00001)


    def segmentation_map(self):
        # Horizontal split
        vsplit_list = numpy.array_split(self.map, self.vsplit, axis=1)
        for i in range(len(vsplit_list)):
            # Longitudinal partitioning
            hsplit_list = numpy.array_split(vsplit_list[i], self.hsplit, axis=0)
            for j in range(len(hsplit_list)):
                sub_img = Subimage_coverage(hsplit_list[j], [int(self.x_max/self.hsplit)*j, int(self.x_max/self.vsplit)*i])
                sub_img.judge_reachable()
                sub_img.covering_method = random.randint(0, 3)
                self.subimage_list.append(sub_img)

    def find_nearest_angle_point(self, next_subimage, now_point):
        temp_dis_angle = []
        if global_map[next_subimage.origin_x, next_subimage.origin_y] != self.barrier:
            temp_dis_angle.append(numpy.sqrt((next_subimage.origin_x - now_point[0])**2 + \
                                             (next_subimage.origin_y - now_point[1])**2))
        else:
            temp_dis_angle.append(9999)
        if global_map[next_subimage.origin_x, next_subimage.origin_y + next_subimage.y_max - 1] != self.barrier:
            temp_dis_angle.append(numpy.sqrt((next_subimage.origin_x - now_point[0]) ** 2 + \
                                             (next_subimage.origin_y + next_subimage.y_max - 1 - now_point[1]) ** 2))
        else:
            temp_dis_angle.append(9999)
        if global_map[next_subimage.origin_x + next_subimage.x_max - 1, \
                      next_subimage.origin_y + next_subimage.y_max - 1] != self.barrier:
            temp_dis_angle.append(numpy.sqrt((next_subimage.origin_x + next_subimage.x_max - 1 - now_point[0]) ** 2 + \
                                             (next_subimage.origin_y + next_subimage.y_max - 1 - now_point[1]) ** 2))
        else:
            temp_dis_angle.append(9999)
        if global_map[next_subimage.origin_x + next_subimage.x_max - 1, next_subimage.origin_y] != self.barrier:
            temp_dis_angle.append(numpy.sqrt((next_subimage.origin_x + next_subimage.x_max - 1 - now_point[0]) ** 2 + \
                                             (next_subimage.origin_y - now_point[1]) ** 2))
        else:
            temp_dis_angle.append(9999)
        min_point_num = temp_dis_angle.index(min(temp_dis_angle))

        if min_point_num == 0:
            x_origin = next_subimage.origin_x
            y_origin = next_subimage.origin_y
            len_mix = min(next_subimage.x_max, next_subimage.y_max)
            if global_map[x_origin, y_origin] != self.barrier:
                return [x_origin, y_origin]
            for i in range(1, len_mix):
                #search the row
                y = y_origin + i
                for j in range(i+1):
                    x = x_origin + j
                    if global_map[x, y] != self.barrier:
                        return [x, y]
                # search the line
                x = x_origin + i
                for j in range(i+1):
                    y = y_origin + j
                    if global_map[x, y] != self.barrier:
                        return [x, y]


        if min_point_num == 1:
            x_origin = next_subimage.origin_x
            y_origin = next_subimage.origin_y + next_subimage.y_max - 1
            len_mix = min(next_subimage.x_max, next_subimage.y_max)
            if global_map[x_origin, y_origin] != self.barrier:
                return [x_origin, y_origin]
            for i in range(1, len_mix):
                # search the row
                y = y_origin - i
                for j in range(i + 1):
                    x = x_origin + j
                    if global_map[x, y] != self.barrier:
                        return [x, y]
                # search the line
                x = x_origin + i
                for j in range(i+1):
                    y = y_origin - j
                    if global_map[x, y] != self.barrier:
                        return [x, y]

        if min_point_num == 2:
            x_origin = next_subimage.origin_x + next_subimage.x_max - 1
            y_origin = next_subimage.origin_y + next_subimage.y_max - 1
            len_mix = min(next_subimage.x_max, next_subimage.y_max)
            if global_map[x_origin, y_origin] != self.barrier:
                return [x_origin, y_origin]
            for i in range(1, len_mix):
                # search the row
                y = y_origin - i
                for j in range(i + 1):
                    x = x_origin - j
                    if global_map[x, y] != self.barrier:
                        return [x, y]
                # search the line
                x = x_origin - i
                for j in range(i+1):
                    y = y_origin - j
                    if global_map[x, y] != self.barrier:
                        return [x, y]

        if min_point_num == 3:
            x_origin = next_subimage.origin_x + next_subimage.x_max - 1
            y_origin = next_subimage.origin_y
            len_mix = min(next_subimage.x_max, next_subimage.y_max)
            if global_map[x_origin, y_origin] != self.barrier:
                return [x_origin, y_origin]
            for i in range(1, len_mix):
                # search the row
                y = y_origin + i
                for j in range(i + 1):
                    x = x_origin - j
                    if global_map[x, y] != self.barrier:
                        return [x, y]
                # search the line
                x = x_origin - i
                for j in range(i+1):
                    y = y_origin + j
                    if global_map[x, y] != self.barrier:
                        return [x, y]


    def coverage_subimage(self, start_point):
        self.now_point = start_point
        self.segmentation_map()
        self.subimage_plan_map = numpy.full((self.hsplit, self.vsplit), int(255), dtype=numpy.int16)
        for i in range(self.hsplit):
            for j in range(self.vsplit):
                if self.subimage_list[i*self.vsplit+j].reachable == 0:
                    self.subimage_plan_map[i, j] = 0
        temp_along_start_x = int(self.now_point[0] / (self.x_max/self.vsplit))
        temp_along_start_y = int(self.now_point[1] / (self.y_max / self.hsplit))
        temp_along_start_point = [temp_along_start_x, temp_along_start_y]
        temp_along_side = Along_side(self.subimage_plan_map)
        self.subimage_path = temp_along_side.get_coverage_path(temp_along_start_point)
        for i in range(len(self.subimage_path)):
            x = self.subimage_path[i][0]
            y = self.subimage_path[i][1]
            list_num = x+y*self.vsplit
            subimage = self.subimage_list[list_num]
            if subimage.subimage_is_clean == 1:
                continue
            subimage_coverage_path = subimage.get_coverage_path(self.now_point)
            self.coverage_path += subimage_coverage_path
            self.now_point = self.subimage_list[list_num].get_now_point()
            #for num in range(len(subimage_coverage_path)):
            #    self.map[subimage_coverage_path[num][0], subimage_coverage_path[num][1]] -= 70
            #self.draw_map()
            #coverage the all map
            if i == len(self.subimage_path) - 1:
                #self.draw_map()
                return 0
            x_next = self.subimage_path[i+1][0]
            y_next = self.subimage_path[i+1][1]
            next_subimage_start_point = self.find_nearest_angle_point(self.subimage_list[x_next+y_next*self.vsplit], self.now_point)
            a_star = A_star.AStar(global_map,global_map.shape[0],global_map.shape[1])
            self.coverage_path += a_star.get_path(self.now_point, next_subimage_start_point)
            self.now_point = next_subimage_start_point

if __name__ == '__main__':
    p1 = Planning(map_grid)
    #p1.draw_map()
    p1.coverage_subimage([35, 30])
    p1.draw_ion()
