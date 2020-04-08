import numpy
from pylab import *
import copy
import matplotlib.pyplot as plt
try:
    import A_star
except:
    from planning_application import A_star
import random
import matplotlib
matplotlib.use('TkAgg')

POLYLINE_LENGTH = 1

class Subimage_coverage(object):

    def __init__(self,global_map, subimage_map, origin_point):
        self.subimage_is_clean = 0
        self.reachable = 0
        self.Walkable = 255
        self.is_clean = 150
        self.outside = 200
        self.barrier = 0
        self.subimage_map = copy.deepcopy(subimage_map)
        self.draw_map = copy.deepcopy(subimage_map)
        self.global_map = copy.deepcopy(global_map)
        self.x_max = subimage_map.shape[0]
        self.y_max = subimage_map.shape[1]
        self.origin_x = origin_point[0]
        self.origin_y = origin_point[1]
        self.now_point = []
        #0 is the Y-axis direction,1 is the X-axis direction,2 is the X-axis direction, 45° is the Y-axis direction,
        # 3 is the X-axis direction, negative 45° is the Y-axis direction
        #self.covering_method = 1
        self.covering_method = random.randint(0, 3)
        #Cover direction 0 is square direction 1 is negative direction
        self.coverage_direction = 0
        # direction of reciprocating motion,0 is to go back,1 is back
        self.reciprocate_direction = 0
        self.coverage_path = []
        self.search_path = []

    def draw_map(self):
        #plt.figure()
        show_map = copy.de
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

    def draw_ion(self):
        for i in range(len(self.coverage_path)):
            plt.clf()
            now_point_data = self.draw_map[self.coverage_path[i][0], self.coverage_path[i][1]]
            self.draw_map[self.coverage_path[i][0], self.coverage_path[i][1]] = 150
            if i%1 == 0:
                plt.imshow(self.draw_map, cmap=plt.cm.hot, interpolation='nearest', vmin=0, vmax=255)
                plt.colorbar()
                plt.grid(True)
                plt.pause(0.00001)
            now_point_data -= 80
            self.draw_map[self.coverage_path[i][0], self.coverage_path[i][1]] = now_point_data



    #self.reachable 为0 不可到达,self.reachable ==1 顺时针,第一条;self.reachable ==2,顺时针,第二条;
    #self.reachable ==3,顺时针,第三条,self.reachable ==4,顺时针,第四条
    def judge_reachable(self):
        if (self.subimage_map[0, :] == self.Walkable).any():
            self.reachable |= 1
        if(self.subimage_map[:, self.y_max-1] == self.Walkable).any():
            self.reachable |= 1 << 1
        if(self.subimage_map[self.x_max - 1, :] == self.Walkable).any():
            self.reachable |= 1 << 2
        if(self.subimage_map[:, 0] == self.Walkable).any():
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
        if self.subimage_map[x, y] == self.outside:
            return 4
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
        if self.subimage_map[x, y] == self.outside:
            return 4
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
        if self.subimage_map[x, y] == self.outside:
            return 4
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
        if self.subimage_map[x, y] == self.outside:
            return 4
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
        if self.subimage_map[x, y] == self.outside:
            return 4
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
        if self.subimage_map[x, y] == self.outside:
            return 4
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
        if self.subimage_map[x, y] == self.outside:
            return 4
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
        if self.subimage_map[x, y] == self.outside:
            return 4
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

        for i in range(POLYLINE_LENGTH):
            if self.coverage_direction == 0:
                if self.reciprocate_direction == 0:
                    if self.move_upper_right() != 0:
                        if self.move_right() != 0:
                            if self.move_under_right() != 0:
                                # 被障碍物,边界,以覆盖区域围困
                                return 1
                else:
                    if self.move_under_right() != 0:
                        if self.move_right() != 0:
                            if self.move_upper_right() != 0:
                                # 被障碍物,边界,以覆盖区域围困
                                return 1
            else:
                if self.reciprocate_direction == 0:
                    if self.move_upper_left() != 0:
                        if self.move_left() != 0:
                            if self.move_under_left() != 0:
                            # 被障碍物,边界,以覆盖区域围困
                                return 1
                else:
                    if self.move_under_left() != 0:
                        if self.move_left() != 0:
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

        for i in range(POLYLINE_LENGTH):
            if self.coverage_direction == 0:
                if self.reciprocate_direction == 0:
                    if self.move_upper_right() != 0:
                        if self.move_up() != 0:
                            if self.move_upper_left() != 0:
                                # 被障碍物,边界,以覆盖区域围困
                                return 1
                else:
                    if self.move_upper_left() != 0:
                        if self.move_up() != 0:
                            if self.move_upper_right() != 0:
                                # 被障碍物,边界,以覆盖区域围困
                                return 1
            else:
                if self.reciprocate_direction == 0:
                    if self.move_under_right() != 0:
                        if self.move_down() != 0:
                            if self.move_under_left() != 0:
                                # 被障碍物,边界,以覆盖区域围困
                                return 1
                else:
                    if self.move_under_left() != 0:
                        if self.move_down() != 0:
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

        for i in range(POLYLINE_LENGTH):
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

        for i in range(POLYLINE_LENGTH):
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
        return (self.subimage_map == self.Walkable).any()

    # return 1 is 
    def coverage(self, now_point):
        self.now_point = now_point
        if self.subimage_map[self.now_point[0],self.now_point[1]] == self.outside:
            print("起点在边界外")
            return 1
        if self.subimage_map[self.now_point[0],self.now_point[1]] == self.barrier:
            print("起点在障碍物中")
            return 2
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
                            no_clean_list = np.where(self.subimage_map == self.Walkable)
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
                            no_clean_list = np.where(self.subimage_map == self.Walkable)
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
                            no_clean_list = np.where(self.subimage_map == self.Walkable)
                            goal_point = [no_clean_list[0][0], no_clean_list[1][0]]
                        else:
                            #返方向, 则先返在往
                            self.reciprocate_direction = 1
                            # 负方向,找 x,y 最大的点
                            no_clean_list = np.where(self.subimage_map == self.Walkable)
                            goal_point = [no_clean_list[0][-1], no_clean_list[1][-1]]

                    temp_a_star = A_star.AStar(self.global_map,self.global_map.shape[0],self.global_map.shape[1])
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
