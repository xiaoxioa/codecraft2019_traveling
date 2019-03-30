import numpy as np
import tool


class Car:
    def __init__(self, car_id, start_point, end_point, max_speed, start_time):
        self.car_id = car_id
        self.start_cross = start_point
        self.end_cross = end_point
        self.max_speed = max_speed
        self.start_time = start_time
        # update when move to new road
        self.actual_speed = 0
        self.route = []
        self.cross_by = []
        self.cross_index = -1
        self.state = 'G'  # 'G' for in garage/ 'R' for on road/ 'E' for reach end
        self.location = [0, 0, 0, 0]  # [road_id,road_map_id,channel_index,location_index]
        # update when plan new path
        self.next_direction = 'D'
        self.next_road = '0'
        self.next_road_map_id = 1
        self.next_cross = start_point  # 车子即将经过的路口
        self.next_next_cross = '0'

    def updateState(self, speed, state='R'):
        self.actual_speed = speed
        self.state = state

    def updateNextPath(self, direction, road_id, road_map, cross):
        self.next_direction = direction
        self.next_road = road_id
        self.next_road_map_id = road_map
        self.next_next_cross = cross



class Road:
    def __init__(self, road_id, road_len, road_speed, num_channel, start_cross, end_cross, reverse):
        self.road_id = road_id
        self.road_len = road_len
        self.road_speed = road_speed
        self.num_channel = num_channel
        self.start_cross = start_cross
        self.end_cross = end_cross
        self.reverse = reverse
        self.min_speed = 0
        self.forwardNum = 0
        self.backwardNum = 0
        self.carCapcity = self.num_channel * self.road_len
        # self.road_map_1= np.zeros((int(num_channel),int(road_len)))
        # self.road_map_2 = np.zeros((int(num_channel),int(road_len)))
        self.road_map = {0: np.zeros((int(num_channel), int(road_len)), dtype=int), 1: np.zeros((int(num_channel), int(road_len)), dtype=int)}

    def updateNum(self):
        map = self.road_map[0]
        self.forwardNum = map[map > 0].size
        map = self.road_map[1]
        self.backwardNum = map[map > 0].size

class Cross:
    def __init__(self, cross_id, road_u, road_r, road_d, road_l):
        self.cross_id = cross_id
        self.cross_road = [road_u, road_r, road_d, road_l]
        self.road_u = [road_u, -1]  # [road_id,road_map_id]
        self.road_r = [road_r, -1]
        self.road_d = [road_d, -1]
        self.road_l = [road_l, -1]
        self.map_index = [0, 0]

    def getSortedRoadId(self):
        road_dict = {}
        road_dict[self.road_u[0]] = self.road_u[1]
        road_dict[self.road_r[0]] = self.road_r[1]
        road_dict[self.road_d[0]] = self.road_d[1]
        road_dict[self.road_l[0]] = self.road_l[1]
        # print(road_dict)
        return road_dict

    def setRoadMap(self, road_id, road_map):
        if road_id == self.road_u[0]:
            self.road_u[1] = road_map
        if road_id == self.road_r[0]:
            self.road_r[1] = road_map
        if road_id == self.road_d[0]:
            self.road_d[1] = road_map
        if road_id == self.road_l[0]:
            self.road_l[1] = road_map
        return


def createCrossMap(all_cross,all_road):

    open_list = []
    for cross_key in all_cross.keys():
        cross = all_cross[cross_key]

        if cross.road_l[0] == '-1' and cross.road_u[0] == '-1':
            left_up_cross = cross_key
            open_list.append(cross_key)

    close_list = []
    map_q = np.zeros((int(np.sqrt(len(all_cross))), int(np.sqrt(len(all_cross)))))
    index_x = 0
    index_y = 0
    map_q[index_x][index_y] = left_up_cross

    while len(open_list) > 0:
        cross_key = open_list.pop(0)
        cross = all_cross[cross_key]
        index_x = cross.map_index[0]
        index_y = cross.map_index[1]
        next_cross = '0'
        if cross.road_u[0] != '-1':
            road_id = cross.road_u[0]
            road = all_road[road_id]
            if road.start_cross == cross_key:
                next_cross = road.end_cross
            else:
                next_cross = road.start_cross
            if next_cross not in close_list:
                map_q[index_y - 1][index_x] = next_cross
                open_list.append(next_cross)
                close_list.append(next_cross)
                all_cross[next_cross].map_index = [index_x, index_y - 1]

        if cross.road_r[0] != '-1':
            road_id = cross.road_r[0]
            road = all_road[road_id]
            if road.start_cross == cross_key:
                next_cross = road.end_cross
            else:
                next_cross = road.start_cross
            if next_cross not in close_list:
                map_q[index_y][index_x + 1] = next_cross
                open_list.append(next_cross)
                close_list.append(next_cross)
                all_cross[next_cross].map_index = [index_x + 1, index_y]

        if cross.road_d[0] != '-1':
            road_id = cross.road_d[0]
            road = all_road[road_id]
            if road.start_cross == cross_key:
                next_cross = road.end_cross
            else:
                next_cross = road.start_cross
            if next_cross not in close_list:
                map_q[index_y + 1][index_x] = next_cross
                open_list.append(next_cross)
                close_list.append(next_cross)
                all_cross[next_cross].map_index = [index_x, index_y + 1]
        if cross.road_l[0] != '-1':
            road_id = cross.road_l[0]
            road = all_road[road_id]
            if road.start_cross == cross_key:
                next_cross = road.end_cross
            else:
                next_cross = road.start_cross
            if next_cross not in close_list:
                map_q[index_y][index_x - 1] = next_cross
                open_list.append(next_cross)
                close_list.append(next_cross)
                all_cross[next_cross].map_index = [index_x - 1, index_y]

    return map_q

def createCostMap(max_cross_id,all_road):
    cost_map = np.array([[10000000]*(max_cross_id+1) for i in range(max_cross_id+1)],np.float)
    roads_map = np.array([[-1]*(max_cross_id+1) for i in range(max_cross_id+1)])


    for road_id in all_road.keys():
        if road_id!='-1':
            road = all_road[road_id]
            start_cross = road.start_cross
            end_cross = road.end_cross
            reverse = road.reverse

            roads_map[int(start_cross)][int(end_cross)]=road_id
            roads_map[int(end_cross)][int(start_cross)]=road_id
            if int(reverse)==1:
                cost_map[int(start_cross)][int(end_cross)] = 0
                cost_map[int(end_cross)][int(start_cross)] = 0
            else:
                cost_map[int(start_cross)][int(end_cross)] = 0

    return cost_map,roads_map

if __name__ == '__main__':
    pass
