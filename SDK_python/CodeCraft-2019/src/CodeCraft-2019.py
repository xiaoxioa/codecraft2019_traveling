from tool import *
from Astart_new import *
import numpy as np
import sys
from Astart_new import *
from Dijkstra import *
car_path = sys.argv[1]
road_path = sys.argv[2]
cross_path = sys.argv[3]

all_car, all_road, all_cross, car_in_garage = getData(car_path, road_path, cross_path)
h_map,crossid2index = GenHMatrix(all_cross,all_road)
cost_map,roads_map = createCostMap(max([int(i) for i in all_cross.keys()]),all_road)


wait_car_list = []
undeal_car_list = []
cars_on_road = []
# cars_not_on_road = sorted(all_car.keys())
end_car_list = []

cross_open_list = []

all_car_id_list = list(all_car.keys())
all_road_id_list = list(all_road.keys())
all_cross_id_list = list(all_cross.keys())
all_cross_id_list = sorted([int(x) for x in all_cross_id_list])
all_cross_id_list = [str(x) for x in all_cross_id_list]


def isCarPassCross(car_id):
    """
    :param car_id:
    :return:
     0 for can't pass
     1 for pass
    """
    car = all_car[car_id]
    if car.state == 'G':
        return 0
    else:
        [current_road_id, current_road_map, current_channel, current_ind] = car.location
        current_road = all_road[current_road_id]
        # rest distance in current road
        s1 = current_road.road_len - current_ind - 1
        if s1 < car.actual_speed:
            if car.next_cross == car.end_cross:
                return 0
            return 1
        else:
            return 0


def getCarOrderForPerRoad(road_map):
    """
    Helper Function For getCarOrderList
    """
    channel, roadLen = road_map.shape
    orderList = []
    for i in range(roadLen - 1, -1, -1):
        for j in range(channel):
            id = str(road_map[j, i])
            if road_map[j, i] != 0:
                orderList.append(id)
    return orderList


def getCarOrderList(cross):
    """
    :param cross:
    :return: list)
    """

    turnLeftConflict = {'road_u': 'road_l', 'road_r': 'road_u', 'road_d': 'road_r', 'road_l': 'road_d'}
    turnRightConflictWithD = {'road_u': 'road_r', 'road_r': 'road_d', 'road_d': 'road_l', 'road_l': 'road_u'}
    turnRightConflictWithL = {'road_u': 'road_d', 'road_r': 'road_l', 'road_d': 'road_u', 'road_l': 'road_r'}

    # Get the Ascending Road_id List
    roadIdList = []
    if cross.road_u[0] != '-1':
        roadIdList.append(int(cross.road_u[0]))
    else:
        roadIdList.append(float("inf"))
    if cross.road_r[0] != '-1':
        roadIdList.append(int(cross.road_r[0]))
    else:
        roadIdList.append(float("inf"))
    if cross.road_d[0] != '-1':
        roadIdList.append(int(cross.road_d[0]))
    else:
        roadIdList.append(float("inf"))
    if cross.road_l[0] != '-1':
        roadIdList.append(int(cross.road_l[0]))
    else:
        roadIdList.append(float("inf"))
    roadIdList.sort()
    # print(roadIdList)

    # Get Helper Data Structure
    roadDirectionInverse = {'road_u': cross.road_u[0], 'road_r': cross.road_r[0], 'road_d': cross.road_d[0], 'road_l': cross.road_l[0]}
    roadDirection = {}
    for key in roadDirectionInverse.keys():
        if roadDirectionInverse[key] != '-1':
            roadDirection[roadDirectionInverse[key]] = key
    # Get Road Map
    roadmapDict = {}
    roadMapIndDict = {'road_u': 1 - cross.road_u[1], 'road_r': 1 - cross.road_r[1], 'road_d': 1 - cross.road_d[1],
                      'road_l': 1 - cross.road_l[1]}
    for key in roadDirection.keys():
        curRoad = all_road[key]
        road_map_ind = roadMapIndDict[roadDirection[key]]
        # roadmapDict[key] = getCarOrderForPerRoad(curRoad.road_map[road_map_ind])
        if road_map_ind != 2:
            roadmapDict[key] = getCarOrderForPerRoad(curRoad.road_map[road_map_ind])
        else:
            if curRoad.end_cross == cross.cross_id:
                roadmapDict[key] = getCarOrderForPerRoad(curRoad.road_map[0])
            else:
                roadmapDict[key] = []

    # Get Car Diection
    carDirection = {}
    for key in roadmapDict.keys():
        curCarList = roadmapDict[key]
        for c in curCarList:
            curCar = all_car[str(c)]
            carDirection[str(c)] = curCar.next_direction

    # Get firstCarPerRoad
    firstCarPerRoad = [0,0,0,0]
    for i in range(4):
        if str(roadIdList[i]) in roadDirection.keys() and len(roadmapDict[str(roadIdList[i])]) > 0:
            firstCarPerRoad[i] = int(roadmapDict[str(roadIdList[i])][0])
    priorityCarList = []
    curRoadInd = 0
    while sum(firstCarPerRoad) > 0:
        if firstCarPerRoad[curRoadInd] == 0:
            curRoadInd = (curRoadInd + 1) % 4
        else:
            if carDirection[str(int(firstCarPerRoad[curRoadInd]))] == "D":
                priorityCarList.append(str(int(firstCarPerRoad[curRoadInd])))
                roadmapDict[str(int(roadIdList[curRoadInd]))].pop(0)
                if len(roadmapDict[str(roadIdList[curRoadInd])]) > 0:
                    firstCarPerRoad[curRoadInd] = int(roadmapDict[str(int(roadIdList[curRoadInd]))][0])
                else:
                    firstCarPerRoad[curRoadInd] = 0
                    curRoadInd = (curRoadInd + 1) % 4
            elif carDirection[str(int(firstCarPerRoad[curRoadInd]))] == "L":
                curRoadDirection = roadDirection[str(int(roadIdList[curRoadInd]))]
                conflictRoadDiection = roadDirectionInverse[turnLeftConflict[curRoadDirection]]
                tempRoadIndex = '-1' if conflictRoadDiection == '-1' else roadIdList.index(int(conflictRoadDiection))
                if tempRoadIndex != '-1' and firstCarPerRoad[tempRoadIndex] != 0 and carDirection[str(int(firstCarPerRoad[tempRoadIndex]))] == "D":
                    curRoadInd = tempRoadIndex
                else:
                    priorityCarList.append(str(int(firstCarPerRoad[curRoadInd])))
                    roadmapDict[str(int(roadIdList[curRoadInd]))].pop(0)
                    if len(roadmapDict[str(roadIdList[curRoadInd])]) > 0:
                        firstCarPerRoad[curRoadInd] = int(roadmapDict[str(int(roadIdList[curRoadInd]))][0])
                    else:
                        firstCarPerRoad[curRoadInd] = 0
                        curRoadInd = (curRoadInd + 1) % 4
            elif carDirection[str(firstCarPerRoad[curRoadInd])] == "R":
                # Check D first
                curRoadDirection = roadDirection[str(int(roadIdList[curRoadInd]))]
                conflictRoadDiection = roadDirectionInverse[turnRightConflictWithD[curRoadDirection]]
                tempRoadIndexD = '-1' if conflictRoadDiection == '-1' else roadIdList.index(int(conflictRoadDiection))
                curRoadDirection = roadDirection[str(int(roadIdList[curRoadInd]))]
                conflictRoadDiection = roadDirectionInverse[turnRightConflictWithL[curRoadDirection]]
                tempRoadIndexL = '-1' if conflictRoadDiection == '-1' else roadIdList.index(int(conflictRoadDiection))
                if tempRoadIndexD != '-1' and firstCarPerRoad[tempRoadIndexD] != 0 and carDirection[str(int(firstCarPerRoad[tempRoadIndexD]))] == "D":
                    curRoadInd = tempRoadIndexD
                elif tempRoadIndexL != '-1' and firstCarPerRoad[tempRoadIndexL] != 0 and carDirection[str(int(firstCarPerRoad[tempRoadIndexL]))] == "L":
                    curRoadInd = tempRoadIndexL
                else:
                    priorityCarList.append(str(int(firstCarPerRoad[curRoadInd])))
                    roadmapDict[str(int(roadIdList[curRoadInd]))].pop(0)
                    if len(roadmapDict[str(roadIdList[curRoadInd])]) > 0:
                        firstCarPerRoad[curRoadInd] = int(roadmapDict[str(int(roadIdList[curRoadInd]))][0])
                    else:
                        firstCarPerRoad[curRoadInd] = 0
                        curRoadInd = (curRoadInd + 1) % 4
    priorityCarListinWaitingList = [x for x in priorityCarList if x in wait_car_list]
    return priorityCarListinWaitingList


def driveAllCarJustOnRoadToEndState(road):
    """
    :param road: obj
    :return:
    """
    global wait_car_list
    global undeal_car_list
    if road.reverse == '1':
        rng = 2
    else:
        rng = 1
    for road_map_id in range(rng):
        road_map = road.road_map[road_map_id]
        for channel_road in road_map:
            for index in range(len(channel_road) - 1, -1, -1):
                if channel_road[index] != 0:  # find a car
                    car_id = str(channel_road[index])
                    # if car passes road, add car into wait list
                    if isCarPassCross(car_id) == 1:
                        carOnRoadPathPlan(car_id)
                        wait_car_list.append(car_id)
                        undeal_car_list.remove(car_id)
                    else:
                        # result, road_id, road_map_id, channel_id, index = isCarCanMove(str(int(car_id)), 0)
                        result = moveWaitingCar(car_id)
                        # print(car_id, result, road_id, road_map_id, channel_id, index)
                        if result == 0:
                            wait_car_list.append(str(int(car_id)))
                            undeal_car_list.remove(str(int(car_id)))
                        else:
                            # print(car_id)
                            undeal_car_list.remove(str(int(car_id)))


def getDistAhead(car_id, road, road_map, channel):
    """
    FUNCTION :  get spare distance ahead in "road":this channel
    :param car_id:
    :param road: object
    :param road_map:
    :param channel:
    :return: s, carid_ahead
            carid_ahead == '-1' for no car ahead
    """
    c = road.road_map[road_map][channel, :]  # a ndarray
    l = c[c > 0]
    '''
    example:
    c = [0 3 0 1 2 5 0 4 0 0 0]
    l = [3 1 2 5 4]
    '''

    car_id = int(car_id)
    # to find the first car to meet
    ind = np.argwhere(l == car_id)
    ind = int(ind) if ind.size > 0 else -1  # >>>car_ahead = l[ind+1]

    # current car index
    ind2 = int(np.argwhere(c == car_id)) if np.argwhere(c == car_id).size > 0 else -1

    s = len(c) - ind2 - 1
    carid_ahead = -1
    for i in range(ind2+1, len(c)):
        if c[i] != 0 and c[i] != ind2:
            carid_ahead = c[i]
            s = i - ind2 - 1
            break

    return s, str(carid_ahead)


def moveCarToRoad(car, next_road, channel, s):
    """
    FUNCTION: move "car" to "next_road" [channel, s]
    UPDATE:
        car: location，next_cross, route
        road: road_map
    :param car: obj
    :param next_road: obj
    :param s: move distance
    :return: Ture/False
    """
    car_id = car.car_id
    if car.state == 'G':
        # car_id in cars_not_on_road:
        current_road_id = '-1'
        current_road_map = -1
        current_channel = 0
        current_ind = -1

    else:
        [current_road_id, current_road_map, current_channel, current_ind] = car.location
        current_road = all_road[current_road_id]
        # update current road map
        current_road.road_map[current_road_map][current_channel, current_ind] = 0
    if car.state == 'E':
        car.location = [0, 0, 0, 0]
        car.next_next_cross = car.next_cross = car.end_cross
        return True
    else:
        if current_road_id == next_road.road_id and channel == current_channel:
            # still in this road
            if current_road.road_map[current_road_map][current_channel, current_ind + s] != 0:
                return ValueError("bu dui a ", car_id)
            current_road.road_map[current_road_map][current_channel, current_ind + s] = car_id
            car.location[3] = current_ind + s
        elif current_road_id != next_road.road_id:
            # turn to road
            s = s-1
            road_map = car.next_road_map_id
            if next_road.road_map[road_map][channel, s] != 0:
                return ValueError("bu dui a ", car_id)
            next_road.road_map[road_map][channel, s] = car_id
            car.location = [next_road.road_id, road_map, channel, s]
            car.next_cross = car.next_next_cross
            car.route.append(next_road.road_id)
            car.cross_by.append(car.next_cross)
        else:
            return ValueError("Where you want to go!")
        return True


def moveWaitingCar(car_id):
    """
    :param car_id:
    :return: 0/1 -1
    0 for wait
    1 for success
    -1 for car in garage drive failue
    """
    """
    update car:(actual_speed, state)
    update end_car_list
    """
    car = all_car[car_id]
    global cars_on_road
    global end_car_list
    # car in Garage
    if car.state == 'G':
        next_road_id = car.next_road
        next_road = all_road[next_road_id]
        road_map = car.next_road_map_id
        v2 = next_road.road_speed
        for i in range(next_road.num_channel):
            s, car_id_ahead = getDistAhead(car_id, next_road, road_map, i)
            if s > 0:
                new_channel = i
                if s <= min(v2, car.max_speed):
                    if car_id_ahead in end_car_list:
                        car_ahead = all_car[car_id_ahead]
                        speed = min(car.max_speed, car_ahead.actual_speed)
                        moveCarToRoad(car, next_road, new_channel, s)
                        car.updateState(speed, 'R')
                        end_car_list.append(car_id)
                    else:
                        return 0
                else:  # s >= speed
                    # no car ahead
                    speed = min(car.max_speed, v2)
                    moveCarToRoad(car, next_road, new_channel, speed)
                    car.updateState(speed, 'R')
                    end_car_list.append(car_id)
                break
            if i == next_road.num_channel - 1 and car_id_ahead in end_car_list:
                return -1
        return 1
    else:
        [current_road_id, current_road_map, current_channel, current_ind] = car.location
        current_road = all_road[current_road_id]
        s, car_id_ahead = getDistAhead(car_id, current_road, current_road_map, current_channel)
        # rest distance in current road
        s1 = current_road.road_len - car.location[3] - 1
        # if there is a car ahead
        s = min(s, s1)   # move s at most
        if s >= car.actual_speed:  # car can move as usual
            moveCarToRoad(car, current_road, current_channel, car.actual_speed)
            end_car_list.append(car_id)
        else:  # s < car.actual_speed
            if car_id_ahead == '-1':
                if car.next_cross == car.end_cross:  # s1 < car.actual_speed
                    # car reach end cross:
                    car.updateState(0, 'E')
                    moveCarToRoad(car, current_road, current_channel, car.actual_speed)
                    # remove car from cars_on_road
                    cars_on_road.remove(car_id)
                else:
                    # move to next road
                    next_road_id = car.next_road
                    next_road = all_road[next_road_id]
                    # speed limitation in next road
                    v2 = next_road.road_speed
                    s2 = max(v2 - s1, 0)
                    if s2 == 0:  # stay in current road
                        moveCarToRoad(car, current_road, current_channel, s)
                        car.actual_speed = min(car.max_speed, v2)
                        end_car_list.append(car_id)
                    else:
                        # turn to next road
                        road_map = car.next_road_map_id
                        for i in range(next_road.num_channel):
                            next_s, next_car_ahead = getDistAhead(car_id, next_road, road_map, i)
                            if next_s > 0:
                                new_channel = i
                                if next_s <= min(v2, car.max_speed):
                                    if next_car_ahead in end_car_list:
                                        car_ahead = all_car[next_car_ahead]
                                        car.actual_speed = min(car.actual_speed, car_ahead.actual_speed)
                                        moveCarToRoad(car, next_road, new_channel, next_s)
                                        end_car_list.append(car_id)
                                    else:
                                        return 0
                                else:  # s > speed
                                    # no car ahead
                                    car.actual_speed = min(car.max_speed, v2)
                                    moveCarToRoad(car, next_road, new_channel, car.actual_speed)
                                    end_car_list.append(car_id)
                                break
                            elif i == next_road.num_channel - 1:
                                if next_car_ahead in end_car_list:
                                    # stay
                                    moveCarToRoad(car, current_road, current_channel, s1)
                                    car.actual_speed = min(car.max_speed, v2)
                                    end_car_list.append(car_id)
                                else:
                                    return 0
            elif car_id_ahead in end_car_list:
                car_ahead = all_car[car_id_ahead]
                moveCarToRoad(car, current_road, current_channel, s)
                # car.updateState()
                car.actual_speed = min(car_ahead.actual_speed, car.actual_speed)
                end_car_list.append(car_id)
            else:
                return 0
        return 1


def updateCostMap():
    """
    :param: cost_map(numpy) n*n
    :return:
    """
    for road in all_road_id_list:
        start_cross = all_road[road].start_cross
        end_cross = all_road[road].end_cross
        forwardNum = all_road[road].forwardNum
        backwardNum = all_road[road].backwardNum

        # if end_cross_index[0]<len(cross_map[0])*0.8 and end_cross_index[0]>len(cross_map[0])*0.2 and end_cross_index[1]<len(cross_map)*0.8 and end_cross_index[1]>len(cross_map)*0.2:
        if cost_map[int(start_cross)][int(end_cross)] != 10000000:
            cost_map[int(start_cross)][int(end_cross)] = (int(forwardNum)) * 100 / all_road[road].carCapcity
        if cost_map[int(end_cross)][int(start_cross)] != 10000000:
            cost_map[int(end_cross)][int(start_cross)] = (int(backwardNum)) * 100 / all_road[road].carCapcity


def carInGaragePathPlan(car):
    """
    update next cross, road, road_map_id, next direction, next_next_cross
    :param car:  obj
    :return:
    """
    # print(car)
    next_cross_id = car.start_cross
    cross = all_cross[next_cross_id]
    start_cross_id = car.next_cross
    end_cross_id = car.end_cross
    next_next_cross_id = getBestRoute(all_cross[start_cross_id], all_cross[end_cross_id])

    car.next_next_cross = next_next_cross_id
    next_road = '-1'
    next_direction = 'D'
    next_road_map_id = 0
    # print(car.start_cross, car.end_cross, cross_list, next_cross_id, next_next_cross_id)
    # next_cross = all_cross[next_next_cross_id]
    for road_id, road_map_id_i in cross.getSortedRoadId().items():
        if road_id != '-1':
            if all_road[road_id].start_cross == next_next_cross_id or \
                    all_road[road_id].end_cross == next_next_cross_id:
                next_road = road_id
                next_road_map_id = road_map_id_i
                break
    car.cross_index = -1
    car.next_cross = car.start_cross
    car.next_road = next_road
    car.next_direction = next_direction
    car.next_road_map_id = next_road_map_id


def driveCarInGarage(time):
    global car_in_garage
    global cars_on_road
    for cross_id, cross_wait_car in car_in_garage.items():
        # cross_wait_car = car_in_garage[cross_id]
        for car_id in cross_wait_car.copy():
            car = all_car[car_id]
            if int(car.start_time) <= time:
                carInGaragePathPlan(car)
                result = moveWaitingCar(car_id)
                if result == -1:
                    # consider penalization
                    break
                elif result == 1:
                    cross_wait_car.remove(car_id)
                    cars_on_road.append(car_id)
                    # cars_not_on_road.remove(car_id)
            else:
                break


def getBestRoute(start_cross, end_cross):

        aStar = AStar(start_cross,end_cross,
                      cost_map=cost_map, all_cross=all_cross,all_road=all_road,h_map=h_map, crossid2index=crossid2index)
        pathList = aStar.start()
        point = pathList[0]
        next_cross_id = point
        return str(int(next_cross_id))

def carOnRoadPathPlan(car_id):
    """
    :return:
    """
    car = all_car[car_id]
    next_cross_id = car.next_cross
    cross = all_cross[next_cross_id]
    start_cross_id = car.next_cross
    last_cross = car.cross_by[-1]
    temp = cost_map[int(start_cross_id)][int(last_cross)]
    cost_map[int(start_cross_id)][int(last_cross)] = 1000000
    end_cross_id = car.end_cross
    if start_cross_id != end_cross_id:
        next_next_cross_id = getBestRoute(all_cross[start_cross_id], all_cross[end_cross_id])
        next_next_cross = all_cross[next_next_cross_id]
        car.next_next_cross = next_next_cross_id
        next_direction = '-1'
        road_id = car.location[0]
        road_list1 = cross.cross_road
        road_list2 = next_next_cross.cross_road
        next_road = [i for i in road_list1 if i in road_list2 and i != '-1'][0]
        ind1 = road_list1.index(road_id)
        ind2 = road_list1.index(next_road)
        dis = abs(ind1 - ind2)
        if dis == 1:
            next_direction = 'L'
        elif dis == 2:
            next_direction = 'D'
        elif dis == 3:
            next_direction = 'R'
        if all_road[next_road].start_cross == next_next_cross_id:
            next_road_map_id = 1
        else:
            next_road_map_id = 0
        car.next_road = next_road
        car.next_direction = next_direction
        car.next_road_map_id = next_road_map_id
    cost_map[int(start_cross_id)][int(last_cross)] = temp
    return


def getRoadIdList(undeal_car_list):
    """
    Get ordered, unique road id list
    :param undeal_car_list:
    :return: road_id_list:
    """
    road_id_list = []
    for i in undeal_car_list:
        car = all_car[i]
        road_id_list.append(car.location[0])
    road_id_list = list(set(road_id_list))
    road_id_list = sorted([int(x) for x in road_id_list])
    road_id_list = [str(x) for x in road_id_list]
    return road_id_list

def notAllCarEnd():
    """
    :return:
    """
    for id, car in all_car.items():
        if car.state != 'E':
            return True
    return False


def getAllCross(car_list):
    """
    :param car_list:
    :return:
    """
    temp = []
    for car_id in car_list:
        car = all_car[car_id]
        temp.append(car.next_cross)
    temp = list(set(temp))
    temp = sorted([int(x) for x in temp])
    temp = [str(x) for x in temp]
    return temp

def main():
    # car_path = sys.argv[1]
    # road_path = sys.argv[2]
    # cross_path = sys.argv[3]
    answer_path = sys.argv[4]
    global undeal_car_list
    global wait_car_list
    global cars_on_road

    global end_car_list

    global cross_open_list
    # # car_path = "./1-map-exam-1/car.txt"
    # # road_path = "./1-map-exam-1/road.txt"
    # # cross_path = "./1-map-exam-1/cross.txt"
    # # answer_path = './answer.txt'
    # # ************************************* M A I N *******************************************#
    # # load .txt files
    # all_car, all_road, all_cross, car_in_garage = getData(car_path, road_path, cross_path)
    system_time = 0
    while notAllCarEnd():
        # print(system_time)
        system_time = system_time + 1
        
        end_car_list = []
        wait_car_list = []
        undeal_car_list = cars_on_road.copy()
        driveCarInGarage(system_time)
        # if len(undeal_car_list) > 0:
        #     CarOnRoadPathPlan(undeal_car_list)
        while len(undeal_car_list) != 0:
            road_id_list = getRoadIdList(undeal_car_list)
            for road_id in road_id_list:
                road = all_road[road_id]
                if road_id != '-1':
                    driveAllCarJustOnRoadToEndState(road)

        cross_open_list = getAllCross(wait_car_list)
        while len(wait_car_list) != 0:
            cross_list = cross_open_list.copy()
            cross_last_count = len(cross_open_list)
            for cross_id in cross_list:
                cross = all_cross[cross_id]
                car_order_list = getCarOrderList(cross)
                if len(car_order_list) > 0:
                    result = 1
                    car_count = 0
                    for car_id in car_order_list:
                        car = all_car[car_id]
                        result = moveWaitingCar(car_id)
                        if result == 0:
                            break
                        else:
                            car_count = car_count + 1
                            wait_car_list.remove(car_id)
                    if result == 0:
                        if car_count == 0:
                            continue
                        break
                    else:
                        cross_open_list.remove(cross_id)
                else:
                    cross_open_list.remove(cross_id)
            if cross_last_count == len(cross_open_list):
                f = open(answer_path, 'w')
                for car_id, car in all_car.items():
                    if len(car.route) == 0:
                        continue
                    else:
                        f.write("(%s,%s," % (car_id, str(car.start_time)) + ",".join(car.route) + ")\n")
                f.close()
                return

        for i, road in all_road.items():
            road.updateNum()
        updateCostMap()


if __name__ == '__main__':

    main()