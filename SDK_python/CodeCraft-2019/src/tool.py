from initialize import *
import random
random.seed(127)

def getData(car_path, road_path, cross_path):
    all_car = {}
    all_road = {}
    all_cross = {}
    car_in_garage_time = {}
    car_in_garage_cross = {}
    car_in_garage_final = {}
    car_file = open(car_path)

    for i in car_file.readlines():
        if i[0] != '(':
            continue
        car_list = i.strip().replace("(", "").replace(")", "").replace(" ", "").split(",")
        car = Car(car_id=car_list[0],
                  start_point=car_list[1],
                  end_point=car_list[2],
                  max_speed=int(car_list[3]),
                  start_time=random.randint(int(car_list[4]),10000))
                  # start_time = int(car_list[4]))
        all_car[car_list[0]] = car
        if car_list[4] not in car_in_garage_time.keys():
            car_in_garage_time[car_list[4]] = []
        car_in_garage_time[car_list[4]].append(car_list[0])
        if car_list[1] not in car_in_garage_cross.keys():
            car_in_garage_cross[car_list[1]] = []
        car_in_garage_cross[car_list[1]].append(car_list[0])

    for cross_id in car_in_garage_cross.keys():
        car_in_garage_final[cross_id] = []
        for time_index in sorted([int(i) for i in list(car_in_garage_time.keys())]):
            time_car = car_in_garage_time[str(time_index)]
            set_car = sorted(list(set(car_in_garage_cross[cross_id]) & set(time_car)))
            car_in_garage_final[cross_id].extend(set_car)


    cross_file = open(cross_path)
    for i in cross_file.readlines():
        if i[0]!='(':
            continue
        cross_list = i.strip().replace("(", "").replace(")", "").replace(" ", "").split(",")
        cross = Cross(cross_id=cross_list[0],
                      road_u=cross_list[1],
                      road_r=cross_list[2],
                      road_d=cross_list[3],
                      road_l=cross_list[4])
        all_cross[cross_list[0]] = cross

    road_file = open(road_path)
    for i in road_file.readlines():
        if i[0]!='(':
            continue
        road_list = i.strip().replace("(", "").replace(")", "").replace(" ", "").split(",")
        road = Road(road_id=road_list[0],
                    road_len=int(road_list[1]),
                    road_speed=int(road_list[2]),
                    num_channel=int(road_list[3]),
                    start_cross=road_list[4],
                    end_cross=road_list[5],
                    reverse=road_list[6])
        all_road[road_list[0]] = road
    all_road['-1'] = Road(road_id='-1',
                          road_len=0,
                          road_speed=0,
                          num_channel=0,
                          start_cross='-1',
                          end_cross='-1',
                          reverse='0')

    for cross_key, cross_value in all_cross.items():
        for road_id in cross_value.cross_road:
            road_map = 0
            if road_id != '-1':
                road = all_road[road_id]
                if road.end_cross == cross_key and road.reverse == '1':
                    road_map = 1
                elif road.start_cross == cross_key:
                    road_map = 0
                cross_value.setRoadMap(road_id, road_map)
                all_cross[cross_key] = cross_value

    return all_car, all_road, all_cross, car_in_garage_final


if __name__ == '__main__':
    all_car, all_road, all_cross, car_in_garage = getData()
    print(car_in_garage['1'])
