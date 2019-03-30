import numpy as np
from tool import *

def dijkstra(graph,src):
    if graph is None:
        return None
    nodes = [i for i in range(len(graph))]
    visited=[]
    if src in nodes:
        visited.append(src)
        nodes.remove(src)
    else:
        return None
    distance={src:0}
    for i in nodes:
        distance[i]=graph[src][i]
    k=src
    while nodes:
        mid_distance=float('inf')
        for v in visited:
            for d in nodes:
                new_distance = graph[src][v]+graph[v][d]
                if new_distance < mid_distance:
                    mid_distance=new_distance
                    graph[src][d]=new_distance
                    k=d
        distance[k]=mid_distance
        visited.append(k)
        nodes.remove(k)
    return distance

def GenHMatrix(all_cross,all_road):
    all_cross_id_list = list(all_cross.keys())
    all_cross_id_list = sorted([int(x) for x in all_cross_id_list])
    cross_len = len(all_cross_id_list)
    crossIdMap2index = {}
    for i in range(cross_len):
        crossIdMap2index[all_cross_id_list[i]] = i

    Index2crossIdMap = {}
    for i in range(cross_len):
        Index2crossIdMap[i] = all_cross_id_list[i]
    H = np.zeros((cross_len, cross_len),dtype=int)
    H = H + 10000000

    dijkstraMat = np.zeros((cross_len, cross_len),dtype=int)
    dijkstraMat = dijkstraMat + 10000000
    for i in range(cross_len):
         dijkstraMat[i,i] = 0

    for key, road in all_road.items():
        if key != '-1':
            start_cross_id = int(road.start_cross)
            end_cross_id = int(road.end_cross)
            dist = int(road.road_len)
            dijkstraMat[crossIdMap2index[start_cross_id], crossIdMap2index[end_cross_id]] = dist
            if road.reverse:
                dijkstraMat[crossIdMap2index[end_cross_id], crossIdMap2index[start_cross_id]] = dist

    for p in range(cross_len):
        dist = dijkstra(dijkstraMat, p)
        for key in dist.keys():
            dijkstraMat[p, key] = dist[key]

    return dijkstraMat, crossIdMap2index

if __name__ == '__main__':
    pass