
from initialize import *


class AStar:
    class Node:
        def __init__(self, point:Cross, endPoint, g=0,h=0):
            self.point = point
            self.father = None
            self.g = g
            self.h = h

    def __init__(self,startPoint, endPoint,cost_map=None,all_cross=None,h_map=None,all_road=None,crossid2index=None):

        self.cost_map=cost_map
        self.openList = []
        self.closeList = []
        self.all_cross=all_cross
        self.h_map=h_map
        self.all_road=all_road

        self.startPoint = startPoint
        self.endPoint = endPoint
        self.crossid2index= crossid2index


    def getMinNode(self):

        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode

    def pointInCloseList(self, point):
        for node in self.closeList:
            if node.point.cross_id == point.cross_id:
                return True
        return False

    def pointInOpenList(self, point):
        for node in self.openList:
            if node.point.cross_id == point.cross_id:
                return node
        return None

    def endPointInCloseList(self):
        for node in self.closeList:
            if node.point == self.endPoint:
                return node
        return None
    def getThisRoadNextCross(self,cross,road:Road)->Cross:
        if road.end_cross==cross.cross_id:
            return self.all_cross[road.start_cross]
        else:
            return self.all_cross[road.end_cross]
    def searchNear(self, minF:Node, direction):
        road_id = minF.point.cross_road[int(direction)]
        if road_id=='-1':
            return
        currentPoint = self.getThisRoadNextCross(minF.point, self.all_road[road_id])

        if self.pointInCloseList(currentPoint):
            return

        start_cross_id = minF.point.cross_id
        end_cross_id = currentPoint.cross_id
        step=self.cost_map[int(start_cross_id)][int(end_cross_id)]


        currentNode = self.pointInOpenList(currentPoint)
        if not currentNode:

            currentNode = AStar.Node(currentPoint, self.endPoint, g=minF.g + step,h=self.h_map[self.crossid2index[int(self.startPoint.cross_id)]][self.crossid2index[int(self.endPoint.cross_id)]])
            currentNode.father = minF
            self.openList.append(currentNode)
            return

        if minF.g + step < currentNode.g:
            currentNode.g = minF.g + step
            currentNode.father = minF

    def start(self):

        startNode = AStar.Node(self.startPoint, self.endPoint,h=self.h_map[self.crossid2index[int(self.startPoint.cross_id)]][self.crossid2index[int(self.endPoint.cross_id)]])
        self.openList.append(startNode)

        while True:

            minF = self.getMinNode()

            self.closeList.append(minF)
            self.openList.remove(minF)
            self.searchNear(minF, '0')
            self.searchNear(minF, '1')
            self.searchNear(minF, '2')
            self.searchNear(minF, '3')
            point = self.endPointInCloseList()
            if point:

                cPoint = point
                pathList = []
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point.cross_id)
                        cPoint = cPoint.father
                    else:


                        return list(reversed(pathList))
            if len(self.openList) == 0:
                return None

if __name__ == '__main__':
    pass

