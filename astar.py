
"""
Created on Sun Feb 16 19:15:57 2020

@author: Andrew
"""

from math import sqrt
import heapq


# returns list of tuple x and y
def astar(waypoints, x_start, y_start, x_end, y_end):
    # find the nearest waypoints to the start and end coordinates
    dmin_s = -1
    dmin_e = -1
    for waypoint in waypoints:
        d_s = euclideanDistance(waypoint.x,waypoint.y,x_start,y_start)
        d_e = euclideanDistance(waypoint.x,waypoint.y,x_end,y_end)
        if dmin_s == -1 or d_s < dmin_s:
            start = waypoint
            dmin_s = d_s
        if dmin_e == -1 or d_e < dmin_e:
            end = waypoint
            dmin_e = d_e
    # now we have start and end waypoints for the search
    # begin astar graph search here with h=euclideanDistance
    
    goal = goalSearch(waypoints,start,end)
    
    # get path from goal node
    path = [(goal.x,goal.y), (x_end,y_end)]
    node = goal
    while node.parent:
        node = node.parent
        path.insert(0,(node.x,node.y))
    path.insert(0,(x_start,y_start))
    return path
    
    
def goalSearch(waypoints,start,goal):
    explored = []
    # initiate frontier
    frontier = PriorityQueue()
    frontier.push(start,euclideanDistance(start.x,start.y,goal.x,goal.y))
    waypoints.remove(start)
    while True:
        if frontier.isEmpty():
            raise ValueError('No goal node found.')
        else:
            # pop the next node on the frontier
            curr_node, priority = frontier.pop()
            # add to explored
            explored.append(curr_node)
            # check if this contains goal node
            if curr_node == goal:
                return curr_node
           # if not in explored add neighbors to frontier
            for neighbor in curr_node.neighbors:
                if neighbor not in explored:
                    neighbor.parent = curr_node
                    frontier.update(neighbor, priority - euclideanDistance(curr_node.x,curr_node.y,goal.x,goal.y)
                                    + euclideanDistance(curr_node.x,curr_node.y,neighbor.x,neighbor.y)
                                    + euclideanDistance(neighbor.x,neighbor.y,goal.x,goal.y))       
    
    
    
# returns Euclidean Distance between points  
def euclideanDistance(x1,y1,x2,y2):
    xdist = x2 - x1
    ydist = abs(y1 - y2)
    if (ydist > 180):
        ydist = 360 - ydist
    return sqrt(pow(xdist,2) + pow(ydist,2))


class PriorityQueue:

    def __init__(self):
        self.heap = []
        self.count = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1

    def pop(self):
        (priority, _, item) = heapq.heappop(self.heap)
        return (item, priority)

    def isEmpty(self):
        return len(self.heap) == 0

    def update(self, item, priority):
        for index, (p, c, i) in enumerate(self.heap):
            if i == item:
                if p <= priority:
                    break
                del self.heap[index]
                self.heap.append((priority, c, item))
                heapq.heapify(self.heap)
                break
        else:
            self.push(item, priority)







