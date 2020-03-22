# -*- coding: utf-8 -*-
from cell_based_forward_search import CellBasedForwardSearch
import Queue
import math
import numpy as np
from math import sqrt

# This class implements the A* Planning
# algorithm. It works by using a priority queue based on the cost-to-go for each cell.
# the element with the highest priority is always popped first.

class AStarPlanner(CellBasedForwardSearch):

    # Construct the new planner object
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.astarQueue = Queue.PriorityQueue()

        #Determine the weighting of the scaled A* algorithm
        self.w = 1
        
        #Value that determines what heuristic to use!
        #We use this method to set the heuristic as its more simple to implement than 
        #passing arguments into the function

        self.heuristic = 0
        #Heuristics available:
        #0:Euclidean Distance 
        #1:Octile Distance
        #2:Manhattan Distance (non-admissible)
        #3:Minkowski Distance
        #4:Cosine Distance
        #5:Euclidean SQUARE Distance (non-admissible)
        #-1: constant

    # Find the cell's "priority value" and add onto the priority queue.
    # We can simply add the cell onto the back of the queue because the get() function returns us the highest priority cell
    def pushCellOntoQueue(self, cell):
        cell.pathCost = self.computePathCost(cell)
        self.astarQueue.put((cell.pathCost,cell))
        #Checks if the new cell length is more than the existing max cell length.
        #If it is then update the max queue length value.
        #if self.astarQueue.qsize() > self.max_queue_length:
        #    self.max_queue_length = self.astarQueue.qsize()
    #  Calculates the Euclidean distance to the goal
    def cal_heuristic(self,cell):

        if self.heuristic == 0:
            #Euclidean distance to the goal. Admissible heuristic.
            return math.sqrt((self.goal.coords[0]-cell.coords[0])**2 + (self.goal.coords[1]-cell.coords[1])**2)
        elif self.heuristic == 1:
            #Octile distance to the goal. Admissible if the robot moves in 8 directions.
            return max(abs(cell.coords[0]-self.goal.coords[0]),abs(cell.coords[1]-self.goal.coords[1]))+(sqrt(2)-1)*min(abs(cell.coords[0]-self.goal.coords[0]),abs(cell.coords[1]-self.goal.coords[1]))
        elif self.heuristic == 2:
            #Manhattan distance to the goal. Only admissible if the robot moves in four directions only.
            return abs(cell.coords[0]-self.goal.coords[0])+abs(cell.coords[1]-self.goal.coords[1])
        elif self.heuristic == 3:
            #Minkowski sum distance to the goal.
            h=10
            return ((self.goal.coords[0]-cell.coords[0])**h + (self.goal.coords[1]-cell.coords[1])**h)**(1/h)
        elif self.heuristic == 4:
            #Cosine heuristic to the goal.
            x = self.goal.coords
            y = cell.coords
            return np.dot(x,y)/(np.sqrt(np.dot(x,x))*np.sqrt(np.dot(y,y)))
        elif self.heuristic == 5:
            #Euclidean Square - demonstration of a NON-admissible heuristic
            return ((self.goal.coords[0]-cell.coords[0])**2 + (self.goal.coords[1]-cell.coords[1])**2)
        else:
            # Returns a constant as the heuristic
            return 1000

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.astarQueue.empty()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        priority, cell = self.astarQueue.get()
        return cell

    # Function to compute the path cost - we use this to compute the distance of the 
    # planned path, disregarding the terrain cost. We use this to compare with the 
    # distance of the actual travelled path in part 2.
    def computePathCost(self,cell):
        itercell = cell.parent
        pathCost = 0
        if itercell:
            pathCost = self.computeLStageAdditiveCost(itercell,cell)  + (self.w)*(self.cal_heuristic(cell))
        while (itercell is not None):
            pathCost = pathCost + self.computeLStageAdditiveCost(itercell.parent, itercell)
            itercell = itercell.parent
        return pathCost

    #resolveDuplicate function. The A* includes a heuristic of the distance to the goal
    # in the pathCost for each cell here, which is used to determine the priority for
    # each cell. We have to disregard that as the resolveDuplicate() function needs to
    #function the same as that for Dijkstra, or else we will end up with a path that loops
    #around itself and does not connect with the start cell.
    def resolveDuplicate(self, cell, parentCell):
        newPathpathCost = parentCell.pathCost-(self.w)*(self.cal_heuristic(parentCell)) + self.computeLStageAdditiveCost(parentCell,cell)
        if newPathpathCost < (self.computePathCost(cell)-(self.w)*(self.cal_heuristic(cell))) and cell != parentCell.parent:
            cell.parent = parentCell
            cell.pathCost = newPathpathCost
            self.pushCellOntoQueue(cell)

    # Reorder the queue. I don't see another way to do this, other than
    # create a new queue and copy over tuple-by-tuple. This rebuilds
    # the heap trees.
    def reorderPriorityQueue(self):
        newQueue = PriorityQueue()

        while self.priorityQueue.empty() is False:
            tuple = self.priorityQueue.get()
            newQueue.put(tuple)
             
        self.priorityQueue = newQueue
            
