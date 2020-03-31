# -*- coding: utf-8 -*-

from cell_based_forward_search import CellBasedForwardSearch
from collections import deque
from comp0037_reactive_planner_controller.cell import *
import rospy

# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.

class PlannerWFD(CellBasedForwardSearch):

     # Construct the new planner object
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.fifoQueue = deque()
        self.fifoInnerQueue = deque()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        self.fifoQueue.append(cell)

    def pushCellOntoInnerQueue(self,cell):
        self.fifoInnerQueue.append(cell)

    # Check the queue size is zero
    def isQueueEmpty(self):
        return not self.fifoQueue

    def isInnerQueueEmpty(self):
        return not self.fifoInnerQueue

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.fifoQueue.popleft()
        return cell
    
    def popCellFromInnerQueue(self):
        cell = self.fifoInnerQueue.popleft()
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass

    def resolveInnerDuplicate(self,cell,parentCell):
        # Nothing to do in self case
        pass

    def outerBFS(self,startCoords,isFrontierCell):
        #This function performs the outer BFS, where it searches from the robot's location until it 
        #finds a frontier cell. If it finds a frontier cell,
        #the search is stopped and that cell is then returned.
        #The cell is returned to the updateFrontiers() function which called this search.
        print("outerBFS startcoords: " + str(startCoords))
        self.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.start = self.searchGrid.getCellFromCoords(startCoords)

        #if self.start.label is CellLabel.OBSTRUCTED:
        #    return False
        
        self.start.label = CellLabel.START
        self.start.pathCost = 0

        if rospy.is_shutdown():
            return False

        # Draw the initial state
        #self.resetGraphics()
        
        #Initialise array of frontier cells identified
        frontiers = list()

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        while (self.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging
            if rospy.is_shutdown():
                return False
            
            cell = self.popCellFromQueue()
            #For each cell popped off the fifoQueue as the search algorithm 
            #visits the cell, check if the cell is a frontier cell
            #If it is a frontier cell, stop the search and return that cell
            if (isFrontierCell(cell.coords[0],cell.coords[1]) is True):
                print(isFrontierCell(cell.coords[0],cell.coords[1]))
                frontiers.append(cell)
                frontiers = self.innerBFS(cell.coords,isFrontierCell,frontiers)
            #Visit next set of cells as usual.
            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.hasCellBeenVisitedAlready(nextCell) == False) and (nextCell not in frontiers):
                    self.markCellAsVisitedAndRecordParent(nextCell, cell)
                    self.pushCellOntoQueue(nextCell)
                    self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                else:
                    self.resolveDuplicate(nextCell, cell)

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.markCellAsDead(cell)

            # Draw the update if required
            self.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()

        return frontiers

    def innerBFS(self,startCoords,isFrontierCell,frontiers):
        # This function performs the inner BFS function.
        # This is where the search starts from the initial frontier cell given by
        # the outer BFS. It then searches and identifies all the neighbouring frontier cells,
        # that form up a continuous frontier.
        # All the frontier cells are appended to the list frontiers
        # So that frontiers contains all the frontier cells of a continuous frontier
        print("test")
        self.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isInnerQueueEmpty() == False):
            self.popCellFromInnerQueue()

        # Check the start and end are not occupied. Note that "0.5" means
        # "don't know" which is why it is used as the threshold for detection.
        if (self.occupancyGrid.getCell(startCoords[0], startCoords[1]) > 0.5):
            return False

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.start = self.searchGrid.getCellFromCoords(startCoords)


        
        self.start.label = CellLabel.START
        self.start.pathCost = 0

        if rospy.is_shutdown():
            return False

        # Draw the initial state
        #self.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoInnerQueue(self.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        while (self.isInnerQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging
            if rospy.is_shutdown():
                return False
            
            cell = self.popCellFromInnerQueue()
            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.hasCellBeenVisitedAlready(nextCell) == False):
                    #Only add cells onto the queue if the cell is a frontier cell
                    if isFrontierCell(nextCell.coords[0],nextCell.coords[1]) and (nextCell not in frontiers):
                        self.markCellAsVisitedAndRecordParent(nextCell, cell)
                        self.pushCellOntoInnerQueue(nextCell)
                        self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                    else:
                        continue
                else:
                    self.resolveDuplicate(nextCell, cell)

            # As we've visited a cell, then we will add them onto the
            # List of frontier cells that we've visited
            frontiers.append(cell)

            # Draw the update if required
            self.drawCurrentState()
        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()

        return frontiers
