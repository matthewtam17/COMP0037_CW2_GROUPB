import rospy

from explorer_node_base import ExplorerNodeBase
from comp0037_reactive_planner_controller.cell import *
from math import sqrt
# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)
        

    def updateFrontiers(self):
        searching = True
        startCoords = self.currentCoords
        while searching:
            if self.useWavefront:
                result = self.outerBFS(startCoords)
                if result:
                    self.frontiers = self.innerBFS(result.coords)
                    if len(self.frontiers) > 5:
                        searching = False
                        return True
                    else:
                        startCoords = (0,0)
                        continue
                else:
                    searching = False
                    return False

    def outerBFS(self,startCoords):
        print("startcoords: " + str(startCoords))
        self.planner.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.planner.isQueueEmpty() == False):
            self.planner.popCellFromQueue()

        # Check the start and end are not occupied. Note that "0.5" means
        # "don't know" which is why it is used as the threshold for detection.
        if (self.occupancyGrid.getCell(startCoords[0], startCoords[1]) > 0.5):
            return False
        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.planner.start = self.planner.searchGrid.getCellFromCoords(startCoords)

        #if self.planner.start.label is CellLabel.OBSTRUCTED:
        #    return False
        
        self.planner.start.label = CellLabel.START
        self.planner.start.pathCost = 0

        if rospy.is_shutdown():
            return False

        # Draw the initial state
        #self.planner.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.planner.markCellAsVisitedAndRecordParent(self.planner.start, None)
        self.planner.pushCellOntoQueue(self.planner.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        while (self.planner.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging
            if rospy.is_shutdown():
                return False
            
            cell = self.planner.popCellFromQueue()
            if self.isFrontierCell(cell.coords[0],cell.coords[1]) is True:
                print("is frontier cell")
                return cell
            cells = self.planner.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.planner.hasCellBeenVisitedAlready(nextCell) == False):
                    self.planner.markCellAsVisitedAndRecordParent(nextCell, cell)
                    self.planner.pushCellOntoQueue(nextCell)
                    self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                else:
                    self.planner.resolveDuplicate(nextCell, cell)

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            self.planner.markCellAsDead(cell)

            # Draw the update if required
            self.planner.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.planner.drawCurrentState()

        #If queue is empty, then this shows that the robot has explored the entire plane
        print("Robot has explored the entire map")
        return False

    def innerBFS(self,startCoords):
        self.planner.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.planner.isQueueEmpty() == False):
            self.planner.popCellFromQueue()

        # Check the start and end are not occupied. Note that "0.5" means
        # "don't know" which is why it is used as the threshold for detection.
        if (self.occupancyGrid.getCell(startCoords[0], startCoords[1]) > 0.5):
            return False

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.planner.start = self.planner.searchGrid.getCellFromCoords(startCoords)


        
        self.planner.start.label = CellLabel.START
        self.planner.start.pathCost = 0

        if rospy.is_shutdown():
            return False

        # Draw the initial state
        #self.planner.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.planner.markCellAsVisitedAndRecordParent(self.planner.start, None)
        self.planner.pushCellOntoQueue(self.planner.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        frontiers = list()
        frontiers.append(self.planner.start)

        while (self.planner.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging
            if rospy.is_shutdown():
                return False
            
            cell = self.planner.popCellFromQueue()
            cells = self.planner.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.planner.hasCellBeenVisitedAlready(nextCell) == False):
                    if self.isFrontierCell(nextCell.coords[0],nextCell.coords[1]) is True:
                        self.planner.markCellAsVisitedAndRecordParent(nextCell, cell)
                        self.planner.pushCellOntoQueue(nextCell)
                        self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                    else:
                        continue
                else:
                    self.planner.resolveDuplicate(nextCell, cell)

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            frontiers.append(cell)

            # Draw the update if required
            self.planner.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.planner.drawCurrentState()

        return frontiers

    def chooseNewDestination(self):

        candidateGood = False
        destination = None
        smallestD2 = float('inf')

        if self.useWavefront:
            maxunknown = 0
            unknownCells = 0
            minDistance = float('inf')
            heuristic = 2
            #Using the imrpoved wavefront frontier detection (WF) algorithm
            # For heuristic = 1, we choose frontier cell that's closest to robot
            # For heuristic = 2, we choose frontier cell with the largest frontier,
            # i.e. with the most neighbouring unknown cells
            for frontier in self.frontiers:
                
                if (frontier.coords[0],frontier.coords[1]) in self.blackList:
                    print("repeat goal")
                    continue
                if heuristic == 2:
                    #For this option, we choose frontier cell with the largest frontier
                    # i.e. most neighbouring frontier cells
                    neighbours = self.planner.getNextSetOfCellsToBeVisited(frontier)
                    for nextCell in neighbours:
                        if (0.45 < self.occupancyGrid.getCell(nextCell.coords[0],nextCell.coords[1]) < 0.55):
                            unknownCells = unknownCells + 1
                    if unknownCells > maxunknown:
                        destination = (frontier.coords[0],frontier.coords[1])
                        candidateGood = True
                        maxunknown = unknownCells
                else:
                    #Choose frontier based on frontier cell that is closet to the robot
                    currentCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates([self.pose.position.x,self.pose.position.y])
                    X = currentCoords[0]
                    Y = currentCoords[1]
                    distance = sqrt((frontier.coords[0]-X)**2+(frontier.coords[1]-Y)**2)
                    if distance < minDistance:
                        destination = (frontier.coords[0],frontier.coords[1])
                        candidateGood = True
                        minDistance = distance
                    

        else:
            # Using the default inefficient algorithm
            for x in range(0, self.occupancyGrid.getWidthInCells()):
                for y in range(0, self.occupancyGrid.getHeightInCells()):
                    candidate = (x, y)
                    if self.isFrontierCell(x, y) is True:
                        candidateGood = True
                        for k in range(0, len(self.blackList)):
                            if self.blackList[k] == candidate:
                                candidateGood = False
                                break
                        
                        if candidateGood is True:
                            d2 = candidate[0]**2+(candidate[1]-0.5*self.occupancyGrid.getHeightInCells())**2

                            if (d2 < smallestD2):
                                destination = candidate
                                smallestD2 = d2

            # If we got a good candidate, use it
        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
