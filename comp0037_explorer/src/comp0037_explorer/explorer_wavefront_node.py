import rospy

from explorer_node_base import ExplorerNodeBase
from comp0037_reactive_planner_controller.fifo_planner import FIFOPlanner

class ExplorerWaveFrontNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)

        self.blackList = []
        self.frontiers = []

    def updateFrontiers(self):
        result = self.outerBFS(self.pose.coords)
        if result == False:
            return False
        else
            self.frontiers = self.innerBFS(result.coords)
            return True

    #The outer search routine that runs BFS until it is able to locate a frontier cell
    def outerBFS(self,startCoords):
        self.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()

        # Check the start and end are not occupied. Note that "0.5" means
        # "don't know" which is why it is used as the threshold for detection.
        if (self.occupancyGrid.getCell(startCoords[0], startCoords[1]) > 0.5):
            return False

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
        self.resetGraphics()

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
            if isFrontierCell(cell.coords[0],cell.coords[1]):
                return cell
            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.hasCellBeenVisitedAlready(nextCell) == False):
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

        #If queue is empty, then this shows that the robot has explored the entire plane
        print("Robot has explored the entire map")
        return False

    def innerBFS(self):
        self.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()

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
        self.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        frontiers = List()
        frontiers.append(self.start)

        while (self.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging
            if rospy.is_shutdown():
                return False
            
            cell = self.popCellFromQueue()
            cells = self.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                if (self.hasCellBeenVisitedAlready(nextCell) == False):
                    if isFrontierCell(cell.coords[0],cell.coords[1]):
                        self.markCellAsVisitedAndRecordParent(nextCell, cell)
                        self.pushCellOntoQueue(nextCell)
                        self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                    else:
                        continue
                else:
                    self.resolveDuplicate(nextCell, cell)

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            frontiers.append(cell)

            # Draw the update if required
            self.drawCurrentState()

        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.drawCurrentState()

        return frontiers

        
    def chooseNewDestination(self):


#         print 'blackList:'
#         for coords in self.blackList:
#             print str(coords)

        candidateGood = False
        destination = None
        smallestD2 = float('inf')
        # This is inefficient - when improving, see if we can get a different loop
        #so that it doesn't check the entire map

        # If we got a good candidate, use it

        return candidateGood, destination

    def destinationReached(self, goal, goalReached):
        if goalReached is False:
#             print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
            
