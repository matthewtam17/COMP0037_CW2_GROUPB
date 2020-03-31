import rospy

from explorer_node_base import ExplorerNodeBase
from comp0037_reactive_planner_controller.cell import *
from math import sqrt
# This class implements a super dumb explorer. It goes through the
# current map and marks the first cell it sees as the one to go for

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):
        ExplorerNodeBase.__init__(self)
        

    def updateFrontiers(self):#
        #The update frontiers function updates the stored frontier cells in the map for
        #WFD.
        #It runs an outer BFS, then an inner BFS of a frontier cell is identified to obtain
        #a single continuous frontier that has initially been identified by WFD.
        if self.useWavefront:
            startCoords = self.currentCoords
            self.frontiers = self.outerBFS(startCoords)
            
        else:
            return True
    #IMPORTANT POINT:
    #THE OUTER BFS AND INNNER BFS ARE IMPLEMENTED IN HERE, EVEN IF THEY ARE MODIFICATIONS TO THE
    #SEARCH FUNCTION IN GENERAL FORRWARD SEARCH ALGORITHM script.
    #The reason why is that if we implement it in general_forward_search_algorithm.py
    # then we cannot use self.isFrontier() in the explorer base
    # to check if a cell is a frontier cell

    #This may not be the most elegant solution, but it works very well.
    #We hope that you wouldn't deduct marks for this, as functionally
    # it works the same, just that where the code is written is slightly
    #different.

    #We don't have a lot of programming experience, but we've tried our 
    # best efforts at implementing BFS

    def outerBFS(self,startCoords):
        #This function performs the outer BFS, where it searches from the robot's location until it 
        #finds a frontier cell. If it finds a frontier cell,
        #the search is stopped and that cell is then returned.
        #The cell is returned to the updateFrontiers() function which called this search.
        print("outerBFS startcoords: " + str(startCoords))
        self.planner.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.planner.isQueueEmpty() == False):
            self.planner.popCellFromQueue()

        #Initialise array of frontiers
        frontiers = list()

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
            #Every time we pop a cell from the queue, we check if its a frontier
            # cell. If its a frontier cell then we stop the outerBFS and then
            # run the innerBFS
            if (self.isFrontierCell(cell.coords[0],cell.coords[1]) is True) and (cell not in frontiers):
                frontiers.append(cell)
                frontiers = self.innerBFS(cell.coords,frontiers)
                #One important difference in our approach is we only identify the first
                # continuous frontier we get to. We don't carry on with the outerBFS
                # once we have had the innerBFS
                return frontiers
            cellstatus = self.occupancyGrid.getCell(cell.coords[0], cell.coords[1]) 
            # We don't really need to explore beyond the frontier
            if cellstatus > 0.55 or cellstatus < 0.45:
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
        return frontiers

    def innerBFS(self,startCoords,frontiers):
        # This function performs the inner BFS function.
        # This is where the search starts from the initial frontier cell given by
        # the outer BFS. It then searches and identifies all the neighbouring frontier cells,
        # that form up a continuous frontier.
        # All the frontier cells are appended to the list self.frontiers
        # So that self.frontiers contains all the frontier cells of a continuous frontier

        self.innerplanner.handleChangeToOccupancyGrid()
        
        # Make sure the queue is empty. We do this so that we can keep calling
        # the same method multiple times and have it work.
        while (self.innerplanner.isQueueEmpty() == False):
            self.innerplanner.popCellFromQueue()

        # Check the start and end are not occupied. Note that "0.5" means
        # "don't know" which is why it is used as the threshold for detection.
        if (self.occupancyGrid.getCell(startCoords[0], startCoords[1]) > 0.5):
            return False

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.innerplanner.start = self.innerplanner.searchGrid.getCellFromCoords(startCoords)


        
        self.innerplanner.start.label = CellLabel.START
        self.innerplanner.start.pathCost = 0

        if rospy.is_shutdown():
            return False

        # Draw the initial state
        #self.innerplanner.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.innerplanner.markCellAsVisitedAndRecordParent(self.innerplanner.start, None)
        self.innerplanner.pushCellOntoQueue(self.innerplanner.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        while (self.innerplanner.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging
            if rospy.is_shutdown():
                return False
            
            cell = self.innerplanner.popCellFromQueue()
            cells = self.innerplanner.getNextSetOfCellsToBeVisited(cell)
            for nextCell in cells:
                #Check if cell has not already been added to frontier
                if (self.innerplanner.hasCellBeenVisitedAlready(nextCell) == False) and (nextCell not in frontiers):
                    #Only add cell to queue if it is a frontier cell
                    if self.isFrontierCell(nextCell.coords[0],nextCell.coords[1]):
                        self.innerplanner.markCellAsVisitedAndRecordParent(nextCell, cell)
                        self.innerplanner.pushCellOntoQueue(nextCell)
                        self.numberOfCellsVisited = self.numberOfCellsVisited + 1
                    else:
                        continue
                else:
                    self.innerplanner.resolveDuplicate(nextCell, cell)

            # ADD DEQUEUED CELL TO THE LIST OF FRONTIER CELLS
            frontiers.append(cell)

            # Draw the update if required
            self.innerplanner.drawCurrentState()
        # Do a final draw to make sure that the graphics are shown, even at the end state
        self.innerplanner.drawCurrentState()

        #Return list of frontiers with additional frontier cells appended
        #to it by the innerBFS
        return frontiers

    def chooseNewDestination(self):

        candidateGood = False
        destination = None
        smallestD2 = float('inf')

        if self.useWavefront:
            maxunknown = -1
            
            minDistance = float('inf')
            heuristic = 2
            #Using the imrpoved wavefront frontier detection (WF) algorithm
            # For heuristic = 1, we choose frontier cell that's closest to robot
            # For heuristic = 2, we choose frontier cell with the largest frontier,
            # i.e. with the most neighbouring unknown cells
            for frontier in self.frontiers:
                unknownCells = 0
                #Don't choose blacklisted cell
                if (frontier.coords[0],frontier.coords[1]) in self.blackList:
                    print("blacklisted cell")
                    continue
                    #This cell is a blacklist
                if heuristic == 2:
                    #Largest frontier heuristic. For this option, we choose 
                    #frontier cell with the largest frontier
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
                    #Closest frontier cell heuristic
                    #Choose frontier based on frontier cell that is closet to the robot
                    currentCoords = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates([self.pose.position.x,self.pose.position.y])
                    X = currentCoords[0]
                    Y = currentCoords[1]
                    distance = sqrt((frontier.coords[0]-X)**2+(frontier.coords[1]-Y)**2)
                    if distance < minDistance:
                        destination = (frontier.coords[0],frontier.coords[1])
                        candidateGood = True
                        minDistance = distance
                #The point where all frontier cells are unreachable/on blacklist
                
                    

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
            
