# -*- coding: utf-8 -*-

from occupancy_grid import OccupancyGrid
from search_grid import SearchGrid
from planner_base import PlannerBase
from collections import deque
from cell import *
from planned_path import PlannedPath
from math import *
import rospy

class GeneralForwardSearchAlgorithm(PlannerBase):

    # This class implements the basic framework for LaValle's general
    # template for forward search. It includes a lot of methods for
    # managing the graphical output as well
    
    def __init__(self, title, occupancyGrid):
        PlannerBase.__init__(self, title, occupancyGrid)
         
        # Flag to store if the last plan was successful
        self.goalReached = None

    # These methods manage the queue of cells to be visied.
    def pushCellOntoQueue(self, cell):
        raise NotImplementedError()

    # Self method returns a boolean - true if the queue is empty,
    # false if it still has some cells on it.
    def isQueueEmpty(self):
        raise NotImplementedError()

    # Self method finds the first cell (at the head of the queue),
    # removes it from the queue, and returns it.
    def popCellFromQueue(self):
        raise NotImplementedError()

    # Self method determines if the goal has been reached.
    def hasGoalBeenReached(self, cell):
        raise NotImplementedError()

    # Self method gets the list of cells which could be visited next.
    def getNextSetOfCellsToBeVisited(self, cell):
        raise NotImplementedError()

    # Self method determines whether a cell has been visited already.
    def hasCellBeenVisitedAlready(self, cell):
        raise NotImplementedError()

    def markCellAsVisitedAndRecordParent(self, cell, parentCell):
        cell.label = CellLabel.ALIVE
        cell.parent = parentCell

    # Mark that a cell is dead. A dead cell is one in which all of its
    # immediate neighbours have been visited.
    def markCellAsDead(self, cell):
        cell.label = CellLabel.DEAD
    
    # Handle the case that a cell has been visited already.
    def resolveDuplicate(self, cell):
        raise NotImplementedError()
    
    # Compute the additive cost of performing a step from the parent to the
    # current cell. This calculation is carried out the same way no matter
    # what heuristics, etc. are used
    def computeLStageAdditiveCost(self, parentCell, cell):
        
        # If the parent is empty, this is the start of the path and the
        # cost is 0.
        if (parentCell is None):
            return 0
        
        # Cost is the Cartesian distance
        dX = cell.coords[0] - parentCell.coords[0]
        dY = cell.coords[1] - parentCell.coords[1]
        L = sqrt(dX * dX + dY * dY)
        
        return L
        
    # The main search routine. The routine searches for a path between a given
    # set of coordinates. These are then converted into start and destination
    # cells in the search grid and the search algorithm is then run.
    def search(self, startCoords, goalCoords):

        # Empty the queue. self is needed to make sure everything is reset
        while (self.isQueueEmpty() == False):
            self.popCellFromQueue()
        
        # Create the search grid from the occupancy grid and seed
        # unvisited and occupied cells.
        if (self.searchGrid is None):
            self.searchGrid = SearchGrid.fromOccupancyGrid(self.occupancyGrid)
        else:
            self.searchGrid.updateFromOccupancyGrid()

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        self.start = self.searchGrid.getCellFromCoords(startCoords)
        self.start.label = CellLabel.START
        self.start.pathCost = 0

        # Get the goal cell object and label it.
        self.goal = self.searchGrid.getCellFromCoords(goalCoords)
        self.goal.label = CellLabel.GOAL

        if rospy.is_shutdown():
            return False

        # Draw the initial state
        self.resetGraphics()

        # Insert the start on the queue to start the process going.
        self.markCellAsVisitedAndRecordParent(self.start, None)
        self.pushCellOntoQueue(self.start)

        # Reset the count
        self.numberOfCellsVisited = 0

        # Indicates if we reached the goal or not
        self.goalReached = False

        # Iterate until we have run out of live cells to try or we reached the goal
        while (self.isQueueEmpty() == False):

            # Check if ROS is shutting down; if so, abort. This stops the
            # planner from hanging
            if rospy.is_shutdown():
                return False
            
            cell = self.popCellFromQueue()
            if (self.hasGoalBeenReached(cell) == True):
                self.goalReached = True
                break
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
        
        print "numberOfCellsVisited = " + str(self.numberOfCellsVisited)
        
        if self.goalReached:
            print "Goal reached"
        else:
            print "Goal not reached"

        return self.goalReached

    # This method extracts a path from the pathEndCell to the start
    # cell. The path is a list actually sorted in the order:
    # cell(x_I), cell(x_1), ... , cell(x_K), cell(x_G). You can use
    # this method to try to find the path from any end cell. However,
    # depending upon the planner used, the results might not be
    # valid. In this case, the path will probably not terminate at the
    # start cell.
    def extractPathEndingAtCell(self, pathEndCell, colour):

        # Construct the path object and mark if the goal was reached
        path = PlannedPath()
        
        path.goalReached = self.goalReached
        
        # Initial condition - the goal cell
        path.waypoints.append(pathEndCell)
               
        # Start at the goal and find the parent. Find the cost associated with the parent
        prev_cell = pathEndCell
        first_cell_in_line_of_sight = pathEndCell
        cell = pathEndCell.parent
        path.travelCost = self.computeLStageAdditiveCost(cell, pathEndCell)
        
        # Iterate back through and extract each parent in turn and add
        # it to the path. To work out the travel length along the
        # path, you'll also have to add self at self stage.
        while (cell.parent is not None):
            if (self.lineOfSight(first_cell_in_line_of_sight, cell.parent)==False):
		path.waypoints.appendleft(prev_cell)
                path.waypoints.appendleft(cell)
		tentative_travelCost = self.computeLStageAdditiveCost(prev_cell, first_cell_in_line_of_sight.parent) + self.computeLStageAdditiveCost(cell, prev_cell)
                path.travelCost = path.travelCost + tentative_travelCost
                first_cell_in_line_of_sight = prev_cell
            prev_cell = cell
            cell = cell.parent
	path.waypoints.appendleft(cell)#append very first waypoint
	tentative_travelCost = self.computeLStageAdditiveCost(first_cell_in_line_of_sight.parent, cell)
	path.travelCost = path.travelCost + tentative_travelCost
            
        # Update the stats on the size of the path
        path.numberOfWaypoints = len(path.waypoints)

        # Note that if we failed to reach the goal, the above mechanism computes a path length of 0.
        # Therefore, if we didn't reach the goal, change it to infinity
        if path.goalReached is False:
            path.travelCost = float("inf")

        print "Path travel cost = " + str(path.travelCost)
        print "Path cardinality = " + str(path.numberOfWaypoints)
        
        # Draw the path if requested
        if (self.showGraphics == True):
            self.plannerDrawer.update()
            self.plannerDrawer.drawPathGraphicsWithCustomColour(path, colour)
            self.plannerDrawer.waitForKeyPress()
        
        # Return the path
        return path

    # Extract the path from a specified end cell to the start. This is not
    # necessarily the full path. Rather, it lets us illustrate parts of the
    # path.
    def extractPathEndingAtCoord(self, endCellCoord):
        endCell = self.searchGrid.getCellFromCoords(endCellCoord)
        self.extractPathEndingAtCell(endCell, 'red')

    # Extract the path between the start and goal.
    def extractPathToGoal(self):
        path = self.extractPathEndingAtCell(self.goal, 'yellow')
       
        return path
    
    def lineOfSight(self, current_cell, target_cell):
	if target_cell == None:
	    return False
        x1 = current_cell.coords[0]
        y1 = current_cell.coords[1]
        x2 = target_cell.coords[0]
        y2 = target_cell.coords[1]
        dy = y2 -y1
        dx = x2 - x1
        f = 0
        signY = 1
        signX = 1
        offsetX = 0
        offsetY = 0

        grid = self.occupancyGrid

        if dy < 0:
            dy =  dy*-1
            signY = -1
            offsetY = -1
        if dx < 0:
            dx = dx*-1
            signX = -1
            offsetX = -1
        if dx >= dy:
            while x1!=x2:
                f+=dy
                if f>=dx:
                    if grid.getCell(x1+((signX-1)/2),y1+((signY-1)/2))>0:
                        return False
                    y1 += signY
                    f-=dx
                if (f!=0 and (grid.getCell(x1+((signX-1)/2),y1+((signY-1)/2))>0)):
                    return False
                if (dy ==0 and (grid.getCell(x1+((signX-1)/2),y1)>0) and (grid.getCell(x1+((signX-1)/2),y1-1)>0)):
                    return False
                x1 += signX

        else:
            while y1 != y2:
                f+=dx
                if f>=dy:
                    if grid.getCell(x1+((signX-1)/2),y1+((signY-1)/2))>0:
                        return False
                    x1 += signX
                    f-=dy
                if (f!=0 and (grid.getCell(x1+((signX-1)/2),y1+((signY-1)/2))>0)):
                    return False
                if (dx==0 and (grid.getCell(x1,y1+((signY-1)/2))>0) and (grid.getCell(x1-1,y1+((signY-1)/2))>0)):
                    return False
                y1 += signY
        return True
            
