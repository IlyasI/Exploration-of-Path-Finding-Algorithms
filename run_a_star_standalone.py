#! /usr/bin/env python

# Import the needed types.
from comp313p_planner_controller.occupancy_grid import OccupancyGrid
from comp313p_planner_controller.a_star_planner import AStarPlanner

occupancyGrid = OccupancyGrid(50, 50, 0.5)

for x in xrange(2, 50):
    occupancyGrid.setCell(x, 8, 1)
for x in xrange(8, 48):
    occupancyGrid.setCell(20, x, 1)
for x in xrange(16,18):
    occupancyGrid.setCell(20, x, 0)
for x in xrange(19,50):
    occupancyGrid.setCell(40, x, 1)
    
start = (0, 0)
goal = (49, 49)

planner = AStarPlanner('AStarPlanner', occupancyGrid, heuristic='Octile')


# This causes the planner to slow down and pause for things like key entries
planner.setRunInteractively(True)

# This specifies the height of the window drawn showing the occupancy grid. Everything
# should scale automatically to properly preserve the aspect ratio
planner.setWindowHeightInPixels(400)
goalReached = planner.search(start, goal)
path = planner.extractPathToGoal()