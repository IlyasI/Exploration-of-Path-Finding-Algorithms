from greedy_planner import GreedyPlanner
import math
import numpy as np

class AStarPlanner(GreedyPlanner):
    def __init__(self, title, occupancyGrid, heuristic='Euclidean', weight=1):
        '''Select from 'Euclidean','Zero','C','Octile' and 'Manhattan' for the heuristic '''
        GreedyPlanner.__init__(self, title, occupancyGrid)
        self.heuristic = heuristic
        self.weight = weight

    def pushCellOntoQueue(self, cell):
        if cell.parent == None:#if no parent, cell must be source
            self.Q.put((0, cell))#put at the beginning of the queue
        else:
            #gScore represents path cost to cell
            L = self.computeLStageAdditiveCost(cell.parent, cell)#cost to move from parent cell to current cell
            parent_gScore = cell.parent.pathCost#cost to go from goal to parent cell
            tentative_gScore = parent_gScore + L#cost to go from goal to current cell
            cell.pathCost = tentative_gScore
            fScore = cell.pathCost + self.heuristic_estimate(cell)
            self.Q.put((fScore,cell))

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
            #gScore represents path cost to cell
        L = self.computeLStageAdditiveCost(parentCell, cell)#cost to move from parent cell to current cell
        parent_gScore = parentCell.pathCost#cost to go from goal to parent cell
        tentative_gScore = parent_gScore + L#cost to go from goal to current cell
            
        if tentative_gScore < cell.pathCost:
            cell.pathCost = tentative_gScore
            cell.parent = parentCell
            for i, (_,c) in enumerate(self.Q.queue):
                fScore = cell.pathCost + self.heuristic_estimate(cell)
                if c.coords == cell.coords:
                    self.Q.put((fScore,cell))
                    break

    def heuristic_estimate(self,cell,heuristic=None,weight=None):
        cell_coords = cell.coords
        x1=cell_coords[0]
        y1=cell_coords[1]

        goal_coords = self.goal.coords
        x2=goal_coords[0]
        y2=goal_coords[1]

        if heuristic==None:
            heuristic=self.heuristic
        if weight==None:
            weight=self.weight
            
        if heuristic == 'Euclidean':
            return weight*math.hypot(abs(x2 - x1), abs(y2 - y1))
        if heuristic == 'Zero':
            return 0
        if heuristic == 'C':
            return weight*1
        if heuristic == 'Octile':
            return weight*(abs(x2 - x1) + abs(y2 - y1) + (np.sqrt(2) - 2)*min(x2 - x1, y2 - y1))
        if heuristic == 'Manhattan':
            return weight*(abs(x2 - x1) + abs(y2 - y1))