from greedy_planner import GreedyPlanner
import math
import numpy as np

class DijkstrasPlanner(GreedyPlanner):

    def pushCellOntoQueue(self, cell):
        if cell.parent == None:
            self.Q.put((0, cell))
        else:
            L = self.computeLStageAdditiveCost(cell.parent, cell)
            tentative_pathCost = cell.parent.pathCost + L
            if tentative_pathCost < cell.pathCost:
                cell.pathCost = tentative_pathCost
                self.Q.put((cell.pathCost,cell))