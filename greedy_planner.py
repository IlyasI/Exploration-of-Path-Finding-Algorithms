from cell_based_forward_search import CellBasedForwardSearch
from Queue import PriorityQueue
import math

# This class implements the FIFO - or breadth first search - planning
# algorithm. It works by using a double ended queue: cells are pushed
# onto the back of the queue, and are popped from the front of the
# queue.

class GreedyPlanner(CellBasedForwardSearch):

    # self implements a simple FIFO search algorithm
    
    def __init__(self, title, occupancyGrid):
        CellBasedForwardSearch.__init__(self, title, occupancyGrid)
        self.occupancyGrid = occupancyGrid
        self.Q = PriorityQueue()

    # Simply put on the end of the queue
    def pushCellOntoQueue(self, cell):
        print(cell.coords)
        estimate_to_goal = math.hypot((self.goal.coords[0] - cell.coords[0]) , (self.goal.coords[1] - cell.coords[1]))
        self.Q.put((estimate_to_goal,cell)) 

    # Check the queue size is zero
    def isQueueEmpty(self):
        return self.Q.empty()

    # Simply pull from the front of the list
    def popCellFromQueue(self):
        cell = self.Q.get()[1]
        return cell

    def resolveDuplicate(self, cell, parentCell):
        # Nothing to do in self case
        pass