from a_star_planner import AStarPlanner

class ThetaStarPlanner(AStarPlanner):
    def __init__(self, title, occupancyGrid, heuristic='Euclidean', weight=1):
        '''Select from 'Euclidean','Zero','C','Octile' and 'Manhattan' for the heuristic '''
        ThetaStarPlanner.__init__(self, title, occupancyGrid)

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

    def lineOfSight(self, current_cell, target_cell):
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
                    if self.getCell(x1+((signX-1)/2),y1+((signY-1)/2)).isOccupied:
                        return False
                    y1 += signY
                    f-=dx
                if (f!=0 and self.getCell(x1+((signX-1)/2),y1+((signY-1)/2)).isOccupied:
                    return False
                if (dy ==0 and self.getCell(x1+((signX-1)/2),y1).isOccupied and self.getCell(x1+((signX-1)/2),y1-1).isOccupied:
                    return False
                x1 += signX

        else:
            while y1 != y2:
                f+=dx
                if f>=dy:
                    if self.getCell(x1+((signX-1)/2),y1+((signY-1)/2)).isOccupied:
                        return False
                    x1 += signX
                    f-=dy
                if (f!=0 and self.getCell(x1+((signX-1)/2),y1+((signY-1)/2)).isOccupied:
                    return False
                if (dx==0 and self.getCell(x1,y1+((signY-1)/2)).isOccupied and self.getCell(x1-1,y1+((signY-1)/2)).isOccupied:
                    return False
                y1 += signY
        return True