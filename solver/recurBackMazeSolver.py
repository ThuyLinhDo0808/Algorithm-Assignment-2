# -------------------------------------------------------------------
# DON'T CHANGE THIS FILE.
# Recursive BackTracking Maze Solver.
#
# __author__ = 'Elham Naghizade'
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------------------------


from maze.util import Coordinates
from maze.maze import Maze

from typing import List

class RecurBackMazeSolver():


    def __init__(self):
        
        # self.m_solverPath: Set of cells that the solver visited.  This does include backtracking.
        self.m_solverPath: List[Coordinates] = list()
        # self.m_cellsExplored: Number of cells explored during the solving process.  Does not include backtracking.
        self.m_cellsExplored = 0 
        # self.m_entranceUsed: Entrance used to enter maze by the solver.
        self.m_entranceUsed = None
        # self.m_exitUsed: Exit found and used by maze solver as the exit.
        self.m_exitUsed = None

 
    def solveMaze(self, maze: Maze, entrance: Coordinates):

        visited = set()  # Set to track visited nodes
        stack   = [entrance]  # populate the stack with the entrance node
        exits   = maze.getExits()
        
        predecessors: dict[Coordinates, Coordinates] = {}
        self.m_cellsExplored += 1
        
        while stack:
            currCell = stack.pop()
            if currCell in exits:
                # Build the path from predecessors
                while currCell is not None:
                    self.m_solverPath.append(currCell)
                    currCell = predecessors.get(currCell, None)
                self.m_solverPath.reverse()
                self.m_entranceUsed = self.m_solverPath[0]
                self.m_exitUsed = self.m_solverPath[-1]
                return

            if currCell not in visited:
                visited.add(currCell)
                self.m_cellsExplored += 1

                neighbours = maze.neighbours(currCell)
                nonVisitedNeighs = [neigh for neigh in neighbours if neigh not in visited and not maze.hasWall(currCell, neigh)]

                for neigh in nonVisitedNeighs:
                    stack.append(neigh)
                    predecessors[neigh] = currCell