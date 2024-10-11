# -------------------------------------------------------------------
# DON'T CHANGE THIS FILE.
# Recursive BackTracking Maze Solver.
#
# __author__ = 'Jeffrey Chan' & 'Elham Naghizade'
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------------------------

from collections import deque

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

        # select starting cell
        startCoord: Coordinates = entrance

		# run recursive backtracking/DFS from starting cell
        stack : deque = deque()
        stack.append(startCoord)
        currCell : Coordinates = startCoord 
        visited : set[Coordinates] = set([startCoord])
        predecessors: dict[Coordinates, Coordinates] = {}
        self.m_cellsExplored += 1

        while currCell not in maze.getExits():

			# find all neighbours of current cell
            neighbours : list[Coordinates] = maze.neighbours(currCell)

			# filter to ones that haven't been visited and within boundary and doesn't have a wall between them	
            nonVisitedNeighs : list[Coordinates] = [neigh for neigh in neighbours if neigh not in visited and\
                                                    not maze.hasWall(currCell, neigh)]

            # see if any unvisited neighbours
            if len(nonVisitedNeighs) > 0:

                neighDist = [(neigh, abs(currCell.getWeight() - neigh.getWeight())) for neigh in nonVisitedNeighs]
                neigh = sorted(neighDist, key = lambda x:x[1])[0][0]

				# add to stack
                stack.append(neigh)

				# updated visited
                visited.add(neigh)
                predecessors[neigh] = currCell
                self.m_cellsExplored += 1

				# update currCell
                currCell = neigh
            else:
                currCell = stack.pop()
  
        # ensure we are currently at the exit
        if currCell in maze.getExits():
            
            while currCell is not None:
                self.m_solverPath.append(currCell)
                currCell = predecessors.get(currCell, None)  
           
            # We have moved backwards through predecessors and need to reverse the path
            self.m_solverPath.reverse()
            self.m_entranceUsed = self.m_solverPath[0]
            self.m_exitUsed = self.m_solverPath[1]





