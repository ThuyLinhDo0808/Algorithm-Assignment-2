# -------------------------------------------------------------------
# PLEASE UPDATE THIS FILE.
# Dijkstra's maze solver.
#
# __author__ =  Linh Thuy Do
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------------------------

from maze.util import Coordinates
from maze.maze import Maze
from typing import List, Dict
import heapq
from collections import defaultdict
import itertools

class DijkstraSolver:
    
    def __init__(self):
        self.m_solverPath: List[Coordinates] = []
        self.m_cellsExplored = 0
        self.m_entranceUsed = None
        self.m_exitUsed = None
        self.counter = itertools.count()  # A counter to ensure uniqueness
    
    def solveMaze(self, maze: Maze, entrance: Coordinates):
        pq = []
        distances: Dict[Coordinates, float] = defaultdict(lambda: float('inf'))
        predecessors: Dict[Coordinates, Coordinates] = {}
        
        distances[entrance] = 0
        heapq.heappush(pq, (0, next(self.counter), entrance))
        self.m_cellsExplored = 0

        # Get valid edges
        edges = [edge for edge in maze.getEdges() 
                 if self.isValidCell(edge[0], maze) and self.isValidCell(edge[1], maze)]

        while pq:
            curr_distance, _, curr_cell = heapq.heappop(pq)

            if curr_cell in maze.getExits():
                self.m_exitUsed = curr_cell
                self.m_entranceUsed = entrance
                break
            
            # Explore neighbors
            for neighbor in maze.neighbours(curr_cell):
                if not maze.hasWall(curr_cell, neighbor):
                    new_distance = curr_distance + maze.edgeWeight(curr_cell, neighbor)
                    
                    if new_distance < distances[neighbor]:
                        distances[neighbor] = new_distance
                        predecessors[neighbor] = curr_cell
                        heapq.heappush(pq, (new_distance, next(self.counter), neighbor))

            self.m_cellsExplored += 1

        self._reconstructPath(predecessors, entrance, self.m_exitUsed)

    def _reconstructPath(self, predecessors: Dict[Coordinates, Coordinates], start: Coordinates, end: Coordinates):
        path = []
        curr_cell = end
        while curr_cell is not None:
            path.append(curr_cell)
            curr_cell = predecessors.get(curr_cell)
        self.m_solverPath = path[::-1]  # Reverse path to get entrance to exit
        self.m_entranceUsed = start
        self.m_exitUsed = end

    def isValidCell(self, cell: Coordinates, maze: Maze) -> bool:
        return 0 <= cell.getRow() < maze.rowNum() and 0 <= cell.getCol() < maze.colNum()