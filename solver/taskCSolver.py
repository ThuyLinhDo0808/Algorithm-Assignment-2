# -------------------------------------------------------------------
# PLEASE UPDATE THIS FILE.
# Greedy maze solver for all entrance, exit pairs
#
# __author__ = <Your Name>
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------------------------


from maze.util import Coordinates
from maze.maze import Maze
from typing import List, Dict

class AStarSolver:

    def __init__(self):
        self.m_solverPath: List[Coordinates] = []  # Stores the final path
        self.m_cellsExplored = 0                    # Counts the explored cells
        self.m_entranceUsed = None                   # Entrance used
        self.m_exitUsed = None                       # Exit used
        self.counter = itertools.count()             # Tie-breaking counter for heap uniqueness

    def heuristic(self, cell: Coordinates, goal: Coordinates) -> float:
        """
        Heuristic function for A* (Manhattan distance in this case).
        This can switch to Euclidean distance if diagonal movement is allowed.
        """
        return abs(cell.getRow() - goal.getRow()) + abs(cell.getCol() - goal.getCol())

    def solveMaze(self, maze: Maze, entrance: Coordinates, exit: Coordinates, blocked_cells: set):
        """
        Solve the maze using the A* algorithm from the entrance to the exit.
        """
        pq = []  # Priority queue for the A* algorithm
        distances: Dict[Coordinates, float] = defaultdict(lambda: float('inf'))
        predecessors: Dict[Coordinates, Coordinates] = {}

        # Initialize entrance
        distances[entrance] = 0
        heapq.heappush(pq, (0 + self.heuristic(entrance, exit), next(self.counter), entrance))  # (f(n), counter, cell)
        self.m_cellsExplored = 0

        while pq:
            # Get the node with the smallest f(n)
            curr_f, _, curr_cell = heapq.heappop(pq)

            # Check if we've reached the goal
            if curr_cell == exit:
                self.m_exitUsed = curr_cell
                self.m_entranceUsed = entrance
                break

            # Explore neighbors
            for neighbor in maze.neighbours(curr_cell):
                if neighbor not in blocked_cells and not maze.hasWall(curr_cell, neighbor):
                    new_g = distances[curr_cell] + abs(curr_cell.getWeight() - neighbor.getWeight())  # g(n)

                    if new_g < distances[neighbor]:
                        distances[neighbor] = new_g
                        predecessors[neighbor] = curr_cell
                        f_n = new_g + self.heuristic(neighbor, exit)  # f(n) = g(n) + h(n)
                        heapq.heappush(pq, (f_n, next(self.counter), neighbor))

            self.m_cellsExplored += 1

        # Reconstruct the path
        self._reconstructPath(predecessors, entrance, self.m_exitUsed)

    def _reconstructPath(self, predecessors: Dict[Coordinates, Coordinates], start: Coordinates, end: Coordinates):
        """
        Reconstruct the path from start to end using the predecessors dictionary.
        """
        path = []
        curr_cell = end
        while curr_cell is not None:
            path.append(curr_cell)
            curr_cell = predecessors.get(curr_cell)
        self.m_solverPath = path[::-1]  # Reverse the path to go from start to goal


class greedySolver:

    def __init__(self):
        self.entrance_exit_paths = {}  # Store the path for each entrance-exit pair
        self.all_solved = False          # Flag to indicate if a valid solution was found
        self.blocked_cells = set()       # Track the cells used in paths to prevent overlapping
        self.cellsExplored = 0           # Track the number of cells explored during solving

    def solveMaze(self, maze: Maze, entrances: List[Coordinates], exits: List[Coordinates]) -> bool:
        """
        Solve the maze using a greedy approach to compute non-overlapping paths 
        between each entrance-exit pair by using AStarSolver.
        """
        # Ensure that the number of entrances equals the number of exits
        if len(entrances) != len(exits):
            return False
        
        for entrance, exit in zip(entrances, exits):
            # Use AStarSolver to find the path between entrance and exit
            path, total_distance = self._findPath(maze, entrance, exit)

            if path:
                print(f"Solution found with total distance: {total_distance} \nFor pair ({entrance.getRow()},{entrance.getCol()}) and ({exit.getRow()},{exit.getCol()})")

                # Mark the path cells as blocked and store the path
                self._markPathBlocked(path)
                self.entrance_exit_paths[(entrance, exit)] = path
            else:
                # If no valid path is found, return False
                print(f"No valid non-overlapping paths found for.\nFor ({entrance.getRow()},{entrance.getCol()}) and ({exit.getRow()},{exit.getCol()})")
                self.all_solved = False
                return False
            
        self.all_solved = True
        return True

    def _findPath(self, maze: Maze, entrance: Coordinates, exit: Coordinates):
        # Initialize the AStarSolver
        astar_solver = AStarSolver()
        astar_solver.solveMaze(maze, entrance, exit, self.blocked_cells)
        
        # Get the path from the AStarSolver
        path = astar_solver.m_solverPath

        # Check if the path overlaps with blocked cells
        if any(cell in self.blocked_cells for cell in path):
            return None, 0  # Invalid path (overlapping)

        # Calculate the total distance for the valid path
        total_distance = sum(abs(path[i].getWeight() - path[i + 1].getWeight()) for i in range(len(path) - 1))
        
        # Update the number of cells explored
        self.cellsExplored += astar_solver.m_cellsExplored
        
        return path, total_distance

    def _markPathBlocked(self, path: List[Coordinates]):
        """
        Mark all cells in the path as blocked to prevent overlap.
        """
        for cell in path:
            self.blocked_cells.add(cell)

    def getSolverPath(self) -> Dict[tuple, List[Coordinates]]:
        """
        Retrieve the entrance-exit paths.
        Return the dictionary with keys as (entrance, exit) tuples and values as the paths.
        """
        return self.entrance_exit_paths