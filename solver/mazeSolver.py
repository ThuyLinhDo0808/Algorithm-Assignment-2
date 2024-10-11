# -------------------------------------------------------------------
# DON'T CHANGE THIS FILE.
# Abstract class for a maze solver.  Provides common variables and method interface for maze solvers.
#
# __author__ = 'Jeffrey Chan' & 'Elham Naghizade'
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------------------------


from maze.maze import Maze
from maze.util import Coordinates
from typing import List

from solver.recurBackMazeSolver import RecurBackMazeSolver
from solver.dijkstraSolver import DijkstraSolver



class MazeSolver:

    def __init__(self, solverName:str):
        
        # self.m_solved: true if the solver has found the exit (maze "solved")
        self.m_solved = False
        # self.solverName: solver to solve the maze
        if solverName == 'recur':
            self.m_solver = RecurBackMazeSolver()
        elif solverName == 'dijkstra':
            self.m_solver = DijkstraSolver()


    def solveMaze(self, maze: Maze, entrance: Coordinates):
        """
        Solves the given maze starting from the entrance using the solver object.
        Once the solver completes the solution, the maze is marked as solved.
        @param maze: The maze to be solved.
        @param entrance: The entrance coordinates where the solving process begins.
        """

        self.m_solver.solveMaze(maze, entrance)
        self.m_solved = True
        

    def isSolved(self)->bool:
        """
        Use after solveMaze(maze), to check whether the maze is solved.
	    @return True if solved. Otherwise false.
        """
        return self.m_solved


    def cellsExplored(self)->int:
        """
        Use after solveMaze(maze), counting the number of cells explored in solving process.
	    @return The number of cells explored.
	    It is not required to be accurate and no marks are given (or lost) on it. 
        """
        return self.m_solver.cellsExplored


    
    def getEntranceUsed(self)->Coordinates:
        """
        @return Return the entrance used in the solution.  Should only be called after a solution is found.
        """
        return self.m_solver.m_entranceUsed



    def getExitUsed(self)->Coordinates:
        """
        @return Return the exit used in the solution.  Should only be called after a solution is found.
        """
        return self.m_solver.m_exitUsed 

    def getSolverPath(self)->Coordinates:
        """
        @return Return the exit used in the solution.  Should only be called after a solution is found.
        """
        return self.m_solver.m_solverPath    
	
