# -------------------------------------------------------------------
# DON'T CHANGE THIS FILE.
# This is used for Task B and Task C
# 
# __author__ = 'Elham Naghizade'
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------------------------

from typing import List

from maze.util import Coordinates
from maze.maze import Maze

from solver.taskBSolver import bruteForceSolver
from solver.taskCSolver import greedySolver


class AllPairsSolver():
    def __init__(self, strategy:str):
        self.all_solved = False
        self.entrance_exit_paths = {}
        
        if strategy == 'brute-force':
            self.m_solver = bruteForceSolver()
        elif strategy == 'greedy':
            self.m_solver = greedySolver()

    
    def solveMaze(self, maze: Maze, entrances: List[Coordinates], exits: List[Coordinates]):
        """
        Solve the maze to find all paths between each pair of entrance and exits using backtracking.
        """
        self.m_solver.solveMaze(maze,entrances, exits)
        self.all_solved = self.m_solver.all_solved

    def isSolved(self)->bool:
        """
        Use after solveMaze(maze), to check whether the maze is solved.
	    @return True if solved. Otherwise false.
        """
        return self.all_solved

    def cellsExplored(self)->int:
        """
        Use after solveMaze(maze), counting the number of cells explored in solving process.
	    @return The number of cells explored.
	    It is not required to be accurate and no marks are given (or lost) on it. 
        """
        return self.m_solver.cellsExplored


    def getSolverPath(self) -> dict:
        """
        Use after solveMaze(maze), 
	    @return the (ent,exit):path pairs where path is a list of Coordinates
        """
        if self.all_solved:
            return self.m_solver.entrance_exit_paths
        return self.entrance_exit_paths




        