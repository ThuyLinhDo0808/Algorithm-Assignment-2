# -------------------------------------------------------------------
# DON'T CHANGE THIS FILE.
# This is the entry point to run the program.
# Refer to usage() for exact format of input expected to the program.
#
# __author__ = 'Jeffrey Chan' & 'Elham Naghizade'
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------------------------


import sys
import time
import json
import random
from typing import List


from maze.util import Coordinates
from maze.maze import Maze

from generator.mazeGenerator import MazeGenerator
from solver.mazeSolver import MazeSolver

from solver.allPairsSolver import AllPairsSolver




# this checks if Visualizer has been imported properly.
# if not, likely missing some packages, e.g., matplotlib.
# in that case, regardless of visualisation flag, we should set the canVisualise flag to False which will not call the visuslisation part.
canVisualise = True
try:
	from maze.maze_viz import Visualizer
except:
	Visualizer = None
	canVisualise = False



def usage():
	"""
	Print help/usage message.
	"""

	# On Teaching servers, use 'python3'
	# On Windows, you may need to use 'python' instead of 'python3' to get this to work
	print('python3 mazeTester2.py', '<configuration file>')
	sys.exit(1)


#
# Main function, when the python script is executed, we execute this.
#
if __name__ == '__main__':
	# Fetch the command line arguments
	args = sys.argv

	if len(args) != 2:
		print('Incorrect number of arguments.')
		usage()


	# open configuration file		
	fileName: str = args[1]
	with open(fileName,"r") as configFile:
		# use json parser
		configDict = json.load(configFile)

		
		# Assign to variables storing various parameters.
		rowNum: int = configDict['rowNum']
		colNum: int = configDict['colNum']
		wtApproach: str = configDict['weight']
		# set of entrances
		entrances: List[List[int]] = configDict['entrances']
		# set of exits
		exits: List[List[int]] = configDict['exits']
		# generator approach to use 
		genApproach: str = configDict['generator']
		# solver approach to use 
		solverApproach: str = configDict['solver']
		# Optional: The index of which entrance to use (start at index 0) 
		solverEntIndex = None
		if 'solverEntranceIndex' in configDict.keys():
			solverEntIndex: int = configDict['solverEntranceIndex']
			multiPath  = False	
		else: 
			allPairStrategy = configDict['strategy']
			multiPath  = True	
		# whether to visualise the generated maze and solving solution or not
		bVisualise: bool = configDict['visualise']
		# Optional: Filename to store visualisation output
		outFilename : str = None
		if 'fileOutput' in configDict.keys():
			outFilename = configDict['fileOutput']
		# Optional: Seed to pass to random generator (used for validation)
		randSeed: int = None
		if 'randSeed' in configDict.keys():
			randSeed = configDict['randSeed']


		# initialise the random seed generator 
		if randSeed != None:
			random.seed(randSeed)


		# Initialise maze object
		
		maze: Maze = Maze(rowNum, colNum, wtApproach)
	

		# add the entraces and exits
		for [r,c] in entrances:
			maze.addEntrance(Coordinates(r, c))
		for [r,c] in exits:
			maze.addExit(Coordinates(r, c))

		

		# Generate maze
	
		generator = MazeGenerator(genApproach)
		# timer for generation
		startGenTime : float = time.perf_counter()
		generator.generateMaze(maze)
		
		
		# stop timer
		endGenTime: float = time.perf_counter()

		print(f'Generation took {endGenTime - startGenTime:0.4f} seconds')


	
		mazeEntrances: List[Coordinates]  = maze.getEntrances()
		mazeExits: List[Coordinates]  = maze.getExits()

		# # check if solver entrance index is within bounds
		if solverEntIndex != None and (solverEntIndex < 0 or solverEntIndex >= len(mazeEntrances)):
			print("Specified index of entrance that solver starts is out of bounds, {}".format(solverEntIndex))
			usage()

		if generator.isMazeGenerated():
			
			# time for solving
			startSolveTime : float = time.perf_counter()

			# Task B mode, where we specify the entrance
			if solverEntIndex is not None:
				
				# Solve maze
				solver = MazeSolver(solverApproach)
				solver.solveMaze(maze, mazeEntrances[solverEntIndex])
			# Task C and D mode, where we need to find non-overlapping paths between all pairs of entrances and exits
			else:
				if len(entrances) != len(exits):
					print("Specified index of entrance that solver starts is out of bounds, {}".format(solverEntIndex))
					usage()

				else:
					solver = AllPairsSolver(allPairStrategy) 
					solver.solveMaze(maze, mazeEntrances, mazeExits)
			
		 	# stop timer
			endSolveTime: float = time.perf_counter()

			print(f'Solving took {endSolveTime - startSolveTime:0.4f} seconds')
			# print('Solver used Entrance {entrance} and Exit {exit}.'.format(entrance=solver.getEntranceUsed(), exit=solver.getExitUsed()))
		else:
			print("Generator hasn't been implemented yet, hence solver wasn't called.")

		print ("getting to visualisation!", canVisualise)
		# # Display maze.
	
		if bVisualise and canVisualise and generator.isMazeGenerated():
			cellSize = 1
			visualiser = Visualizer(maze, solver, multiPath, cellSize) 
	
			if outFilename:
				visualiser.show_maze(outFilename)
			else:
				visualiser.show_maze()
					
				
