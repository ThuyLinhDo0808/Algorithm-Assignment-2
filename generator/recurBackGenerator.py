# -------------------------------------------------------------------
# DON'T CHANGE THIS FILE.
# Recursive backtracking maze generator.
# (similar to Assignment 1) The code is modified to find the  
# closest neighbour instead of a random neighbour
#
# __author__ = 'Jeffrey Chan' & 'Elham Naghizade'
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------------------------

from random import choice
from collections import deque

from maze.maze import Maze
from maze.util import Coordinates



class RecurBackMazeGenerator():
	"""
	Recursive backtracking maze generator.
	Overrides genrateMaze of parent class.
	"""

	def generateMaze(self, maze: Maze):

		# Select a random starting cell from the initialized maze cells
		
		startCoord: Coordinates = choice(maze.getCoords())
		print (startCoord)
		while startCoord.getWeight() == 0: # the random cell is a boundary cell
			startCoord = choice(maze.getCoords())


		# run recursive backtracking/DFS from starting cell
		stack : deque = deque()
		stack.append(startCoord)
		currCell : Coordinates = startCoord 
		visited : set[Coordinates] = set([startCoord])

		totalCells = maze.rowNum() * maze.colNum()
		

		while len(visited) < totalCells:
			# find all neighbours of current cell
			neighbours : list[Coordinates] = maze.neighbours(currCell)

			# filter to ones that haven't been visited and within boundary
			nonVisitedNeighs : list[Coordinates] = [neigh for neigh in neighbours if neigh not in visited and neigh.getRow() >= 0 and neigh.getRow() < maze.rowNum() and neigh.getCol() >= 0 and neigh.getCol() < maze.colNum()]
			
			# see if any unvisited neighbours
			if len(nonVisitedNeighs) > 0:
							
				# choose the neighbour with the least distance to the current cell
				neighDist = [(neigh, abs(currCell.getWeight() - neigh.getWeight())) for neigh in nonVisitedNeighs]	
				neigh = sorted(neighDist, key = lambda x:x[1])[0][0]

				# we move there and knock down wall
				maze.removeWall(currCell, neigh)

				# add to stack
				stack.append(neigh)

				# updated visited
				visited.add(neigh)

				# update currCell
				currCell = neigh
			else:
				# backtrack
				currCell = stack.pop()
		
			

		
