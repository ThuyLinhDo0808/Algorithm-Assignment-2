# -------------------------------------------------
# DON'T CHANGE THIS FILE.
# Visualiser, original code from https://github.com/jostbr/pymaze writteb by Jostein Brændshøi
# Subsequentially modified by Jeffrey Chan.
#
# __author__ = 'Jostein Brændshøi, Jeffrey Chan', 'Elham Naghizade'
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------

# MIT License

# Copyright (c) 2021 Jostein Brændshøi

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.



try:
    import matplotlib.pyplot as plt
except:
    plt = None
    

from maze.maze import Maze
from maze.util import Coordinates

from solver.mazeSolver import MazeSolver


class Visualizer(object):
    """Class that handles all aspects of visualization.


    Attributes:
        maze: The maze that will be visualized
        solver: Contains the info of the solved path
        multiPath: Related to Task C & D where non-overlapping paths 
        between pairs of (ent, exit) are found
        cell_size (int): How large the cells will be in the plots
        height (int): The height of the maze
        width (int): The width of the maze
        ax: The axes for the plot
    """

    def __init__(self, maze :Maze, solver: MazeSolver, multiPath, cellSize):
        self.m_maze     = maze
        self.m_solver   = solver
        self.multiPaths = multiPath 
        self.m_cellSize = cellSize
        self.m_height   = (maze.rowNum()+2) * cellSize
        self.m_width    = (maze.colNum()+2) * cellSize
        self.m_ax       = None


    def show_maze(self, outFilename: str = None):
        """Displays a plot of the maze without the solution path"""

        # create the plot figure and style the axes
        fig = self.configure_plot()

        # plot the walls on the figure
        self.plot_walls()

        # plot the entrances and exits on the figure
        self.plotEntExit()

        # plot the solver path
        if self.m_solver != None:
            self.plotSolverPath()

        # display the plot to the user
        if outFilename == None:
            plt.show()
        else:
            # save image
            plt.savefig(outFilename)



    def plot_walls(self):
        """ 
        Plots the walls of a maze. This is used when generating the maze image.
        """
        for r in range(0, self.m_maze.rowNum()):
            for c in range(0, self.m_maze.colNum()):
                # top
                if self.m_maze.hasWall(Coordinates(r-1,c), Coordinates(r,c)):
                    self.m_ax.plot([(c+1)*self.m_cellSize, (c+1+1)*self.m_cellSize],
                                   [(r+1)*self.m_cellSize, (r+1)*self.m_cellSize], color="k")    
                # left
                if self.m_maze.hasWall(Coordinates(r,c-1), Coordinates(r,c)):
                    self.m_ax.plot([(c+1)*self.m_cellSize, (c+1)*self.m_cellSize],
                                   [(r+1)*self.m_cellSize, (r+1+1)*self.m_cellSize], color="k")  


        # do bottom boundary 
        for c in range(0, self.m_maze.colNum()):
            # top
            if self.m_maze.hasWall(Coordinates(self.m_maze.rowNum()-1,c), Coordinates(self.m_maze.rowNum(),c)):
                self.m_ax.plot([(c+1)*self.m_cellSize, (c+1+1)*self.m_cellSize],
                                [(self.m_maze.rowNum()+1)*self.m_cellSize, (self.m_maze.rowNum()+1)*self.m_cellSize], color="k")    

        # do right boundary 
        for r in range(0, self.m_maze.rowNum()):
            # left
            if self.m_maze.hasWall(Coordinates(r,self.m_maze.colNum()-1), Coordinates(r,self.m_maze.colNum())):
                self.m_ax.plot([(self.m_maze.colNum()+1)*self.m_cellSize, (self.m_maze.colNum()+1)*self.m_cellSize],
                                [(r+1)*self.m_cellSize, (r+1+1)*self.m_cellSize], color="k")  


    def plotEntExit(self):
        """
        Plots the entrances and exits in the displayed maze.
        """

        for ent in self.m_maze.getEntrances():
            # check direction of arrow
            # upwards arrow
            if ent.getRow() == -1:
                self.m_ax.arrow((ent.getCol()+1.5)*self.m_cellSize, (ent.getRow()+1)*self.m_cellSize, 0, self.m_cellSize*0.6, head_width=0.1)
            # downwards arrow
            elif ent.getRow() == self.m_maze.rowNum():
                self.m_ax.arrow((ent.getCol()+1.5)*self.m_cellSize, (ent.getRow()+2)*self.m_cellSize, 0, -self.m_cellSize*0.6, head_width=0.1)
            # rightward arrow
            elif ent.getCol() == -1:
                self.m_ax.arrow((ent.getCol()+1)*self.m_cellSize, (ent.getRow()+1.5)*self.m_cellSize, self.m_cellSize*0.6, 0, head_width=0.1)
            # leftward arrow
            elif ent.getCol() == self.m_maze.colNum():
                self.m_ax.arrow((ent.getCol()+2)*self.m_cellSize, (ent.getRow()+1.5)*self.m_cellSize, -self.m_cellSize*0.6, 0, head_width=0.1)

        for ext in self.m_maze.getExits():
            # downwards arrow
            if ext.getRow() == -1:
                self.m_ax.arrow((ext.getCol()+1.5)*self.m_cellSize, (ext.getRow()+1.8)*self.m_cellSize, 0, -self.m_cellSize*0.6, head_width=0.1)
            # upwards arrow
            elif ext.getRow() == self.m_maze.rowNum():
                self.m_ax.arrow((ext.getCol()+1.5)*self.m_cellSize, (ext.getRow()+1.2)*self.m_cellSize, 0, self.m_cellSize*0.6, head_width=0.1)
            # leftward arrow
            elif ext.getCol() == -1:
                self.m_ax.arrow((ext.getCol())*self.m_cellSize, (ext.getRow()+1.5)*self.m_cellSize, -self.m_cellSize*0.6, 0, head_width=0.1)
            # leftward arrow
            elif ext.getCol() == self.m_maze.colNum():
                self.m_ax.arrow((ext.getCol()+1.2)*self.m_cellSize, (ext.getRow()+1.5)*self.m_cellSize, self.m_cellSize*0.6, 0, head_width=0.1)

    def plotSolverPath(self):
        """
        Draw the path that the solver used to solve the maze.  They are displayed as a series of circles.
        """
        if not self.multiPaths:
            # retrieved the stored solver path
            solverPath = self.m_solver.getSolverPath()
            solverPath = [solverPath] 
        else:
            solverPath = self.m_solver.getSolverPath().values()

        # if no path, then just return
        if len(solverPath) == 0:
                return
        
        base_cmap = plt.get_cmap('tab10')
        
        for idx, path in enumerate(solverPath):
            base_color = base_cmap(idx % 10)[:3]  # filtering out the alpha value
           
            for circle_num in range(len(path)):
                # Calculate gradient factor based on progress along the path
                gradient_factor = circle_num / (len(path) - 1)  # Normalized gradient value between 0 and 1
                
                color = tuple([gradient_factor * base + (1 - gradient_factor) * 0.8  for base in base_color])
                

                # Draw circles with distinct base colors and gradients along the path
                self.m_ax.add_patch(plt.Circle(((path[circle_num].getCol() + 1.5) * self.m_cellSize,
                                        (path[circle_num].getRow() + 1.5) * self.m_cellSize), 
                                        0.2 * self.m_cellSize, fc=color, alpha=0.5))

    
    def configure_plot(self):
        """Sets the initial properties of the maze plot. Also creates the plot and axes"""

        # Create the plot figure
        fig = plt.figure(figsize = (7, 7*self.m_maze.rowNum() / self.m_maze.colNum()))

        # Create the axes
        self.m_ax = plt.axes()

        # Set an equal aspect ratio
        self.m_ax.set_aspect("equal")

        # Remove the axes from the figure
        self.m_ax.axes.get_xaxis().set_visible(False)
        self.m_ax.axes.get_yaxis().set_visible(False)


        return fig

