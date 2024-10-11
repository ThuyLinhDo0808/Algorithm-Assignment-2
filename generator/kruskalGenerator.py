# -------------------------------------------------------------------
# PLEASE UPDATE THIS FILE.
# Kruskal's maze generator.
#
# __author__ = Linh Thuy Do
# __copyright__ = 'Copyright 2024, RMIT University'
# -------------------------------------------------------------------


from maze.maze import Maze
from maze.util import Coordinates
from typing import List

class DisjointSet:
    """
    Union-Find (Disjoint-set) data structure to manage connected components.
    """
    def __init__(self, size: int):
        self.parent = list(range(size))
        self.rank = [1] * size
    
    def find(self, node: int) -> int:
        if self.parent[node] != node:
            self.parent[node] = self.find(self.parent[node])  # Path compression
        return self.parent[node]

    def union(self, node1: int, node2: int) -> None:
        root1 = self.find(node1)
        root2 = self.find(node2)

        if root1 != root2:
            if self.rank[root1] > self.rank[root2]:
                self.parent[root2] = root1
            elif self.rank[root1] < self.rank[root2]:
                self.parent[root1] = root2
            else:
                self.parent[root2] = root1
                self.rank[root1] += 1


class KruskalMazeGenerator:
    """
    Kruskal's algorithm maze generator using the provided Maze and Coordinates classes.
    Does not remove boundary walls.
    """
    
    def generateMaze(self, maze: Maze) -> None:
        # Get all coordinates (cells) from the maze
        coords = maze.getCoords()
        num_cells = len(coords)

        # Initialize DisjointSet with the number of cells
        disjoint_set = DisjointSet(num_cells)

        # Get all non-boundary edges (walls) from the maze
        edges = [edge for edge in maze.getEdges() 
                 if self.isValidCell(edge[0], maze) and self.isValidCell(edge[1], maze)]

        # Sort edges by their weights (ascending order)
        edges.sort(key=lambda x: maze.edgeWeight(x[0], x[1]))

        # Process each edge
        for edge in edges:
            cell1, cell2 = edge[:2]
            cell1_index = coords.index(cell1)
            cell2_index = coords.index(cell2)

            # If the two cells are not part of the same set, remove the wall
            if disjoint_set.find(cell1_index) != disjoint_set.find(cell2_index):
                maze.removeWall(cell1, cell2)
                disjoint_set.union(cell1_index, cell2_index)

    def isValidCell(self, cell: Coordinates, maze: Maze) -> bool:
        """
        Check if the cell is within the valid boundaries of the maze (not a boundary cell).
        A valid cell has row and column indices within the bounds [0, rowNum-1] and [0, colNum-1].

        @param cell: The coordinates of the cell to check.
        @param maze: The maze object to get the dimensions.
        @return: True if the cell is valid (within bounds), False otherwise.
        """
        return 0 <= cell.getRow() < maze.rowNum() and 0 <= cell.getCol() < maze.colNum()


# from maze.maze import Maze
# from maze.util import Coordinates
# from typing import List


# class DisjointSet:
#     """
#     Union-Find (Disjoint-set) data structure to manage connected components.
#     """
#     def __init__(self, size):
#         self.parent = list(range(size))
#         self.rank = [1] * size
    
#     def find(self, node):
#         if self.parent[node] != node:
#             self.parent[node] = self.find(self.parent[node])  # Path compression
#         return self.parent[node]

#     def union(self, node1, node2):
#         root1 = self.find(node1)
#         root2 = self.find(node2)

#         if root1 != root2:
#             if self.rank[root1] > self.rank[root2]:
#                 self.parent[root2] = root1
#             elif self.rank[root1] < self.rank[root2]:
#                 self.parent[root1] = root2
#             else:
#                 self.parent[root2] = root1
#                 self.rank[root1] += 1


# class KruskalMazeGenerator:
#     """
#     Kruskal's algorithm maze generator using the provided Maze and Coordinates classes.
#     Does not remove boundary walls.
#     """
    
#     def generateMaze(self, maze: Maze):
#         # Get all coordinates (cells) from the maze
#         coords = maze.getCoords()
#         num_cells = len(coords)

#         # Initialize DisjointSet with the number of cells
#         disjoint_set = DisjointSet(num_cells)

#         # Get all non-boundary edges (walls) from the maze
#         edges = [edge for edge in maze.getEdges()
#                  if self.isValidCell(edge[0], maze) and self.isValidCell(edge[1], maze)]

#         # Sort edges by their weights (ascending order)
#         edges.sort(key=lambda x: maze.edgeWeight(x[0], x[1]))

#         # Process each edge
#         for edge in edges:
#             cell1, cell2 = edge[:2]
#             cell1_index = coords.index(cell1)
#             cell2_index = coords.index(cell2)

#             # If the two cells are not part of the same set, remove the wall
#             if disjoint_set.find(cell1_index) != disjoint_set.find(cell2_index):
#                 maze.removeWall(cell1, cell2)
#                 disjoint_set.union(cell1_index, cell2_index)

#         # The maze is now generated using Kruskal's algorithm

#     def isValidCell(self, cell: Coordinates, maze: Maze) -> bool:
#         """
#         Check if the cell is within the valid boundaries of the maze (not a boundary cell).
#         A valid cell has row and column indices within the bounds [0, rowNum-1] and [0, colNum-1].

#         @param cell: The coordinates of the cell to check.
#         @param maze: The maze object to get the dimensions.
#         @return: True if the cell is valid (within bounds), False otherwise.
#         """
#         return 0 <= cell.getRow() < maze.rowNum() and 0 <= cell.getCol() < maze.colNum()
