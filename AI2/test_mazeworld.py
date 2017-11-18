# Author: Ping-Jung Liu
# Date: September 26th 2017
# COSC 76 Assignment 2: Mazeworld
# Acknowledgement: Professor Devin Balkom for providing the general structure 

from MazeworldProblem import MazeworldProblem
from Maze import Maze

from astar_search import bfs_search
from astar_search import astar_search

# null heuristic, useful for testing astar search without heuristic (uniform cost search).
def null_heuristic(state):
    return 0

# Test problems

test_maze1 = Maze("maze1.maz")
print(test_maze1)
test_mp = MazeworldProblem(test_maze1, (32, 4))

# this should explore a lot of nodes; it's just uniform-cost search
result = astar_search(test_mp, null_heuristic)
print(result)

# this should do a bit better:
result = astar_search(test_mp, test_mp.manhattan_heuristic)
print(result)
# test_mp.animate_path(result.path)

result = astar_search(test_mp, test_mp.bfs_heuristic)
print(result)


test_maze2 = Maze("maze2.maz")
print(test_maze2)
test_mp = MazeworldProblem(test_maze2, (4, 2, 5, 1, 6, 0))

# this should explore a lot of nodes; it's just uniform-cost search
result = astar_search(test_mp, null_heuristic)
print(result)

# this should do a bit better:
result = astar_search(test_mp, test_mp.manhattan_heuristic)
print(result)
# test_mp.animate_path(result.path)

result = astar_search(test_mp, test_mp.bfs_heuristic)
print(result)


test_maze3 = Maze("maze3.maz")
print(test_maze3)
test_mp = MazeworldProblem(test_maze3, (0, 4, 1, 4, 2, 4))

# this should explore a lot of nodes; it's just uniform-cost search
result = astar_search(test_mp, null_heuristic)
print(result)

# this should do a bit better:
result = astar_search(test_mp, test_mp.manhattan_heuristic)
print(result)
# test_mp.animate_path(result.path)

result = astar_search(test_mp, test_mp.bfs_heuristic)
print(result)


test_maze5 = Maze("maze5.maz")
print(test_maze5)
test_mp = MazeworldProblem(test_maze5, (4, 0))

# this should explore a lot of nodes; it's just uniform-cost search
result = astar_search(test_mp, null_heuristic)
print(result)

# this should do a bit better:
result = astar_search(test_mp, test_mp.manhattan_heuristic)
print(result)
# test_mp.animate_path(result.path)

result = astar_search(test_mp, test_mp.bfs_heuristic)
print(result)


test_maze6 = Maze("maze6.maz")
print(test_maze6)
test_mp = MazeworldProblem(test_maze6, (1, 0, 3, 1, 2, 5))

# this should explore a lot of nodes; it's just uniform-cost search
result = astar_search(test_mp, null_heuristic)
print(result)

# this should do a bit better:
result = astar_search(test_mp, test_mp.manhattan_heuristic)
print(result)
# test_mp.animate_path(result.path)

result = astar_search(test_mp, test_mp.bfs_heuristic)
print(result)


