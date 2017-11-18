# Author: Ping-Jung Liu
# Date: September 26th 2017
# COSC 76 Assignment 2: Mazeworld
# Acknowledgement: Professor Devin Balkom for providing the general structure 
from SensorlessProblem import SensorlessProblem
from Maze import Maze

from astar_search import bfs_search
from astar_search import astar_search

# this function can test whether the path found by the algorithm works for a given start location and the goal
def test_start(maze, start_loc, path, goal):
	x = start_loc[0]
	y = start_loc[1]

	for i in range(0, len(path)):
		x = x + path[i][0]
		y = y + path[i][1]
		if not maze.is_floor(x, y):
			x = x - path[i][0]
			y = y - path[i][1]
	
	return (x, y) == goal

# this function test the solution path for all possible start location
def test_all(maze, problem, path, goal):
	all_loc = problem.start_state[0]
	flag = "ALL PASS"
	for loc in all_loc:
		if not test_start(maze, loc, path, goal):
			print(loc)
			flag = "FAIL"
	print(flag)

# obtain the actions from result
def get_path(result):
	path = []
	for i in range(1, len(result.path)):
		path.append(result.path[i][1])
	return path

# obtain the solution path from a start location
def create_path(maze, start_loc, path):

	p = [start_loc]
	x = start_loc[0]
	y = start_loc[1]
	#print((x, y))
	for i in range(1, len(path) + 1):

		x = x + path[i - 1][0]
		y = y + path[i - 1][1]
		

		if not maze.is_floor(x, y):
			x = x - path[i - 1][0]
			y = y - path[i - 1][1]
	
		p.append((x, y))
	return p
#############################################################################
# I recommend ignoring everything up there. They are just helper functions to make live easier.

goal = (2, 0)
test_maze = Maze("maze4.maz")
print(test_maze)
test_mp = SensorlessProblem(test_maze, goal)
result = astar_search(test_mp, test_mp.num_heuristic)
result.path = get_path(result)
print(result)
test_all(test_maze, test_mp, result.path, goal)
#test_mp.animate_path(create_path(test_maze, (2, 0), result.path))
result = astar_search(test_mp, test_mp.spam_heuristic)
result.path = get_path(result)
print(result)

test_all(test_maze, test_mp, result.path, goal)



goal = (0, 4)
test_maze = Maze("maze3.maz")
print(test_maze)
test_mp = SensorlessProblem(test_maze, goal)
result = astar_search(test_mp, test_mp.num_heuristic)
result.path = get_path(result)
print(result)
test_all(test_maze, test_mp, result.path, goal)
#test_mp.animate_path(create_path(test_maze, (2, 0), result.path))
result = astar_search(test_mp, test_mp.spam_heuristic)
result.path = get_path(result)
print(result)

test_all(test_maze, test_mp, result.path, goal)


goal = (4, 0)
test_maze = Maze("maze5.maz")
print(test_maze)
test_mp = SensorlessProblem(test_maze, goal)
result = astar_search(test_mp, test_mp.num_heuristic)
result.path = get_path(result)
print(result)
test_all(test_maze, test_mp, result.path, goal)
#test_mp.animate_path(create_path(test_maze, (2, 0), result.path))
result = astar_search(test_mp, test_mp.spam_heuristic)
result.path = get_path(result)
print(result)

test_all(test_maze, test_mp, result.path, goal)






