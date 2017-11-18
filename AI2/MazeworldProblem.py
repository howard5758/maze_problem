# Author: Ping-Jung Liu
# Date: September 26th 2017
# COSC 76 Assignment 2: Mazeworld
# Acknowledgement: Professor Devin Balkom for providing the general structure 
from Maze import Maze
from time import sleep

from astar_search import bfs_search
from astar_search import astar_search

class MazeworldProblem:

    ## you write the constructor, and whatever methods your astar function needs
    def __init__(self, maze, goal_locations):
        self.maze = maze
        self.goal_locations = goal_locations
        self.start_state = maze.start_state

    def __str__(self):
        string =  "Mazeworld problem: "
        return string

    # the state[0] represents which robot is moving next
    # the rest of the state represent the locations of all robots
    def goal_test(self, state):
        return state[1 : len(state)] == self.goal_locations

    def get_successors(self, state):

        # actions available for robots
        action = [[0, 0], [1, 0], [0, 1], [-1, 0], [0, -1]]
        maze = self.maze
        maze.robotloc = list(state[1 : len(state)])

        # whos turn
        turn = state[0]

        # number of robot in the maze
        robotNum = len(maze.robotloc) / 2

        # this list, tuple operation will be perform several times in the project, to prevent altering elements in original list
        # change the turn to the next robot, or the first robot if current is the last robot
        state = list(state)
        state[0] = int((state[0] + 1) % robotNum)
        state = tuple(state)

        successors = []

        # loop through all available actions
        for i in range(0, len(action)):
 
            # if new positions are legal, append the new state
            if maze.is_floor(state[turn * 2 + 1] + action[i][0], state[turn * 2 + 2] + action[i][1]) and (not maze.has_robot(state[turn * 2 + 1] + action[i][0], state[turn * 2 + 2] + action[i][1])):
                new_state = list(state)
                new_state[turn * 2 + 1] = new_state[turn * 2 + 1] + action[i][0]
                new_state[turn * 2 + 2] = new_state[turn * 2 + 2] + action[i][1]
                new_state = tuple(new_state)

                # the second element indicates whether the cost should be increased or not
                # 1 for yes, 0 for no
                successors.append((new_state, 1))

            elif i == 0:
                # append no action state, cost remains the same
                successors.append((state, 0))

        return successors
        # given a sequence of states (including robot turn), modify the maze and print it out.
        #  (Be careful, this does modify the maze!)

    def animate_path(self, path):
        # reset the robot locations in the maze
        self.maze.robotloc = tuple(self.start_state[1:])

        for state in path:
            print(str(self))
            self.maze.robotloc = tuple(state[1:])
            sleep(1)

            print(str(self.maze))

    def manhattan_heuristic(self, state):

        robotNum = int(len(state) / 2)
        goal = self.goal_locations
        heuristic = 0
        for i in range(0, robotNum):
            heuristic = heuristic + abs(goal[i * 2] - state[i * 2 + 1]) + abs(goal[i * 2 + 1] - state[i * 2 + 2])
        return heuristic


    def bfs_heuristic(self, state):
        maze = self.maze
        start_state = maze.start_state
        robotloc = maze.robotloc
        heuristic = 0
        robotNum = int(len(state) / 2)
     
        individual_loc = []

        for i in range(0, robotNum):
            individual_loc.append((state[i * 2 + 1], state[i * 2 + 2]))

        # perform bfs on each location
        for i in range(0, len(individual_loc)):
            
            start = individual_loc[i]
            maze.robotloc = start
            maze.start_state = (0, start[0], start[1])

            new_mp = MazeworldProblem(maze, (self.goal_locations[i * 2], self.goal_locations[i * 2 + 1]))
            result = bfs_search(new_mp)
            heuristic = heuristic + len(result.path) - 1

        maze.robotloc = robotloc
        maze.start_state = start_state

        return heuristic
        #test_mp = MazeworldProblem(self.maze, (8, 7, 9, 7, 10, 7))
        #for i in range(0, robotNum):
        #    result = bfs_search(self)
        #    heuristic = heuristic + len(result.path)

        #return heuristic




## A bit of test code. You might want to add to it to verify that things
#  work as expected.

if __name__ == "__main__":
    test_maze3 = Maze("maze3.maz")
    test_maze2 = Maze("maze2.maz")
    test_mp = MazeworldProblem(test_maze2, (1, 1, 1, 2, 1, 3))

    print(test_mp.get_successors((1, 0, 0, 0, 1, 3, 1)))
    #print(test_mp.manhattan_heuristic((0, 1, 0, 1, 2, 2, 1)))
    print(test_mp.bfs_heuristic((0, 0, 0, 1, 0, 2, 0)))
    #test_mp = MazeworldProblem(test_maze2, (3, 0))
    #print(test_mp.get_successors((1, 1)))