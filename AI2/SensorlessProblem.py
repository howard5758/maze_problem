# Author: Ping-Jung Liu
# Date: September 26th 2017
# COSC 76 Assignment 2: Mazeworld
# Acknowledgement: Professor Devin Balkom for providing the general structure 
from Maze import Maze
from time import sleep

class SensorlessProblem:

    ## You write the good stuff here:
    def __init__(self, maze, goal):
        self.maze = maze
        self.goal = goal
        self.start_state = create_start(maze)
        self.maze_size = maze.width * maze.height

    # the state contains two elements:
    # 1. the possible positions of robot
    # 2. the action its parent took to reach this state
    def goal_test(self, state):
        return len(state[0]) == 1 and state[0][0] == self.goal

    def get_successors(self, state):

        # no point for the robot to stay in one robot case
        action = [[1, 0], [0, 1], [-1, 0], [0, -1]]
        maze = self.maze

        successors = []

        for i in range(0, len(action)):
            new_state = []
            
            # modify every possible positions
            for j in range(0, len(state[0])):
                newX = state[0][j][0] + action[i][0]
                newY = state[0][j][1] + action[i][1]

                # append the new position only if it's legal
                if maze.is_floor(newX, newY):
                    new_state.append((newX, newY))
                # else append original position
                else:
                    new_state.append((state[0][j]))

            # set() can eliminate duplicates in a list or tuple
            # make sure the new_state is different from original state
            if len(new_state) > 0 and not set(new_state) == set(state[0]):
                new_state = list(set(new_state))

                # note that the state has two elements: all locations, and action
                # the 1 indicates cost should be increased
                successors.append(((tuple(new_state), tuple(action[i])), 1))

        successors = list(set(successors))
        return successors


            

    def __str__(self):
        string =  "Blind robot problem: "
        return string


        # given a sequence of states (including robot turn), modify the maze and print it out.
        #  (Be careful, this does modify the maze!)

    def animate_path(self, path):
        # reset the robot locations in the maze
        self.maze.robotloc = tuple(self.start_state)

        for state in path:
            print(str(self))
            self.maze.robotloc = tuple(state)
            sleep(1)

            print(str(self.maze))

    def num_heuristic(self, state):
        return len(state[0])

    def spam_heuristic(self, state):
        xs = []
        ys = []

        for i in range(0, len(state[0])):
            xs.append(state[0][i][0])
            ys.append(state[0][i][1])

        x_dist = max(xs) - min(xs)
        y_dist = max(ys) - min(ys)

        return x_dist + y_dist

# create a new start_state based on given maze
def create_start(maze):
    width = maze.width
    height = maze.height

    start = []

    for i in range(0, height):
        for j in range(0, width):
            if maze.is_floor(j, i):
                start.append((j, i))

    return (tuple(start), (0, 0))

## A bit of test code

if __name__ == "__main__":
    test_maze3 = Maze("maze4.maz")
    test_problem = SensorlessProblem(test_maze3, (2, 1))
    print(test_problem.get_successors(test_problem.start_state))
    
