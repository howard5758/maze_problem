from Maze import Maze
from time import sleep

class SensorlessProblem:

    ## You write the good stuff here:
    def __init__(self, maze, goal):
        self.maze = maze
        self.goal = goal
        self.start_state = create_start(maze)
        self.maze_size = maze.width * maze.height

    def goal_test(self, state):
        return state[0][0] == self.goal

    def get_successors(self, state):

        action = [[1, 0], [0, 1], [-1, 0], [0, -1]]
        maze = self.maze

        successors = []

        for i in range(0, len(action)):
            newX = state[1][0] + action[i][0]
            newY = state[1][1] + action[i][1]

            if maze.is_floor(newX, newY):
                new_table = modify_table(maze, state[0], action[i])
                successors.append(((new_table, (newX, newY)), 1))

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

    def test_heuristic(self, state):
        return len(state[0])

def modify_table(maze, table, action):
    new_table = []

    for i in range(0, len(table)):
        if maze.is_floor(table[i][0] + action[0], table[i][1] + action[1]):
            new_table.append((table[i][0] + action[0], table[i][1] + action[1]))

    return tuple(new_table)


def create_start(maze):
    width = maze.width
    height = maze.height

    start = []

    for i in range(0, height):
        for j in range(0, width):
            if maze.is_floor(j, i):
                start.append((j, i))

    return (tuple(start), tuple(maze.robotloc))

## A bit of test code

if __name__ == "__main__":
    test_maze1 = Maze("maze1.maz")
    test_problem = SensorlessProblem(test_maze1, (2, 1))
    print(test_maze1)
    print(test_problem.goal)