# Author: Ping-Jung Liu
# Date: September 17th 2017
# COSC 76 Assignment 1: Missionaries and Cannibals
# Acknowledgement: Professor Devin Balkom for providing the general structure

from collections import deque
from SearchSolution import SearchSolution

# you might find a SearchNode class useful to wrap state objects,
#  keep track of current depth for the dfs, and point to parent nodes
class SearchNode:
    # each search node except the root has a parent node
    # and all search nodes wrap a state object

    def __init__(self, state, parent=None):
        self.state = state
        self.depth = 0
        # you write this part

# you might write other helper functions, too. For example,
#  I like to separate out backchaining, and the dfs path checking functions
#def goal_test(state):
#    return state == (0, 0, 0)

def bfs_search(search_problem):
    # create solution
    solution = SearchSolution(search_problem, "BFS")
    s_p = search_problem

    # create frontier que
    que = deque()
    # the visited dict is a mapping between child and parent
    visited = {}

    # put start_state into que and visited
    que.append(s_p.start_state)
    visited[s_p.start_state] = None

    visited_nodes = 0
    while(not len(que) == 0):

        # get the first element in que
        toVisit = que.popleft()

        # if toVisit state is the goal, backchain and return solution
        if(s_p.goal_test(toVisit)):
            solution.path = backchain(toVisit, visited)
            solution.nodes_visited = visited_nodes
            return solution

        # if not the goal, obtain its successors
        successors = s_p.get_successors(toVisit)

        # loop through the successors, if not visited, put to the back of que and save into visited
        for child in successors:
            if(not child in visited):
                que.append(child)
                visited[child] = toVisit
                visited_nodes = visited_nodes + 1

    # if the search never reach goal state, return [] as path            
    solution.path = []
    solution.nodes_visited = visited_nodes

    return solution

def backchain(state, visited):

    # use the child and parent pairs in visited to track solution path
    result = [state] 
    prev = visited[state]

    # if prev = none, then it must be the start state --> break the loop
    while(not prev == None):
        result.append(prev)
        prev = visited[prev]

    # reverse the result for correct representation
    result.reverse()
    return result
# Don't forget that your dfs function should be recursive and do path checking,
#  rather than memoizing (no visited set!) to be memory efficient

# We pass the solution along to each new recursive call to dfs_search
#  so that statistics like number of nodes visited or recursion depth
#  might be recorded
def dfs_search(search_problem, depth_limit=100, node=None, solution=None):
    # if no node object given, create a new search from starting state
    # create empty path and empty path_set for the sake of runtime
    path = []
    path_set = {}
    if node == None and solution == None:
        node = SearchNode(search_problem.start_state)
        solution = SearchSolution(search_problem, "DFS")

    # start recursive dfs to find solution
    path.append(node.state)
    path_set[node.state] = True
    path = dfspc(search_problem, node, path_set, path, depth_limit, solution)
    solution.path = path

    return solution 
    # you write this part

# set
def dfspc(search_problem, node, path_set, path, depth_limit, solution):

    solution.nodes_visited = solution.nodes_visited + 1
    s_p = search_problem

    ##### BASE CASE! if current node is goal, return current path
    if node.depth > depth_limit:
        return []
    elif s_p.goal_test(node.state):
        return path

    ##### RECURSIVE CASE!
    # get the successors of current node
    successors = s_p.get_successors(node.state)
  
    # loop through successors and find all those not already in current path
    for child in successors:
        if not child in path_set:

            # extend current path, and call dfspc on child for recursion
            path.append(child)
            path_set[child] = True
            node.depth = node.depth + 1
            node.state = child
            new_path = dfspc(s_p, node, path_set, path, depth_limit, solution)

            # if the new_path extended from child is [], it is not part of the solution
            # so remove the child from path
            if new_path == []:
                node.depth = node.depth - 1
                path.remove(child)
                del path_set[child]
            else:
                return path

    return []

 
def ids_search(search_problem, depth_limit=50):

    # depth limit initially 0
    depth = 0
    nodes_visited = 0
    solution = SearchSolution(search_problem, "IDS")

    # increase depth limit until dfs find a solution
    while solution.path == [] and depth < depth_limit:
        nodes_visited = nodes_visited + solution.nodes_visited
        solution = dfs_search(search_problem, depth)
        depth = depth + 1

    solution.search_method = "IDS"
    solution.nodes_visited = nodes_visited
    return solution

    # you write this part
