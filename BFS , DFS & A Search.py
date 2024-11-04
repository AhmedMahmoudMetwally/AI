import heapq
import math
import time
from collections import deque

# Define the goal state
goal_state = [
    [0, 1, 2],
    [3, 4, 5],
    [6, 7, 8]
]

# Define the possible moves
moves = {
    'Up': (-1, 0),
    'Down': (1, 0),
    'Left': (0, -1),
    'Right': (0, 1)
}

# Function to find the position of the blank space (0)
def find_blank(state):
    for i in range(3):
        for j in range(3):
            if state[i][j] == 0:
                return i, j

# Function to generate the next state after a move
def generate_next_state(state, move):
    i, j = find_blank(state)
    ni, nj = i + moves[move][0], j + moves[move][1]
    if 0 <= ni < 3 and 0 <= nj < 3:
        new_state = [row[:] for row in state]
        new_state[i][j], new_state[ni][nj] = new_state[ni][nj], new_state[i][j]
        return new_state
    return None

# Function to check if the current state is the goal state
def is_goal(state):
    return state == goal_state

# Manhattan heuristic function
def manhattan_heuristic(state):
    h = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] != 0:
                goal_i, goal_j = divmod(state[i][j], 3)
                h += abs(i - goal_i) + abs(j - goal_j)
    return h

# Euclidean heuristic function
def euclidean_heuristic(state):
    h = 0
    for i in range(3):
        for j in range(3):
            if state[i][j] != 0:
                goal_i, goal_j = divmod(state[i][j], 3)
                h += math.sqrt((i - goal_i) ** 2 + (j - goal_j) ** 2)
    return h

# BFS algorithm with accurate node expansion count
# BFS algorithm
def bfs(initial_state):
    queue = deque([(initial_state, [])])
    visited = set()
    nodes_expanded = 0
    start_time = time.time()
    
    while queue:
        state, path = queue.popleft()
        
        if is_goal(state):
            end_time = time.time()
            return path, len(path), nodes_expanded, len(path), end_time - start_time
        
        visited.add(tuple(map(tuple, state)))
        nodes_expanded += 1
        
        for move in moves:
            next_state = generate_next_state(state, move)
            if next_state and tuple(map(tuple, next_state)) not in visited:
                queue.append((next_state, path + [move]))
    
    return None, None, None, None, None

# DFS recursive algorithm with accurate node expansion count
def dfs_recursive(state, path, visited, depth_limit, nodes_expanded):
    if is_goal(state):
        return path, nodes_expanded + 1

    if depth_limit == 0:
        return None, nodes_expanded

    visited.add(tuple(map(tuple, state)))
    nodes_expanded += 1

    for move in moves:
        next_state = generate_next_state(state, move)
        next_state_tuple = tuple(map(tuple, next_state)) if next_state else None
        if next_state and next_state_tuple not in visited:
            result_path, result_nodes_expanded = dfs_recursive(
                next_state, path + [move], visited, depth_limit - 1, nodes_expanded
            )
            if result_path:
                return result_path, result_nodes_expanded

    visited.remove(tuple(map(tuple, state)))
    return None, nodes_expanded

# DFS wrapper function with depth limit of 5
def dfs(initial_state, depth_limit=5):
    visited = set()
    start_time = time.time()
    path, nodes_expanded = dfs_recursive(initial_state, [], visited, depth_limit, 0)
    end_time = time.time()
    
    if path is not None:
        return path, len(path), nodes_expanded, len(path), end_time - start_time
    return None, None, None, None, None

# A* search algorithm with heuristic function as parameter
def a_star_search(initial_state, heuristic_func):
    frontier = []
    heapq.heappush(frontier, (0 + heuristic_func(initial_state), initial_state, []))
    visited = set()
    nodes_expanded = 0
    start_time = time.time()
    
    while frontier:
        _, current_state, path = heapq.heappop(frontier)
        
        if is_goal(current_state):
            end_time = time.time()
            return path, len(path), nodes_expanded + 1, len(path), end_time - start_time
        
        visited.add(tuple(map(tuple, current_state)))
        nodes_expanded += 1
        
        for move in moves:
            next_state = generate_next_state(current_state, move)
            if next_state and tuple(map(tuple, next_state)) not in visited:
                new_path = path + [move]
                heapq.heappush(frontier, (len(new_path) + heuristic_func(next_state), next_state, new_path))
    
    return None, None, None, None, None

# Function to print the puzzle state
def print_puzzle(state):
    for row in state:
        print(' '.join(map(str, row)))
    print()

# Function to take user input for the initial state
def get_initial_state():
    initial_state = []
    print("Enter the initial state as 3 rows of 3 numbers each (use 0 for the blank space):")
    for i in range(3):
        while True:
            try:
                row = list(map(int, input(f"Enter row {i + 1} (3 numbers separated by spaces): ").split()))
                if len(row) == 3:
                    initial_state.append(row)
                    break
                else:
                    print("Please enter exactly 3 numbers for this row.")
            except ValueError:
                print("Invalid input. Please enter integers only.")
    return initial_state

# Get the initial state from the user
initial_state = get_initial_state()

# Executing and displaying results for each algorithm
print("BFS Search:")
bfs_path, bfs_cost, bfs_nodes_expanded, bfs_depth, bfs_time = bfs(initial_state)
print("Path:", bfs_path)
print("Cost of path:", bfs_cost)
print("Nodes expanded:", bfs_nodes_expanded)
print("Search depth:", bfs_depth)
print("Running time:", bfs_time, "seconds")

print("\nDFS Search:")
dfs_path, dfs_cost, dfs_nodes_expanded, dfs_depth, dfs_time = dfs(initial_state)
print("Path:", dfs_path)
print("Cost of path:", dfs_cost)
print("Nodes expanded:", dfs_nodes_expanded)
print("Search depth:", dfs_depth)
print("Running time:", dfs_time, "seconds")

print("\nA* Search using Manhattan Heuristic:")
a_star_manhattan_path, a_star_manhattan_cost, a_star_manhattan_nodes_expanded, a_star_manhattan_depth, a_star_manhattan_time = a_star_search(initial_state, manhattan_heuristic)
print("Path:", a_star_manhattan_path)
print("Cost of path:", a_star_manhattan_cost)
print("Nodes expanded:", a_star_manhattan_nodes_expanded)
print("Search depth:", a_star_manhattan_depth)
print("Running time:", a_star_manhattan_time, "seconds")

print("\nA* Search using Euclidean Heuristic:")
a_star_euclidean_path, a_star_euclidean_cost, a_star_euclidean_nodes_expanded, a_star_euclidean_depth, a_star_euclidean_time = a_star_search(initial_state, euclidean_heuristic)
print("Path:", a_star_euclidean_path)
print("Cost of path:", a_star_euclidean_cost)
print("Nodes expanded:", a_star_euclidean_nodes_expanded)
print("Search depth:", a_star_euclidean_depth)
print("Running time:", a_star_euclidean_time, "seconds")