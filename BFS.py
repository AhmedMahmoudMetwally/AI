from collections import deque

# Define the goal state
goal_state = [[0, 1, 2],
              [3, 4, 5],
              [6, 7, 8]]  # 0 represents the blank tile

# Utility function to check if the current state is the goal state
def is_goal(state):
    return state == goal_state

# Utility function to find the position of the blank tile (0)
def find_blank(state):
    for i in range(len(state)):
        for j in range(len(state[i])):
            if state[i][j] == 0:
                return i, j

# Utility function to create a deep copy of the state
def copy_state(state):
    return [row[:] for row in state]

# Utility function to get the possible moves from the current blank tile position
def get_neighbors(pos):
    i, j = pos
    moves = []
    
    if i > 0:  # Move blank up
        moves.append((i - 1, j))
    if i < 2:  # Move blank down
        moves.append((i + 1, j))
    if j > 0:  # Move blank left
        moves.append((i, j - 1))
    if j < 2:  # Move blank right
        moves.append((i, j + 1))
    
    return moves

# BFS to solve the 8-puzzle
def bfs(start_state):
    # Queue for BFS
    queue = deque([(start_state, [])])  # (state, path to state)
    visited = set()
    
    # Convert the initial state to a tuple of tuples for immutability
    initial_state_tuple = tuple(tuple(row) for row in start_state)
    visited.add(initial_state_tuple)
    
    while queue:
        current_state, path = queue.popleft()
        
        # Check if the current state is the goal state
        if is_goal(current_state):
            return path
        
        # Find the position of the blank tile (0)
        blank_pos = find_blank(current_state)
        
        # Explore all possible neighbors (possible moves)
        for move in get_neighbors(blank_pos):
            new_state = copy_state(current_state)
            i, j = blank_pos  # Blank tile position
            ni, nj = move     # New blank tile position after the move
            
            # Swap the blank tile with the neighboring tile
            new_state[i][j], new_state[ni][nj] = new_state[ni][nj], new_state[i][j]
            
            # Convert the new state to a tuple of tuples for immutability
            new_state_tuple = tuple(tuple(row) for row in new_state)
            
            # If the new state hasn't been visited yet
            if new_state_tuple not in visited:
                visited.add(new_state_tuple)
                # Append the new state to the queue with the updated path
                queue.append((new_state, path + [new_state]))
    
    # If no solution is found
    return None

# Function to print the puzzle state
def print_puzzle(state):
    for row in state:
        print(row)
    print()

# Example usage
start_state = [[1, 2, 5],
               [3, 4, 0],
               [6, 7, 8]]

print("Initial state:")
print_puzzle(start_state)

result = bfs(start_state)

if result is not None:
    print("Solution found in {} steps:".format(len(result)))
    for step, state in enumerate(result, start=1):
        print("Step", step)
        print_puzzle(state)
else:
    print("No solution found.")