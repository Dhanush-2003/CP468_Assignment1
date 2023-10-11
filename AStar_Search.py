import heapq

class Node:
    def __init__(self, state, g, h):
        self.state = state    # Assign the state of the node
        self.g = g        # Assign the cost to reach this node from the start
        self.h = h        # Assign the heuristic estimate to reach the goal from this node

    def __lt__(self, other):
        return self.g + self.h < other.g + other.h   # Compare the total costs of the two nodes

def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1]) # Calculate the Manhattan distance

def neighbors(current, grid):
    neighbors = []     # Initialize the list to hold the neighboring states
    for i, j in [(0, -1), (-1, 0), (0, 1), (1, 0)]:    # Iterate through the possible directions
        x, y = current[0] + i, current[1] + j    # Calculate the coordinates of the neighboring cell
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0: # Check if the coordinates are valid and if the cell is not an obstacle
            neighbors.append((x, y))
    return neighbors

def a_star_search(start, goal, grid):
    open_list  = []
    heapq.heappush(open_list , Node(start, 0, heuristic(start, goal))) # Push the start node to the open_list
    explored = set()
    g_values = {start: 0}

    while open_list :
        current_node = heapq.heappop(open_list )
        current_state = current_node.state

        if current_state == goal:
            return current_state, explored  # Return the goal state and the explored set 

        explored.add(current_state)

        for neighbor in neighbors(current_state, grid):
            tentative_g_value = g_values[current_state] + 1  # Assuming each move has a cost of 1

            if neighbor not in g_values or tentative_g_value < g_values[neighbor]:
                g_values[neighbor] = tentative_g_value
                f_value = tentative_g_value + heuristic(neighbor, goal)
                heapq.heappush(open_list , Node(neighbor, tentative_g_value, heuristic(neighbor, goal)))

    return None, explored  

# Simple grid for testing (0 = free path, 1 = obstacle)
grid = [
    [0, 0, 0, 0, 1, 0],
    [1, 1, 0, 0, 1, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0]
]

# Start and goal positions
start = (0, 0)
goal = (4, 5)

# Test the A* Search
goal_state, explored = a_star_search(start, goal, grid)
print("Goal State:", goal_state)
print("Explored States:", explored)
