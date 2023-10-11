import heapq

class Node:                     
    def __init__(self, state, cost): # Constructor for the Node class.
        self.state = state  # Assign the state of the node
        self.cost = cost # Assign the cost to reach this node

    def __lt__(self, other):
        return self.cost < other.cost # Compare the costs of the two nodes

def heuristic(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1]) # Calculate the Manhattan distance

def neighbors(current, grid):
    neighbors = []
    for i, j in [(0, -1), (-1, 0), (0, 1), (1, 0)]:
        x, y = current[0] + i, current[1] + j
        if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0:
            neighbors.append((x, y))
    return neighbors

def greedy_best_first_search(start, goal, grid):
    open_list = []
    heapq.heappush(open_list, Node(start, 0))
    explored = set()

    while open_list:
        current_node = heapq.heappop(open_list)
        current_state = current_node.state

        if current_state == goal:
            return current_state, explored  # Return the goal state and the explored set for visualization

        explored.add(current_state)

        for neighbor in neighbors(current_state, grid):
            if neighbor not in explored:
                cost = heuristic(neighbor, goal)
                heapq.heappush(open_list, Node(neighbor, cost))

    return None, explored  # No path found

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

# Test the Greedy Best-First Search
goal_state, explored = greedy_best_first_search(start, goal, grid)
print("Goal State:", goal_state)
print("Explored States:", explored)
