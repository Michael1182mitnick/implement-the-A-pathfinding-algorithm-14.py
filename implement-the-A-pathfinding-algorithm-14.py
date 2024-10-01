# Implement The A Pathfinding Algorithm
# Create a program that uses the A* algorithm to find the shortest path between two points on a grid. Include obstacles and different terrains with varying traversal costs.

import heapq


class Node:
    def __init__(self, position, parent=None, g=0, h=0):
        self.position = position  # (x, y)
        self.parent = parent      # Parent node (for path tracing)
        self.g = g  # Cost from start node
        self.h = h  # Heuristic cost to goal node
        self.f = g + h  # Total cost

    def __lt__(self, other):
        return self.f < other.f  # Compare nodes by f-value


# Heuristic: Manhattan distance
def manhattan_distance(current, goal):
    return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

# A* algorithm implementation


def astar(grid, start, goal, cost_map):
    open_list = []
    closed_list = set()

    # Initialize the start node
    start_node = Node(start, g=0, h=manhattan_distance(start, goal))
    heapq.heappush(open_list, start_node)

    # Directions for moving in the grid (up, down, left, right, diagonals)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                  (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while open_list:
        # Get the node with the lowest f-value
        current_node = heapq.heappop(open_list)

        # If we reach the goal, trace the path and return
        if current_node.position == goal:
            return reconstruct_path(current_node)

        # Add the current node to the closed list
        closed_list.add(current_node.position)

        # Explore neighbors
        for direction in directions:
            neighbor_pos = (
                current_node.position[0] + direction[0], current_node.position[1] + direction[1])

            # Check if the neighbor is within bounds
            if not (0 <= neighbor_pos[0] < len(grid) and 0 <= neighbor_pos[1] < len(grid[0])):
                continue

            # Check if the neighbor is an obstacle
            if grid[neighbor_pos[0]][neighbor_pos[1]] == -1:  # -1 for obstacle
                continue

            # If the neighbor is already in the closed list, skip it
            if neighbor_pos in closed_list:
                continue

            # Calculate the g, h, and f costs
            # Get the cost from the cost map
            traversal_cost = cost_map[grid[neighbor_pos[0]][neighbor_pos[1]]]
            # g(n) = cost to move from start to neighbor
            g_cost = current_node.g + traversal_cost
            # h(n) = heuristic (Manhattan distance)
            h_cost = manhattan_distance(neighbor_pos, goal)

            neighbor_node = Node(position=neighbor_pos,
                                 parent=current_node, g=g_cost, h=h_cost)

            # Check if the neighbor is already in the open list with a lower f-value
            in_open_list = False
            for open_node in open_list:
                if neighbor_node.position == open_node.position and neighbor_node.g >= open_node.g:
                    in_open_list = True
                    break

            if not in_open_list:
                heapq.heappush(open_list, neighbor_node)

    return None  # No path found


# Function to reconstruct the path by following parent pointers
def reconstruct_path(node):
    path = []
    current = node
    while current:
        path.append(current.position)
        current = current.parent
    return path[::-1]  # Reverse the path to go from start to goal


# Example grid: -1 represents obstacles, 0 represents normal terrain, higher values represent difficult terrains
grid = [
    [0, 0, 0, -1, 0, 0, 0],
    [0, -1, 0, -1, 0, -1, 0],
    [0, -1, 0, 0, 0, -1, 0],
    [0, 0, 0, -1, 0, 0, 0],
    [0, -1, -1, -1, -1, -1, 0],
    [0, 0, 0, 0, 0, 0, 0],
]

# Cost map: 0 = normal terrain, 1 = forest, 2 = water, -1 = obstacle
cost_map = {
    0: 1,   # Normal terrain with cost 1
    1: 5,   # Forest terrain with cost 5
    2: 10,  # Water terrain with cost 10
}

# Define start and goal positions
start = (0, 0)  # Starting at top-left corner
goal = (5, 6)   # Goal at bottom-right corner

# Run the A* algorithm
path = astar(grid, start, goal, cost_map)

if path:
    print("Path found:", path)
else:
    print("No path found.")
