import heapq
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def heuristic_cost_estimate(current, goal):
    return (
                   (current[0] - goal[0]) ** 2
                   + (current[1] - goal[1]) ** 2
                   + (current[2] - goal[2]) ** 2
           ) ** 0.5


def a_star_3d(start, goal, grid):
    open_set = [(0, start)]
    closed_set = set()
    g_score = {start: 0}
    f_score = {start: heuristic_cost_estimate(start, goal)}
    path = []

    while open_set:
        current_f, current_node = heapq.heappop(open_set)

        if current_node == goal:
            # Reconstruct and return the path if the goal is reached
            # while current_node in g_score:
            #     path.append(current_node)
            #     current_node = g_score[current_node]
            return path[::-1]

        closed_set.add(current_node)
        path.append(current_node)

        neighbors = [
            (current_node[0] + i, current_node[1] + j, current_node[2] + k)
            for i in range(-1, 2)
            for j in range(-1, 2)
            for k in range(-1, 2)
            if (i, j, k) != (0, 0, 0)
        ]

        for neighbor in neighbors:
            if neighbor not in grid:
                print(f"Skipping neighbor {neighbor} - not in grid")
                continue

            if grid[neighbor] == "obstacle":
                print(f"Skipping neighbor {neighbor} - obstacle")
                continue

            if neighbor in closed_set:
                print(f"Skipping neighbor {neighbor} - already closed")
                continue

            tentative_g_score = (
                    g_score[current_node] + 1
            )  # Assuming each move has a cost of 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic_cost_estimate(
                    neighbor, goal
                )
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

        # Print intermediate steps for debugging
        print("Current Node:", current_node)
        print("Open Set:", open_set)
        print("Closed Set:", closed_set)
        print("-----")

    path = path.append(goal)
    return None


def visualize_3d_grid(grid, path=[]):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for point, status in grid.items():
        if status == "empty":
            ax.scatter(point[0], point[1], point[2], c='blue', marker='o', s=50)
        elif status == "obstacle":
            ax.scatter(point[0], point[1], point[2], c='red', marker='o', s=50)

    # Visualize the A* path in green
    if path:
        path_nodes = list(zip(*path))
        ax.plot(path_nodes[0], path_nodes[1], path_nodes[2], color='green', linewidth=2, markersize=10)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Grid with A* Path')

    plt.show()


# Example usage:
start = (0, 0, 0)
goal = (3, 4, 2)

grid = {
    (0, 0, 0): "empty", (1, 0, 0): "empty", (2, 0, 0): "empty", (3, 0, 0): "empty", (4, 0, 0): "empty",
    (0, 1, 0): "empty", (1, 1, 0): "empty", (2, 1, 0): "obstacle", (3, 1, 0): "empty", (4, 1, 0): "empty",
    (0, 2, 0): "empty", (1, 2, 0): "empty", (2, 2, 0): "empty", (3, 2, 0): "empty", (4, 2, 0): "empty",
    (0, 3, 0): "empty", (1, 3, 0): "empty", (2, 3, 0): "empty", (3, 3, 0): "empty", (4, 3, 0): "empty",
    (0, 4, 0): "empty", (1, 4, 0): "empty", (2, 4, 0): "empty", (3, 4, 0): "empty", (4, 4, 0): "empty",

    (0, 0, 1): "empty", (1, 0, 1): "empty", (2, 0, 1): "empty", (3, 0, 1): "empty", (4, 0, 1): "empty",
    (0, 1, 1): "empty", (1, 1, 1): "empty", (2, 1, 1): "empty", (3, 1, 1): "empty", (4, 1, 1): "empty",
    (0, 2, 1): "empty", (1, 2, 1): "empty", (2, 2, 1): "empty", (3, 2, 1): "empty", (4, 2, 1): "empty",
    (0, 3, 1): "empty", (1, 3, 1): "empty", (2, 3, 1): "empty", (3, 3, 1): "empty", (4, 3, 1): "empty",
    (0, 4, 1): "empty", (1, 4, 1): "empty", (2, 4, 1): "empty", (3, 4, 1): "empty", (4, 4, 1): "empty",

    (0, 0, 2): "empty", (1, 0, 2): "empty", (2, 0, 2): "empty", (3, 0, 2): "empty", (4, 0, 2): "empty",
    (0, 1, 2): "empty", (1, 1, 2): "empty", (2, 1, 2): "empty", (3, 1, 2): "empty", (4, 1, 2): "empty",
    (0, 2, 2): "empty", (1, 2, 2): "empty", (2, 2, 2): "obstacle", (3, 2, 2): "empty", (4, 2, 2): "empty",
    (0, 3, 2): "empty", (1, 3, 2): "empty", (2, 3, 2): "empty", (3, 3, 2): "empty", (4, 3, 2): "empty",
    (0, 4, 2): "empty", (1, 4, 2): "empty", (2, 4, 2): "empty", (3, 4, 2): "empty", (4, 4, 2): "empty",
}

path = a_star_3d(start, goal, grid)
print("Path:", path)

visualize_3d_grid(grid, path)