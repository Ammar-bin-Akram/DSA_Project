import heapq
from multiprocessing import heap
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time


def plot_3d_grid_with_delay(grid, delay=0.1):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    for point, status in grid.items():
        if status == "obstacle":
            ax.scatter(point[0], point[1], point[2], color="black", marker="s")
        else:
            ax.scatter(point[0], point[1], point[2], color="white")

    plt.show(block=False)
    plt.pause(delay)


def plot_a_star_path_on_grid(path, grid, delay=0.1):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Plot the grid points first
    for point, status in grid.items():
        if status == "obstacle":
            ax.scatter(point[0], point[1], point[2], color="red", marker="s")
        else:
            ax.scatter(point[0], point[1], point[2], color="green")

    ax.text(2, 2, 3, "Start: (0, 0, 0) \n Goal: (3, 4, 2)", color="red", fontsize=12)
    ax.text(
        -2,
        3,
        3,
        "Time taken to find path is : " + str(totaltime),
        color="green",
        fontsize=12,
    )
    plt.show(block=False)
    plt.pause(delay)

    # Plot the A* path on the same grid
    for i in range(len(path) - 1):
        node1 = path[i]
        node2 = path[i + 1]
        ax.plot(
            [node1[0], node2[0]],
            [node1[1], node2[1]],
            [node1[2], node2[2]],
            color="blue",
            marker="o",
        )
        plt.show(block=False)
        plt.pause(delay)

    for point in path:
        ax.text(point[0], point[1], point[2], f"{point}", color="black", fontsize=8)

    plt.show()


def heuristic_cost_estimate(current, goal):
    return (
        (current[0] - goal[0]) ** 2
        + (current[1] - goal[1]) ** 2
        + (current[2] - goal[2]) ** 2
    ) ** 0.5


def fcost(gcost, hcost):
    return gcost + hcost


def calculate_neighbors(current_node):
    return [
        (current_node[0] + i, current_node[1] + j, current_node[2] + k)
        for i in range(-1, 2)
        for j in range(-1, 2)
        for k in range(-1, 2)
        if (i, j, k) != (0, 0, 0)
    ]


def a_star_3d(start, goal, grid):
    open_set = [(0, start)]
    closed_set = set()
    g_score = {start: 0}
    f_score = {start: heuristic_cost_estimate(start, goal)}
    current_f, current_node = (start, 0)
    path = []
    while open_set:
        current_f, current_node = heapq.heappop(open_set)
        closed_set.add(current_node)

        if current_node == goal:
            path.append(goal)
            endtime = time.time()
            print(endtime - starttime)
            return path

        path.append(current_node)

        neighbors = calculate_neighbors(current_node)

        for neighbor in neighbors:
            if neighbor not in grid:
                continue

            if grid[neighbor] == "obstacle":
                continue

            if neighbor in closed_set:
                continue

            tentative_g_score = g_score[current_node] + 1

            # Assuming each move has a cost of 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                # path[neighbor] = current_node
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic_cost_estimate(
                    neighbor, goal
                )
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None


start = (0, 0, 0)
goal = (3, 4, 2)

grid = {
    (0, 0, 0): "empty",
    (1, 0, 0): "empty",
    (2, 0, 0): "obstacle",
    (3, 0, 0): "empty",
    (4, 0, 0): "empty",
    (0, 1, 0): "empty",
    (1, 1, 0): "empty",
    (2, 1, 0): "obstacle",
    (3, 1, 0): "empty",
    (4, 1, 0): "empty",
    (0, 2, 0): "empty",
    (1, 2, 0): "empty",
    (2, 2, 0): "empty",
    (3, 2, 0): "empty",
    (4, 2, 0): "empty",
    (0, 3, 0): "empty",
    (1, 3, 0): "empty",
    (2, 3, 0): "empty",
    (3, 3, 0): "empty",
    (4, 3, 0): "empty",
    (0, 4, 0): "empty",
    (1, 4, 0): "empty",
    (2, 4, 0): "empty",
    (3, 4, 0): "empty",
    (4, 4, 0): "empty",
    (0, 0, 1): "empty",
    (1, 0, 1): "empty",
    (2, 0, 1): "empty",
    (3, 0, 1): "empty",
    (4, 0, 1): "empty",
    (0, 1, 1): "empty",
    (1, 1, 1): "empty",
    (2, 1, 1): "empty",
    (3, 1, 1): "empty",
    (4, 1, 1): "empty",
    (0, 2, 1): "empty",
    (1, 2, 1): "empty",
    (2, 2, 1): "empty",
    (3, 2, 1): "empty",
    (4, 2, 1): "empty",
    (0, 3, 1): "empty",
    (1, 3, 1): "empty",
    (2, 3, 1): "empty",
    (3, 3, 1): "empty",
    (4, 3, 1): "empty",
    (0, 4, 1): "empty",
    (1, 4, 1): "empty",
    (2, 4, 1): "empty",
    (3, 4, 1): "empty",
    (4, 4, 1): "empty",
    (0, 0, 2): "empty",
    (1, 0, 2): "empty",
    (2, 0, 2): "empty",
    (3, 0, 2): "empty",
    (4, 0, 2): "empty",
    (0, 1, 2): "empty",
    (1, 1, 2): "empty",
    (2, 1, 2): "empty",
    (3, 1, 2): "empty",
    (4, 1, 2): "empty",
    (0, 2, 2): "empty",
    (1, 2, 2): "empty",
    (2, 2, 2): "obstacle",
    (3, 2, 2): "empty",
    (4, 2, 2): "empty",
    (0, 3, 2): "empty",
    (1, 3, 2): "empty",
    (2, 3, 2): "empty",
    (3, 3, 2): "empty",
    (4, 3, 2): "empty",
    (0, 4, 2): "empty",
    (1, 4, 2): "empty",
    (2, 4, 2): "empty",
    (3, 4, 2): "empty",
    (4, 4, 2): "empty",
}

starttime = time.time()
path = a_star_3d(start, goal, grid)
endtime = time.time()
totaltime = endtime - starttime

if path is not None:
    print("A* Path:", path)

    # Plot A* path on the same grid with delay
    plot_a_star_path_on_grid(path, grid, delay=0.1)

else:
    print(f"No path exists from {start} to {goal}")
