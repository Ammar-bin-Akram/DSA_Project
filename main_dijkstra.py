import networkx as nx
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time


def plot_weighted_edges_and_path_with_delay(
    weighted_edges, path, start, end, delay=0.15
):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Plotting weighted edges with delay
    for edge in weighted_edges:
        (x1, y1, z1), (x2, y2, z2), weight = edge
        ax.plot([x1, x2], [y1, y2], [z1, z2], c="blue", linestyle="dashed", marker="o")
        # plt.pause(delay)

    # Plotting the path with delay
    if path:
        path_coords = list(zip(*path))
        for i in range(len(path_coords[0])):
            ax.plot(
                path_coords[0][: i + 1],
                path_coords[1][: i + 1],
                path_coords[2][: i + 1],
                c="green",
                marker="o",
                linestyle="-",
            )
            plt.pause(delay)

    ax.text(-6, 3, 2.5, "Time taken: " + str(totaltime), color="green")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Start: " + str(start) + " \n Goal: " + str(end))

    plt.show()


def plot_3d_path(path, obstacles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    # Plotting obstacles
    obstacle_coords = list(zip(*obstacles))
    ax.scatter(
        obstacle_coords[0], obstacle_coords[1], obstacle_coords[2], c="red", marker="X"
    )

    ax.text(2, 2, 3, "Start: (0, 0, 0) \n Goal: (0, 4, 0)", color="red", fontsize=12)

    # for loop to add text of points in the path
    for point in path:
        ax.text(point[0], point[1], point[2], f"{point}", color="black", fontsize=8)

    # Plotting path
    if path:
        path_coords = list(zip(*path))
        ax.plot(
            path_coords[0],
            path_coords[1],
            path_coords[2],
            c="green",
            marker="o",
            linestyle="-",
        )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.show()


def calculate_shortest_path(grid, weighted_edges, start_node, end_node):
    # Create a graph
    G = nx.Graph()

    # Add nodes to the graph
    for node in grid.keys():
        print(node)
        G.add_node(node)

    # Add weighted edges to the graph
    for edge in weighted_edges:
        node1, node2, weight = edge
        G.add_edge(node1, node2, weight=weight)

    try:
        # Calculate the shortest path using Dijkstra's algorithm
        shortest_path = nx.shortest_path(
            G, source=start_node, target=end_node, weight="weight"
        )
        shortest_path_length = nx.shortest_path_length(
            G, source=start_node, target=end_node, weight="weight"
        )
        return shortest_path, shortest_path_length
    except nx.NetworkXNoPath:
        return None, None


# Example usage
start_node = (0, 0, 0)
end_node = (3, 4, 2)

weighted_edges = [
    ((0, 0, 0), (1, 0, 0), 1.0),
    ((0, 0, 0), (0, 1, 0), 1.0),
    ((1, 0, 0), (2, 0, 0), 1.0),
    ((1, 0, 0), (1, 1, 0), 1.0),
    ((2, 0, 0), (3, 0, 0), 1.0),
    ((2, 0, 0), (2, 1, 0), 1.0),
    ((3, 0, 0), (4, 0, 0), 1.0),
    ((3, 0, 0), (3, 1, 0), 1.0),
    ((4, 0, 0), (4, 1, 0), 1.0),
    ((0, 1, 0), (1, 1, 0), 1.0),
    ((0, 1, 0), (0, 2, 0), 1.0),
    ((1, 1, 0), (2, 1, 0), 1.0),
    ((1, 1, 0), (1, 2, 0), 1.0),
    ((2, 1, 0), (3, 1, 0), 1.0),
    ((2, 1, 0), (2, 2, 0), 1.0),
    ((3, 1, 0), (4, 1, 0), 1.0),
    ((3, 1, 0), (3, 2, 0), 1.0),
    ((4, 1, 0), (4, 2, 0), 1.0),
    ((0, 2, 0), (1, 2, 0), 1.0),
    ((0, 2, 0), (0, 3, 0), 1.0),
    ((1, 2, 0), (2, 2, 0), 1.0),
    ((1, 2, 0), (1, 3, 0), 1.0),
    ((2, 2, 0), (3, 2, 0), 1.0),
    ((2, 2, 0), (2, 3, 0), 1.0),
    ((3, 2, 0), (4, 2, 0), 1.0),
    ((3, 2, 0), (3, 3, 0), 1.0),
    ((4, 2, 0), (4, 3, 0), 1.0),
    ((0, 3, 0), (1, 3, 0), 1.0),
    ((0, 3, 0), (0, 4, 0), 1.0),
    ((1, 3, 0), (2, 3, 0), 1.0),
    ((1, 3, 0), (1, 4, 0), 1.0),
    ((2, 3, 0), (3, 3, 0), 1.0),
    ((2, 3, 0), (2, 4, 0), 1.0),
    ((3, 3, 0), (4, 3, 0), 1.0),
    ((3, 3, 0), (3, 4, 0), 1.0),
    ((4, 3, 0), (4, 4, 0), 1.0),
    ((0, 4, 0), (1, 4, 0), 1.0),
    ((1, 4, 0), (2, 4, 0), 1.0),
    ((2, 4, 0), (3, 4, 0), 1.0),
    ((3, 4, 0), (4, 4, 0), 1.0),
    ((0, 0, 1), (1, 0, 1), 1.0),
    ((0, 0, 1), (0, 1, 1), 1.0),
    ((1, 0, 1), (2, 0, 1), 1.0),
    ((1, 0, 1), (1, 1, 1), 1.0),
    ((2, 0, 1), (3, 0, 1), 1.0),
    ((2, 0, 1), (2, 1, 1), 1.0),
    ((3, 0, 1), (4, 0, 1), 1.0),
    ((3, 0, 1), (3, 1, 1), 1.0),
    ((4, 0, 1), (4, 1, 1), 1.0),
    ((0, 1, 1), (1, 1, 1), 1.0),
    ((0, 1, 1), (0, 2, 1), 1.0),
    ((1, 1, 1), (2, 1, 1), 1.0),
    ((1, 1, 1), (1, 2, 1), 1.0),
    ((2, 1, 1), (3, 1, 1), 1.0),
    ((2, 1, 1), (2, 2, 1), 1.0),
    ((3, 1, 1), (4, 1, 1), 1.0),
    ((3, 1, 1), (3, 2, 1), 1.0),
    ((4, 1, 1), (4, 2, 1), 1.0),
    ((0, 2, 1), (1, 2, 1), 1.0),
    ((0, 2, 1), (0, 3, 1), 1.0),
    ((1, 2, 1), (2, 2, 1), 1.0),
    ((1, 2, 1), (1, 3, 1), 1.0),
    ((2, 2, 1), (3, 2, 1), 1.0),
    ((2, 2, 1), (2, 3, 1), 1.0),
    ((3, 2, 1), (4, 2, 1), 1.0),
    ((3, 2, 1), (3, 3, 1), 1.0),
    ((4, 2, 1), (4, 3, 1), 1.0),
    ((0, 3, 1), (1, 3, 1), 1.0),
    ((0, 3, 1), (0, 4, 1), 1.0),
    ((1, 3, 1), (2, 3, 1), 1.0),
    ((1, 3, 1), (1, 4, 1), 1.0),
    ((2, 3, 1), (3, 3, 1), 1.0),
    ((2, 3, 1), (2, 4, 1), 1.0),
    ((3, 3, 1), (4, 3, 1), 1.0),
    ((3, 3, 1), (3, 4, 1), 1.0),
    ((4, 3, 1), (4, 4, 1), 1.0),
    ((0, 4, 1), (1, 4, 1), 1.0),
    ((1, 4, 1), (2, 4, 1), 1.0),
    ((2, 4, 1), (3, 4, 1), 1.0),
    ((3, 4, 1), (4, 4, 1), 1.0),
    ((0, 0, 0), (0, 0, 1), 1.0),
    ((0, 1, 0), (0, 1, 1), 1.0),
    ((0, 2, 0), (0, 2, 1), 1.0),
    ((0, 3, 0), (0, 3, 1), 1.0),
    ((0, 4, 0), (0, 4, 1), 1.0),
    ((1, 0, 0), (1, 0, 1), 1.0),
    ((1, 1, 0), (1, 1, 1), 1.0),
    ((1, 2, 0), (1, 2, 1), 1.0),
    ((1, 3, 0), (1, 3, 1), 1.0),
    ((1, 4, 0), (1, 4, 1), 1.0),
    ((2, 0, 0), (2, 0, 1), 1.0),
    ((2, 1, 0), (2, 1, 1), 1.0),
    ((2, 2, 0), (2, 2, 1), 1.0),
    ((2, 3, 0), (2, 3, 1), 1.0),
    ((2, 4, 0), (2, 4, 1), 1.0),
    ((3, 0, 0), (3, 0, 1), 1.0),
    ((3, 1, 0), (3, 1, 1), 1.0),
    ((3, 2, 0), (3, 2, 1), 1.0),
    ((3, 3, 0), (3, 3, 1), 1.0),
    ((3, 4, 0), (3, 4, 1), 1.0),
    ((4, 0, 0), (4, 0, 1), 1.0),
    ((4, 1, 0), (4, 1, 1), 1.0),
    ((4, 2, 0), (4, 2, 1), 1.0),
    ((4, 3, 0), (4, 3, 1), 1.0),
    ((4, 4, 0), (4, 4, 1), 1.0),
    ((0, 0, 2), (1, 0, 2), 1.0),
    ((0, 0, 2), (0, 1, 2), 1.0),
    ((1, 0, 2), (2, 0, 2), 1.0),
    ((1, 0, 2), (1, 1, 2), 1.0),
    ((2, 0, 2), (3, 0, 2), 1.0),
    ((2, 0, 2), (2, 1, 2), 1.0),
    ((3, 0, 2), (4, 0, 2), 1.0),
    ((3, 0, 2), (3, 1, 2), 1.0),
    ((4, 0, 2), (4, 1, 2), 1.0),
    ((0, 1, 2), (1, 1, 2), 1.0),
    ((0, 1, 2), (0, 2, 2), 1.0),
    ((1, 1, 2), (2, 1, 2), 1.0),
    ((1, 1, 2), (1, 2, 2), 1.0),
    ((2, 1, 2), (3, 1, 2), 1.0),
    ((2, 1, 2), (2, 2, 2), 1.0),
    ((3, 1, 2), (4, 1, 2), 1.0),
    ((3, 1, 2), (3, 2, 2), 1.0),
    ((4, 1, 2), (4, 2, 2), 1.0),
    ((0, 2, 2), (1, 2, 2), 1.0),
    ((0, 2, 2), (0, 3, 2), 1.0),
    ((1, 2, 2), (2, 2, 2), 1.0),
    ((1, 2, 2), (1, 3, 2), 1.0),
    ((2, 2, 2), (3, 2, 2), 1.0),
    ((2, 2, 2), (2, 3, 2), 1.0),
    ((3, 2, 2), (4, 2, 2), 1.0),
    ((3, 2, 2), (3, 3, 2), 1.0),
    ((4, 2, 2), (4, 3, 2), 1.0),
    ((0, 3, 2), (1, 3, 2), 1.0),
    ((0, 3, 2), (0, 4, 2), 1.0),
    ((1, 3, 2), (2, 3, 2), 1.0),
    ((1, 3, 2), (1, 4, 2), 1.0),
    ((2, 3, 2), (3, 3, 2), 1.0),
    ((2, 3, 2), (2, 4, 2), 1.0),
    ((3, 3, 2), (4, 3, 2), 1.0),
    ((3, 3, 2), (3, 4, 2), 1.0),
    ((4, 3, 2), (4, 4, 2), 1.0),
    ((0, 4, 2), (1, 4, 2), 1.0),
    ((1, 4, 2), (2, 4, 2), 1.0),
    ((2, 4, 2), (3, 4, 2), 1.0),
    ((3, 4, 2), (4, 4, 2), 1.0),
    ((0, 0, 1), (0, 0, 2), 1.0),
    ((0, 1, 1), (0, 1, 2), 1.0),
    ((0, 2, 1), (0, 2, 2), 1.0),
    ((0, 3, 1), (0, 3, 2), 1.0),
    ((0, 4, 1), (0, 4, 2), 1.0),
    ((1, 0, 1), (1, 0, 2), 1.0),
    ((1, 1, 1), (1, 1, 2), 1.0),
    ((1, 2, 1), (1, 2, 2), 1.0),
    ((1, 3, 1), (1, 3, 2), 1.0),
    ((1, 4, 1), (1, 4, 2), 1.0),
    ((2, 0, 1), (2, 0, 2), 1.0),
    ((2, 1, 1), (2, 1, 2), 1.0),
    ((2, 2, 1), (2, 2, 2), 1.0),
    ((2, 3, 1), (2, 3, 2), 1.0),
    ((2, 4, 1), (2, 4, 2), 1.0),
    ((3, 0, 1), (3, 0, 2), 1.0),
    ((3, 1, 1), (3, 1, 2), 1.0),
    ((3, 2, 1), (3, 2, 2), 1.0),
    ((3, 3, 1), (3, 3, 2), 1.0),
    ((3, 4, 1), (3, 4, 2), 1.0),
    ((4, 0, 1), (4, 0, 2), 1.0),
    ((4, 1, 1), (4, 1, 2), 1.0),
    ((4, 2, 1), (4, 2, 2), 1.0),
    ((4, 3, 1), (4, 3, 2), 1.0),
    ((4, 4, 1), (4, 4, 2), 1.0),
    ((0, 0, 3), (1, 0, 3), 1.0),
    ((0, 0, 3), (0, 1, 3), 1.0),
    ((1, 0, 3), (2, 0, 3), 1.0),
    ((1, 0, 3), (1, 1, 3), 1.0),
    ((2, 0, 3), (3, 0, 3), 1.0),
    ((2, 0, 3), (2, 1, 3), 1.0),
    ((3, 0, 3), (4, 0, 3), 1.0),
    ((3, 0, 3), (3, 1, 3), 1.0),
    ((4, 0, 3), (4, 1, 3), 1.0),
    ((0, 1, 3), (1, 1, 3), 1.0),
    ((0, 1, 3), (0, 2, 3), 1.0),
    ((1, 1, 3), (2, 1, 3), 1.0),
    ((1, 1, 3), (1, 2, 3), 1.0),
    ((2, 1, 3), (3, 1, 3), 1.0),
    ((2, 1, 3), (2, 2, 3), 1.0),
    ((3, 1, 3), (4, 1, 3), 1.0),
    ((3, 1, 3), (3, 2, 3), 1.0),
    ((4, 1, 3), (4, 2, 3), 1.0),
    ((0, 2, 3), (1, 2, 3), 1.0),
    ((0, 2, 3), (0, 3, 3), 1.0),
    ((1, 2, 3), (2, 2, 3), 1.0),
    ((1, 2, 3), (1, 3, 3), 1.0),
    ((2, 2, 3), (3, 2, 3), 1.0),
    ((2, 2, 3), (2, 3, 3), 1.0),
    ((3, 2, 3), (4, 2, 3), 1.0),
    ((3, 2, 3), (3, 3, 3), 1.0),
    ((4, 2, 3), (4, 3, 3), 1.0),
    ((0, 3, 3), (1, 3, 3), 1.0),
    ((0, 3, 3), (0, 4, 3), 1.0),
    ((1, 3, 3), (2, 3, 3), 1.0),
    ((1, 3, 3), (1, 4, 3), 1.0),
    ((2, 3, 3), (3, 3, 3), 1.0),
    ((2, 3, 3), (2, 4, 3), 1.0),
    ((3, 3, 3), (4, 3, 3), 1.0),
    ((3, 3, 3), (3, 4, 3), 1.0),
    ((4, 3, 3), (4, 4, 3), 1.0),
    ((0, 4, 3), (1, 4, 3), 1.0),
    ((1, 4, 3), (2, 4, 3), 1.0),
    ((2, 4, 3), (3, 4, 3), 1.0),
    ((3, 4, 3), (4, 4, 3), 1.0),
    ((0, 0, 2), (0, 0, 3), 1.0),
    ((0, 1, 2), (0, 1, 3), 1.0),
    ((0, 2, 2), (0, 2, 3), 1.0),
    ((0, 3, 2), (0, 3, 3), 1.0),
    ((0, 4, 2), (0, 4, 3), 1.0),
    ((1, 0, 2), (1, 0, 3), 1.0),
    ((1, 1, 2), (1, 1, 3), 1.0),
    ((1, 2, 2), (1, 2, 3), 1.0),
    ((1, 3, 2), (1, 3, 3), 1.0),
    ((1, 4, 2), (1, 4, 3), 1.0),
    ((2, 0, 2), (2, 0, 3), 1.0),
    ((2, 1, 2), (2, 1, 3), 1.0),
    ((2, 2, 2), (2, 2, 3), 1.0),
    ((2, 3, 2), (2, 3, 3), 1.0),
    ((2, 4, 2), (2, 4, 3), 1.0),
    ((3, 0, 2), (3, 0, 3), 1.0),
    ((3, 1, 2), (3, 1, 3), 1.0),
    ((3, 2, 2), (3, 2, 3), 1.0),
    ((3, 3, 2), (3, 3, 3), 1.0),
    ((3, 4, 2), (3, 4, 3), 1.0),
    ((4, 0, 2), (4, 0, 3), 1.0),
    ((4, 1, 2), (4, 1, 3), 1.0),
    ((4, 2, 2), (4, 2, 3), 1.0),
    ((4, 3, 2), (4, 3, 3), 1.0),
    ((4, 4, 2), (4, 4, 3), 1.0),
    ((0, 0, 4), (1, 0, 4), 1.0),
    ((0, 0, 4), (0, 1, 4), 1.0),
    ((1, 0, 4), (2, 0, 4), 1.0),
    ((1, 0, 4), (1, 1, 4), 1.0),
    ((2, 0, 4), (3, 0, 4), 1.0),
    ((2, 0, 4), (2, 1, 4), 1.0),
    ((3, 0, 4), (4, 0, 4), 1.0),
    ((3, 0, 4), (3, 1, 4), 1.0),
    ((4, 0, 4), (4, 1, 4), 1.0),
    ((0, 1, 4), (1, 1, 4), 1.0),
    ((0, 1, 4), (0, 2, 4), 1.0),
    ((1, 1, 4), (2, 1, 4), 1.0),
    ((1, 1, 4), (1, 2, 4), 1.0),
    ((2, 1, 4), (3, 1, 4), 1.0),
    ((2, 1, 4), (2, 2, 4), 1.0),
    ((3, 1, 4), (4, 1, 4), 1.0),
    ((3, 1, 4), (3, 2, 4), 1.0),
    ((4, 1, 4), (4, 2, 4), 1.0),
    ((0, 2, 4), (1, 2, 4), 1.0),
    ((0, 2, 4), (0, 3, 4), 1.0),
    ((1, 2, 4), (2, 2, 4), 1.0),
    ((1, 2, 4), (1, 3, 4), 1.0),
    ((2, 2, 4), (3, 2, 4), 1.0),
    ((2, 2, 4), (2, 3, 4), 1.0),
    ((3, 2, 4), (4, 2, 4), 1.0),
    ((3, 2, 4), (3, 3, 4), 1.0),
    ((4, 2, 4), (4, 3, 4), 1.0),
    ((0, 3, 4), (1, 3, 4), 1.0),
    ((0, 3, 4), (0, 4, 4), 1.0),
    ((1, 3, 4), (2, 3, 4), 1.0),
    ((1, 3, 4), (1, 4, 4), 1.0),
    ((2, 3, 4), (3, 3, 4), 1.0),
    ((2, 3, 4), (2, 4, 4), 1.0),
    ((3, 3, 4), (4, 3, 4), 1.0),
    ((3, 3, 4), (3, 4, 4), 1.0),
    ((4, 3, 4), (4, 4, 4), 1.0),
    ((0, 4, 4), (1, 4, 4), 1.0),
    ((1, 4, 4), (2, 4, 4), 1.0),
    ((2, 4, 4), (3, 4, 4), 1.0),
    ((3, 4, 4), (4, 4, 4), 1.0),
    ((0, 0, 3), (0, 0, 4), 1.0),
    ((0, 1, 3), (0, 1, 4), 1.0),
    ((0, 2, 3), (0, 2, 4), 1.0),
    ((0, 3, 3), (0, 3, 4), 1.0),
    ((0, 4, 3), (0, 4, 4), 1.0),
    ((1, 0, 3), (1, 0, 4), 1.0),
    ((1, 1, 3), (1, 1, 4), 1.0),
    ((1, 2, 3), (1, 2, 4), 1.0),
    ((1, 3, 3), (1, 3, 4), 1.0),
    ((1, 4, 3), (1, 4, 4), 1.0),
    ((2, 0, 3), (2, 0, 4), 1.0),
    ((2, 1, 3), (2, 1, 4), 1.0),
    ((2, 2, 3), (2, 2, 4), 1.0),
    ((2, 3, 3), (2, 3, 4), 1.0),
    ((2, 4, 3), (2, 4, 4), 1.0),
    ((3, 0, 3), (3, 0, 4), 1.0),
    ((3, 1, 3), (3, 1, 4), 1.0),
    ((3, 2, 3), (3, 2, 4), 1.0),
    ((3, 3, 3), (3, 3, 4), 1.0),
    ((3, 4, 3), (3, 4, 4), 1.0),
    ((4, 0, 3), (4, 0, 4), 1.0),
    ((4, 1, 3), (4, 1, 4), 1.0),
    ((4, 2, 3), (4, 2, 4), 1.0),
    ((4, 3, 3), (4, 3, 4), 1.0),
    ((4, 4, 3), (4, 4, 4), 1.0),
]


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
shortest_path, shortest_path_length = calculate_shortest_path(
    grid, weighted_edges, start_node, end_node
)
endtime = time.time()
totaltime = endtime - starttime

obstacles = [point for point, status in grid.items() if status == "obstacle"]

if shortest_path is not None:
    print(f"Shortest Path from {start_node} to {end_node}: {shortest_path}")
    print(f"Shortest Path Length: {shortest_path_length}")
    plot_weighted_edges_and_path_with_delay(
        weighted_edges, shortest_path, start_node, end_node, delay=0.5
    )
else:
    print(f"No path exists from {start_node} to {end_node}")
