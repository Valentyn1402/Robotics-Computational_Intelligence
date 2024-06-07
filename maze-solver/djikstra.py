import functools
import itertools

import numpy as np


@functools.total_ordering
class Node:
    def __init__(self, coord, dist):
        self.coord = coord
        self.dist = dist

    def __gt__(self, other):
        return self.dist[self.coord] > other.dist[other.coord]

    def __lt__(self, other):
        return self.dist[self.coord] < other.dist[other.coord]

    def __eq__(self, other):
        return self.dist[self.coord] == other.dist[other.coord]

    def __hash__(self):
        return hash(self.coord)

    def __repr__(self):
        return f"Node({self.coord}, {self.dist[self.coord]})"


def djikstra(graph: np.ndarray, start: tuple[int, int]):
    dist: dict[tuple[int, int], float] = {tuple(start): 0}
    prev: dict[tuple[int, int], tuple[int, int]] = dict()

    queue: list[Node] = []

    for coord in itertools.product(range(graph.shape[0]), range(graph.shape[1])):
        if coord != start:
            dist[coord] = float("inf")
        queue.append(Node(coord, dist))

    while queue:
        queue.sort()
        node = queue.pop(0)

        # 2 = right, 3 = left, 1 = down, 0 = up
        for i, c in enumerate([(0, -1), (0, 1), (1, 0), (-1, 0)]):
            next_coord: np.ndarray = np.add(node.coord, c)

            # Check if the next coordinate is out of bounds
            if (
                    next_coord[0] < 0
                    or next_coord[0] >= graph.shape[0]
                    or next_coord[1] < 0
                    or next_coord[1] >= graph.shape[1]
            ):
                continue

            # check if path is blocked by a wall
            if not graph[node.coord[0], node.coord[1], i]:
                continue

            alt = dist[node.coord] + 1
            if alt < dist[tuple(next_coord)]:
                prev[tuple(next_coord)] = node.coord
                dist[tuple(next_coord)] = alt

    return dist, prev
