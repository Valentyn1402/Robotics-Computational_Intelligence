import functools
import itertools
from typing import Optional, Tuple, List

import numpy as np

coordinate = Tuple[int, int]


@functools.total_ordering
class Node:
    def __init__(self, coord, dist):
        self.coord = coord
        self.dist = dist
        self.prev: Optional["Node"] = None

    def __lt__(self, other):
        return self.dist < other.dist

    def __hash__(self):
        return hash(self.coord)

    def __repr__(self):
        return f"Node({self.coord}, {self.dist})"


def dijkstra(graph: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]):
    steps = 0

    dist: dict[coordinate, Node] = {}
    nodes: List[Node] = list()

    for coord in itertools.product(range(graph.shape[0]), range(graph.shape[1])):
        if coord == start:
            continue

        node = Node(coord, float("inf"))
        dist[coord] = node
        nodes.append(node)

    # Set the start node to 0
    dist[start] = Node(start, 0)
    nodes.insert(0, dist[start])

    while nodes:
        steps += 1

        node = nodes.pop(0)

        if node.coord == goal:
            print(f"Dijkstra steps needed: {steps}")
            break

        update = False

        # 2 = right, 3 = left, 1 = down, 0 = up
        for i, c in enumerate([(0, -1), (0, 1), (1, 0), (-1, 0)]):
            next_coord: tuple = tuple(np.add(node.coord, c))

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

            next_node = dist[next_coord]

            new_cost: float = node.dist + 1
            if new_cost < next_node.dist:
                next_node.prev = node
                next_node.dist = new_cost
                update = True

        if update:
            nodes.sort()

    return dist, steps
