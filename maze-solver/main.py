import itertools
import random
from datetime import datetime
from typing import Optional
import tkinter as tk

import numpy as np

from djikstra import djikstra


class Maze:
    def __init__(self, width: int, height: int, seed: Optional[int] = None):
        if seed is None:
            seed = datetime.now().timestamp()

        self.width: int = width
        self.height: int = height
        self.connectivity_matrix = np.zeros((width, height, 4), dtype=bool)
        self.start_coordinate: Optional[tuple] = None
        self.goal_coordinate: Optional[tuple] = None
        self.seed: int = seed
        print(f"seed: {seed}")

        # Set the seed for reproducibility
        random.seed(seed)

        # Set the start and end coordinates
        self.set_random_start_end()
        print(f"start: {self.start_coordinate}")
        print(f"end: {self.goal_coordinate}")

        self.path = self.find_random_path()
        self.create_branch_paths()
        print(f"path: {self.path}")

        #Update the connectivity matrix
        for i in range(len(self.path)-1):
            current_coordinate = self.path[i]
            next_coordinate = self.path[i + 1]

            # 2 = right, 3 = left, 1 = down, 0 = up
            # Check if the next coordinate is to the right
            if next_coordinate[0] > current_coordinate[0]:
                self.connectivity_matrix[current_coordinate[0], current_coordinate[1], 2] = True
                self.connectivity_matrix[next_coordinate[0], next_coordinate[1], 3] = True
            # Check if the next coordinate is to the left
            elif next_coordinate[0] < current_coordinate[0]:
                self.connectivity_matrix[current_coordinate[0], current_coordinate[1], 3] = True
                self.connectivity_matrix[next_coordinate[0], next_coordinate[1], 2] = True
            # Check if the next coordinate is below
            elif next_coordinate[1] > current_coordinate[1]:
                self.connectivity_matrix[current_coordinate[0], current_coordinate[1], 1] = True
                self.connectivity_matrix[next_coordinate[0], next_coordinate[1], 0] = True
            # Check if the next coordinate is above
            else:
                self.connectivity_matrix[current_coordinate[0], current_coordinate[1], 0] = True
                self.connectivity_matrix[next_coordinate[0], next_coordinate[1], 1] = True

    def set_random_start_end(self):
        direction = random.randint(0, 3)

        if direction == 0:
            # Start from left, end at right
            self.start_coordinate = (0, random.randint(0, self.height - 1))
            self.goal_coordinate = (self.width - 1, random.randint(0, self.height - 1))
        elif direction == 1:
            # Start from top, end at bottom
            self.start_coordinate = (random.randint(0, self.width - 1), 0)
            self.goal_coordinate = (random.randint(0, self.width - 1), self.height - 1)
        elif direction == 2:
            # Start from right, end at left
            self.start_coordinate = (self.width - 1, random.randint(0, self.height - 1))
            self.goal_coordinate = (0, random.randint(0, self.height - 1))
        else:
            # Start from bottom, end at top
            self.start_coordinate = (random.randint(0, self.width - 1), self.height - 1)
            self.goal_coordinate = (random.randint(0, self.width - 1), 0)

    def find_random_path(self):
        return self.find_random_path_recursive(np.array(self.start_coordinate), {self.start_coordinate})

    def find_random_path_recursive(self, current_coordinate: np.ndarray, path_set: set[tuple[int, int]]):
        if tuple(current_coordinate) == self.goal_coordinate:
            return [current_coordinate]

        moves = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        random.shuffle(moves)
        for move in moves:

            next_coordinate = np.add(current_coordinate, move)

            # Check if the next coordinate is out of bounds
            if (
                    next_coordinate[0] < 0
                    or next_coordinate[0] >= self.width
                    or next_coordinate[1] < 0
                    or next_coordinate[1] >= self.height
            ):
                continue

            # Check if the next coordinate is already in the path
            t_next_coordinate = tuple(next_coordinate)
            if t_next_coordinate in path_set:
                continue

            path = self.find_random_path_recursive(next_coordinate, path_set | {t_next_coordinate})
            if path:
                return [current_coordinate] + path

        return None

    def create_branch_paths(self):
        visited = set()
        path_set = {tuple(a) for a in self.path}
        for coord in itertools.product(range(self.width), range(self.height)):
            if coord in visited or coord in path_set or coord == self.start_coordinate or coord == self.goal_coordinate:
                continue

            own_path = {coord}
            while True:
                moves = [(0, 1), (1, 0), (0, -1), (-1, 0)]
                random.shuffle(moves)
                move_failed = True
                for move in moves:
                    next_coordinate = np.add(coord, move)

                    # Check if the next coordinate is out of bounds
                    if (
                            next_coordinate[0] < 0
                            or next_coordinate[0] >= self.width
                            or next_coordinate[1] < 0
                            or next_coordinate[1] >= self.height
                    ):
                        continue

                    # Check if the next coordinate is already in the path
                    t_next_coordinate = tuple(next_coordinate)
                    if t_next_coordinate in own_path:
                        continue

                    # update the connectivity matrix
                    current_coordinate = coord
                    if next_coordinate[0] > current_coordinate[0]:
                        self.connectivity_matrix[current_coordinate[0], current_coordinate[1], 2] = True
                        self.connectivity_matrix[next_coordinate[0], next_coordinate[1], 3] = True
                    elif next_coordinate[0] < current_coordinate[0]:
                        self.connectivity_matrix[current_coordinate[0], current_coordinate[1], 3] = True
                        self.connectivity_matrix[next_coordinate[0], next_coordinate[1], 2] = True
                    elif next_coordinate[1] > current_coordinate[1]:
                        self.connectivity_matrix[current_coordinate[0], current_coordinate[1], 1] = True
                        self.connectivity_matrix[next_coordinate[0], next_coordinate[1], 0] = True
                    else:
                        self.connectivity_matrix[current_coordinate[0], current_coordinate[1], 0] = True
                        self.connectivity_matrix[next_coordinate[0], next_coordinate[1], 1] = True

                    visited.add(t_next_coordinate)
                    own_path.add(t_next_coordinate)
                    if t_next_coordinate not in path_set or t_next_coordinate not in visited:
                        move_failed = False
                    break

                if move_failed:
                    break

    def draw_maze(self, scale=50, wall_width=2, distances: Optional[dict[tuple[int, int], float]] = None, shortest_path: Optional[list[tuple[int, int]]] = None):
        """
        Draw the maze using tkinter. Each cell should be a square.
        The connectivity matrix should be used to determine which walls to draw.
        The start and end coordinates should be drawn in a different color.
        The wall color should be different from the path color.
        The wall facing outwards the maze at the start and end coordinates should not be drawn.
        :return:
        """

        # create the tkinter window
        window = tk.Tk()
        window.title("Maze")
        window.geometry(f"{self.width * scale}x{self.height * scale}")

        # create the canvas
        canvas = tk.Canvas(window, width=self.width * scale, height=self.height * scale)
        canvas.pack()

        # draw the start and end coordinates
        start_x, start_y = self.start_coordinate
        end_x, end_y = self.goal_coordinate
        canvas.create_rectangle(start_x * scale, start_y * scale, start_x * scale + scale, start_y * scale + scale, fill="blue",
                                outline="")
        canvas.create_rectangle(end_x * scale, end_y * scale, end_x * scale + scale, end_y * scale + scale, fill="red", outline="")

        #draw the maze
        for x, y in itertools.product(range(self.width), range(self.height)):
            if not self.connectivity_matrix[x, y, 0]:
                canvas.create_line(x * scale, y * scale, x * scale + scale, y * scale, width=wall_width)
            if not self.connectivity_matrix[x, y, 1]:
                canvas.create_line(x * scale, y * scale + scale, x * scale + scale, y * scale + scale, width=wall_width)
            if not self.connectivity_matrix[x, y, 2]:
                canvas.create_line(x * scale + scale, y * scale, x * scale + scale, y * scale + scale, width=wall_width)
            if not self.connectivity_matrix[x, y, 3]:
                canvas.create_line(x * scale, y * scale, x * scale, y * scale + scale, width=wall_width)

        if shortest_path:
            for i in range(len(shortest_path) - 1):
                x1, y1 = shortest_path[i]
                x2, y2 = shortest_path[i + 1]
                canvas.create_line(x1 * scale + scale // 2, y1 * scale + scale // 2, x2 * scale + scale // 2, y2 * scale + scale // 2, fill="green", width=2)

        if distances:
            for coord, dist in distances.items():
                x, y = coord
                canvas.create_text(x * scale + scale // 2, y * scale + scale // 2, text=str(dist))

        # run the tkinter main loop
        window.mainloop()


def main():
    maze = Maze(4, 4)
    dist, prev = djikstra(maze.connectivity_matrix, maze.start_coordinate)

    node = maze.goal_coordinate
    shortest_path = [node]
    while node != maze.start_coordinate:
        node = prev[node]
        shortest_path.append(node)

    maze.draw_maze(distances=dist, shortest_path=shortest_path)
    print()

if __name__ == "__main__":
    main()
