#!/usr/bin/env python3

import itertools
import random
import os
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import font
from typing import Optional, Tuple, Set, List, Dict

import numpy as np

import maze_solver.src.tags_parser
from maze_solver.src.a_star import a_star
from maze_solver.src.dijkstra import dijkstra, Node


class Maze:
    def __init__(self, width: int, height: int, seed: Optional[int] = None):
        if seed is None:
            seed = int(datetime.now().timestamp())

        self.width: int = width
        self.height: int = height
        self.connectivity_matrix = np.zeros((width, height, 4), dtype=bool)
        self.start_coordinate: Optional[tuple] = None
        self.goal_coordinate: Optional[tuple] = None
        self.seed: int = seed

        # Set the seed for reproducibility
        random.seed(seed)

    @staticmethod
    def from_tags(tags: Path, tile_size: float = 0.25, wall_thickness: float = 0.003):
        cm, width, height = maze_solver.src.tags_parser.parse(tags, tile_size, wall_thickness)
        maze = Maze(width, height)
        maze.connectivity_matrix = cm
        return maze

    def generate(self):
        print(f"seed: {self.seed}")

        # Set the start and end coordinates
        self.set_random_start_end()
        print(f"start: {self.start_coordinate}")
        print(f"end: {self.goal_coordinate}")

        self.path = self.find_random_path()
        self.create_branch_paths()
        print(f"path: {self.path}")

        # Update the connectivity matrix
        for i in range(len(self.path) - 1):
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

    def find_random_path_recursive(self, current_coordinate: np.ndarray, path_set: Set[Tuple[int, int]]):
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

    def draw_maze(self, scale=100, wall_width=2, distances: Optional[Dict[Tuple[int, int], Node]] = None,
                  shortest_path: Optional[List[Node]] = None):
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
        #print(sorted(font.families()))
        font_ = font.Font(family='arial', size=16, weight='bold')

        # create the canvas
        canvas = tk.Canvas(window, width=self.width * scale, height=self.height * scale)
        canvas.pack()

        # draw the start and end coordinates
        start_x, start_y = self.start_coordinate
        end_x, end_y = self.goal_coordinate
        canvas.create_rectangle(start_x * scale, start_y * scale, start_x * scale + scale, start_y * scale + scale,
                                fill="blue",
                                outline="")
        canvas.create_rectangle(end_x * scale, end_y * scale, end_x * scale + scale, end_y * scale + scale, fill="red",
                                outline="")
        # write start and end
        if distances:
            canvas.create_text(start_x * scale + scale // 2, start_y * scale + scale // 2 - int(scale / 5),
                               text="START", font=font_)
            canvas.create_text(end_x * scale + scale // 2, end_y * scale + scale // 2 - int(scale / 5), text="END",
                               font=font_)
        else:
            canvas.create_text(start_x * scale + scale // 2, start_y * scale + scale // 2, text="START", font=font_)
            canvas.create_text(end_x * scale + scale // 2, end_y * scale + scale // 2, text="END", font=font_)

        #draw the maze
        for x, y in itertools.product(range(self.width), range(self.height)):
            canvas.create_line(
                x * scale,
                y * scale,
                x * scale + scale,
                y * scale,
                width=wall_width,
                fill=("lightgray" if self.connectivity_matrix[x, y, 0] else "black")
            )
            canvas.create_line(
                x * scale + scale,
                y * scale,
                x * scale + scale,
                y * scale + scale,
                width=wall_width,
                fill=("lightgray" if self.connectivity_matrix[x, y, 1] else "black")
            )
            canvas.create_line(
                x * scale,
                y * scale + scale,
                x * scale + scale,
                y * scale + scale,
                width=wall_width,
                fill=("lightgray" if self.connectivity_matrix[x, y, 2] else "black")
            )
            canvas.create_line(
                x * scale,
                y * scale,
                x * scale,
                y * scale + scale,
                width=wall_width,
                fill=("lightgray" if self.connectivity_matrix[x, y, 3] else "black")
            )

        if shortest_path:
            for i in range(len(shortest_path) - 1):
                x1, y1 = shortest_path[i].coord
                x2, y2 = shortest_path[i + 1].coord
                canvas.create_line(x1 * scale + scale // 2, y1 * scale + scale // 2, x2 * scale + scale // 2,
                                   y2 * scale + scale // 2, fill="yellow", width=4)

        if distances:
            for coord, node in distances.items():
                x, y = coord
                canvas.create_text(x * scale + scale // 2, y * scale + scale // 2, text=f"{node.dist}", font=font_)

        # run the tkinter main loop
        window.mainloop()


def main():
    # seed: 1718364844
    # seed: 1718365623
    # seed: 1718526565

    #a_star:
    # seed: 1719257926

    # perfect maze:
    # seed: 1719261105

    #  error: 1719261183
    #maze = Maze(4, 4)
    # dist = dijkstra(maze.connectivity_matrix, maze.start_coordinate, maze.goal_coordinate)
    # dist = a_star(maze.connectivity_matrix, maze.start_coordinate, maze.goal_coordinate)
    #
    # node = dist[maze.goal_coordinate]
    # shortest_path = [node]
    # while node.coord != maze.start_coordinate:
    #     node = node.prev
    #     shortest_path.append(node)
    #
    # maze.draw_maze(distances=dist, shortest_path=shortest_path)
    #maze.draw_maze()

    maze = Maze.from_tags(Path('/home/jetson/Downloads/5.yaml'))
    maze.start_coordinate = (0, 3)
    maze.goal_coordinate = (0, 1)
    maze.draw()
    #a_steps = 0
    #d_steps = 0

    #steps = 10000

    #for _ in range(steps):

    #    maze = Maze(4, 4, seed=random.randint(0, 1000000000))
    #    maze.generate()
    #    dist, ds = dijkstra(maze.connectivity_matrix, maze.start_coordinate, maze.goal_coordinate)
    #    dist, da = a_star(maze.connectivity_matrix, maze.start_coordinate, maze.goal_coordinate)

    #    a_steps += da
    #    d_steps += ds

    #    node = dist[maze.goal_coordinate]
    #    shortest_path = [node]
    #    while node.coord != maze.start_coordinate:
    #        node = node.prev
    #        shortest_path.append(node)

    #print(f"Average steps for Dijkstra: {d_steps / steps}")
    #print(f"Average steps for A*: {a_steps / steps}")

    #maze.draw_maze(distances=dist, shortest_path=shortest_path)
    #print()


def get_path(tags_file: Path, start: Tuple[int, int], end: Tuple[int, int], tile_size: float = 0.25,
             wall_thickness: float = 0.003):
    maze = Maze.from_tags(tags_file)
    maze.start_coordinate = start
    maze.goal_coordinate = end
    dist, steps = a_star(maze.connectivity_matrix, maze.start_coordinate, maze.goal_coordinate)

    node = dist[maze.goal_coordinate]
    shortest_path = [node]
    while node.coord != maze.start_coordinate:
        node = node.prev
        shortest_path.append(node)

    print(f"Shortest Path: {[n.coord for n in reversed(shortest_path)]}")

    coordinates = [
        np.array(node.coord) * (tile_size + wall_thickness) + tile_size / 2 + wall_thickness
        for node in reversed(shortest_path)
    ]

    return coordinates


if __name__ == "__main__":
    #print(os.getcwd())
    #path = get_path(tags_file=Path('maze-solver/tags.yaml'), start=(1, 0), end=(2, 2))
    #print(path)
    main()
