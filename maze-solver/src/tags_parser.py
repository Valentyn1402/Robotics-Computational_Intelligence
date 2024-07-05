import itertools
from pathlib import Path

import numpy as np
import yaml


def parse(file: Path, tile_size: float = 0.25, wall_thickness: float = 0.003):
    """
    Parse the tags.yaml file and return the layout of the maze
    :param file: file path to the tags.yaml file
    :param tile_size: the size of the tiles in the maze in meters
    :return:
    """
    with file.open() as f:
        tags = yaml.safe_load(f)

    bundle = tags['tag_bundles'][0]
    print(bundle['name'])

    layout: list[dict[str, float]] = bundle['layout']

    # get all coordinates of the walls without duplicates
    walls = {(tag["y"], tag["x"]) for tag in layout}

    x_values = [tag["x"] for tag in layout]
    y_values = [tag["y"] for tag in layout]

    max_x = max(x_values)
    max_y = max(y_values)

    # get the width and height of the maze
    width = int(max_y / tile_size)
    height = int(max_x / tile_size)

    print(f"Maze shape: {width}x{height}")

    # 2 = right, 3 = left, 1 = down, 0 = up
    connectivity_matrix = np.zeros((width, height, 4), dtype=bool)

    for x, y in itertools.product(range(0, width - 1), range(0, height)):
        c_row = ((x + 1) * tile_size + wall_thickness, y * tile_size + tile_size / 2 + wall_thickness)
        if c_row not in walls:
            connectivity_matrix[x, y, 2] = True
            connectivity_matrix[x + 1, y, 3] = True

    for x, y in itertools.product(range(0, width), range(0, height - 1)):
        c_col = (
        x * (tile_size + wall_thickness) + tile_size / 2 + wall_thickness, (y + 1) * (tile_size + wall_thickness))
        if c_col not in walls:
            connectivity_matrix[x, y, 1] = True
            connectivity_matrix[x, y + 1, 0] = True


    return connectivity_matrix


if __name__ == '__main__':
    file = Path('../tags.yaml')
    parse(file)
