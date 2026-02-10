
import random
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def _neighbors(cell, size):
    x, y = cell
    candidates = ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1))
    return [(nx, ny) for nx, ny in candidates if 0 <= nx < size and 0 <= ny < size]


def generate_with_dfs(size, start_pos, end_pos):
    maze = [[0 for _ in range(size)] for _ in range(size)]
    visited = [[False for _ in range(size)] for _ in range(size)]

    cells = [(x, y) for x in range(size) for y in range(size)]
    random.shuffle(cells)

    visited_count = 0
    root_idx = 0
    stack = []

    while visited_count < size * size:
        if not stack:
            while root_idx < len(cells) and visited[cells[root_idx][0]][cells[root_idx][1]]:
                root_idx += 1
            if root_idx >= len(cells):
                break

            root = cells[root_idx]
            root_idx += 1
            rx, ry = root
            visited[rx][ry] = True
            visited_count += 1
            maze[rx][ry] = 0
            stack.append(root)

        current = stack[-1]
        unvisited_neighbors = [
            neighbor
            for neighbor in _neighbors(current, size)
            if not visited[neighbor[0]][neighbor[1]]
        ]

        if not unvisited_neighbors:
            stack.pop()
            continue

        nx, ny = random.choice(unvisited_neighbors)
        visited[nx][ny] = True
        visited_count += 1

        should_block = random.random() < 0.3 and (nx, ny) not in (start_pos, end_pos)
        maze[nx][ny] = 1 if should_block else 0
        if not should_block:
            stack.append((nx, ny))

    sx, sy = start_pos
    tx, ty = end_pos
    maze[sx][sy] = "A"
    maze[tx][ty] = "T"
    return maze


def generate_mazes(limit=50, size=101):
    print(f"Generating {limit} mazes of size {size}x{size}...")

    mazes = []
    for _ in range(limit):
        start_pos = (random.randint(0, size - 1), random.randint(0, size - 1))
        end_pos = (random.randint(0, size - 1), random.randint(0, size - 1))

        if start_pos == end_pos:
            end_pos = ((end_pos[0] + 1) % size, end_pos[1])

        mazes.append(generate_with_dfs(size, start_pos, end_pos))

    return mazes


def save_unsolved_mazes(mazes, output_dir="mazes/unsolved"):
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    colors = {
        0: (255, 255, 255),  # unblocked
        1: (0, 0, 0),        # blocked
        "A": (255, 0, 0),    # start
        "T": (0, 255, 0),    # goal
    }

    for index, maze in enumerate(mazes):
        if not maze or not maze[0]:
            continue

        height = len(maze)
        width = len(maze[0])
        image_data = np.zeros((height, width, 3), dtype=np.uint8)

        for row_idx, row in enumerate(maze):
            for col_idx, cell in enumerate(row):
                image_data[row_idx, col_idx] = colors.get(cell, (255, 255, 255))

        plt.imsave(output_path / f"maze_{index:03d}.png", image_data)


def save_solved_maze(maze, path, index, output_dir="mazes/solved"):
    if not maze or not maze[0]:
        raise ValueError("maze must be a non-empty grid")

    height = len(maze)
    width = len(maze[0])
    image_data = np.zeros((height, width, 3), dtype=np.uint8)

    colors = {
        0: (255, 255, 255),  # unblocked
        1: (0, 0, 0),        # blocked
        "A": (255, 0, 0),    # start
        "T": (0, 255, 0),    # goal
    }

    for row_idx, row in enumerate(maze):
        for col_idx, cell in enumerate(row):
            image_data[row_idx, col_idx] = colors.get(cell, (255, 255, 255))

    for row_idx, col_idx in path:
        if 0 <= row_idx < height and 0 <= col_idx < width:
            image_data[row_idx, col_idx] = (0, 0, 255)  # path in blue

    for row_idx, row in enumerate(maze):
        for col_idx, cell in enumerate(row):
            if cell == "A":
                image_data[row_idx, col_idx] = (255, 0, 0)
            elif cell == "T":
                image_data[row_idx, col_idx] = (0, 255, 0)

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    filename = output_path / f"maze_{index:03d}.png"
    plt.imsave(filename, image_data)


def visualize_maze(maze, path=None, title="Maze"):
    if maze is None or not maze or not maze[0]:
        raise ValueError("maze must be a non-empty grid")

    height = len(maze)
    width = len(maze[0])
    image_data = np.zeros((height, width, 3), dtype=np.uint8)

    colors = {
        0: (255, 255, 255),  # unblocked
        1: (0, 0, 0),        # blocked
        "A": (255, 0, 0),    # start
        "T": (0, 255, 0),    # goal
    }

    for row_idx, row in enumerate(maze):
        for col_idx, cell in enumerate(row):
            image_data[row_idx, col_idx] = colors.get(cell, (255, 255, 255))

    if path:
        for row_idx, col_idx in path:
            if 0 <= row_idx < height and 0 <= col_idx < width:
                image_data[row_idx, col_idx] = (0, 0, 255)  # path in blue

    for row_idx, row in enumerate(maze):
        for col_idx, cell in enumerate(row):
            if cell == "A":
                image_data[row_idx, col_idx] = (255, 0, 0)
            elif cell == "T":
                image_data[row_idx, col_idx] = (0, 255, 0)

    plt.figure(figsize=(8, 8))
    plt.imshow(image_data, interpolation="nearest")
    plt.title(title)
    plt.axis("off")
    plt.show()
