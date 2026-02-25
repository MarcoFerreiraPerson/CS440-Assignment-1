"""
gen_test_json.py — Generate N random 101x101 mazes and save as mazes.json. Uses same algorithm as maze_generator.py.

Usage:
    python gen_test_json.py [--num_mazes N] [--seed S] [--output FILE]
"""
import json
import random
import argparse
import random
from constants import ROWS
from tqdm import tqdm
import argparse

# set random seed for reproducibility
random.seed(42)


def create_maze() -> list:
    # TODO: Implement this function to generate and return a random maze as a 2D list of 0s and 1s.
    start = (0, 0)
    goal = (ROWS - 1, ROWS - 1)

    def neighbors(cell, size):
        x, y = cell
        candidates = ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1))
        for nx, ny in candidates:
            if 0 <= nx < size and 0 <= ny < size:
                yield (nx, ny)

    maze = []
    visited = []
    for _ in range(ROWS):
        maze_row = []
        visited_row = []
        for _ in range(ROWS):
            maze_row.append(0)
            visited_row.append(False)
        maze.append(maze_row)
        visited.append(visited_row)

    cells = []
    for x in range(ROWS):
        for y in range(ROWS):
            cells.append((x, y))
    random.shuffle(cells)

    visited_count = 0
    root_idx = 0
    stack = []

    while visited_count < ROWS * ROWS:
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
        unvisited_neighbors = []
        for neighbor in neighbors(current, ROWS):
            if not visited[neighbor[0]][neighbor[1]]:
                unvisited_neighbors.append(neighbor)

        if not unvisited_neighbors:
            stack.pop()
            continue

        nx, ny = random.choice(unvisited_neighbors)
        visited[nx][ny] = True
        visited_count += 1

        should_block = random.random() < 0.3 and (nx, ny) not in (start, goal)
        maze[nx][ny] = 1 if should_block else 0
        if not should_block:
            stack.append((nx, ny))

    maze[start[0]][start[1]] = 0
    maze[goal[0]][goal[1]] = 0
    return maze


def main():
    parser = argparse.ArgumentParser(description="Generate random mazes as JSON")
    parser.add_argument("--num_mazes", type=int, default=50,
                        help="Number of mazes to generate")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed for reproducibility")
    parser.add_argument("--output", type=str, default="mazes.json",
                        help="Output JSON file path")
    args = parser.parse_args()

    random.seed(args.seed)

    mazes = []
    for _ in tqdm(range(args.num_mazes), desc="Generating mazes"):
        mazes.append(create_maze())

    with open(args.output, "w") as fp:
        json.dump(mazes, fp)
    print(f"Generated {args.num_mazes} mazes (seed={args.seed}) -> {args.output}")

if __name__ == "__main__":
    main()
