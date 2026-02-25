"""
q2.py — Repeated Forward A* (Forward Replanning) with tie-breaking variants + Pygame visualization

Renders TWO views side-by-side:
- LEFT  : full (ground-truth) maze used for the run
- RIGHT : agent knowledge + search visualization

Controls:
- R : generate a new random maze and run again (max-g by default)
- 1 : run MAX-G on the current maze
- 2 : run MIN-G on the current maze
- L : load maze from text file (see readFile format) and run MAX-G
- ESC or close window : quit

Maze file format (readFile):
- Space-separated 0/1 values, 1 = blocked, 0 = free, one row per line.

Legend (colors):
GREY   = expanded / frontier / unknown (unseen)
PATH   = executed path
YELLOW = start + agent position
BLUE   = goal
WHITE  = known free
BLACK  = known blocked
"""

from __future__ import annotations

import heapq
import argparse
import json
import time
from typing import Callable, Dict, List, Optional, Tuple
from tqdm import tqdm
import pygame
import matplotlib.pyplot as plt
import numpy as np
from constants import ROWS, START_NODE, END_NODE, BLACK, WHITE, GREY, YELLOW, BLUE, PATH, NODE_LENGTH, GRID_LENGTH, WINDOW_W, WINDOW_H, GAP
from custom_pq import CustomPQ_maxG, CustomPQ_minG


def readMazes(fname: str) -> List[List[List[int]]]:
    """
    Reads a JSON file containing a list of mazes.
    Each maze is a list of ROWS lists, each with ROWS int values (0=free, 1=blocked).
    Returns a list of maze[r][c] grids.
    """
    with open(fname, "r", encoding="utf-8") as fp:
        data = json.load(fp)
    mazes: List[List[List[int]]] = []
    for idx, grid in enumerate(data):
        if len(grid) != ROWS or any(len(row) != ROWS for row in grid):
            raise ValueError(f"Maze {idx}: expected {ROWS}x{ROWS}, got {len(grid)}x{len(grid[0]) if grid else 0}")
        maze = [[int(v) for v in row] for row in grid]
        maze[START_NODE[0]][START_NODE[1]] = 0
        maze[END_NODE[0]][END_NODE[1]] = 0
        mazes.append(maze)
    return mazes


def repeated_forward_astar(
    actual_maze: List[List[int]],
    start: Tuple[int, int] = START_NODE,
    goal: Tuple[int, int] = END_NODE,
    tie_breaking: str = "max_g", # "min_g"
    visualize_callbacks: Optional[Dict[str, Callable[[Tuple[int, int]], None]]] = None,
    tie_braking: Optional[str] = None,
) -> Tuple[bool, List[Tuple[int, int]], int, int]:

    # TODO: Implement Repeated Forward A* with min_g & max_g tie-braking strategies.
    # Use heapq for standard priority queue implementation and name your max_g heap class as `CustomPQ_maxG`
    # and min_g heap class as `CustomPQ_minG`. Place them inside `custom_pq.py` file (see import statement in line 41).
    # and use it.

    if tie_braking is not None:
        tie_breaking = tie_braking

    if tie_breaking not in {"max_g", "min_g"}:
        raise ValueError("tie_breaking must be one of: max_g, min_g")

    if not actual_maze or not actual_maze[0]:
        return False, [], 0, 0

    rows = len(actual_maze)
    cols = len(actual_maze[0])

    def manhattan(first, second):
        return abs(first[0] - second[0]) + abs(first[1] - second[1])

    def neighbors(cell, max_rows, max_cols):
        row, col = cell
        if row > 0:
            yield (row - 1, col)
        if row + 1 < max_rows:
            yield (row + 1, col)
        if col > 0:
            yield (row, col - 1)
        if col + 1 < max_cols:
            yield (row, col + 1)

    def reconstruct_path(parent, s, g):
        path = []
        current = g
        while current is not None and current != s:
            path.append(current)
            current = parent.get(current)
        if current != s:
            return None
        path.append(s)
        path.reverse()
        return path

    def a_star(s, g, known_blocked, h, prefer_smaller_g_tie_break=False):
        g_score = {s: 0}
        parent = {s: None}
        closed = set()

        open_heap = CustomPQ_minG() if prefer_smaller_g_tie_break else CustomPQ_maxG()
        open_heap.push(float(h[s[0]][s[1]]), 0.0, s)

        expanded = []
        while open_heap:
            _, _, current = open_heap.pop()
            if current in closed:
                continue

            closed.add(current)
            expanded.append(current)

            if current == g:
                break

            for neighbor in neighbors(current, rows, cols):
                if neighbor in known_blocked or neighbor in closed:
                    continue

                tentative_score = g_score[current] + 1
                if tentative_score < g_score.get(neighbor, float("inf")):
                    g_score[neighbor] = tentative_score
                    parent[neighbor] = current
                    f_score = tentative_score + h[neighbor[0]][neighbor[1]]
                    open_heap.push(float(f_score), float(tentative_score), neighbor)

        if g not in g_score:
            return None, expanded, g_score

        return reconstruct_path(parent, s, g), expanded, g_score

    def observe_adjacent_blocked(maze, cell, known_blocked):
        for neighbor in neighbors(cell, rows, cols):
            if maze[neighbor[0]][neighbor[1]] == 1:
                known_blocked.add(neighbor)

    h = []
    for _ in range(rows):
        h.append([0] * cols)
    for row in range(rows):
        for col in range(cols):
            h[row][col] = manhattan((row, col), goal)

    prefer_smaller = tie_breaking == "min_g"
    known_blocked = set()
    total_expanded = 0
    replans = 0
    path_taken = [start]

    current = start
    observe_adjacent_blocked(actual_maze, current, known_blocked)

    while current != goal:
        replans += 1
        planned_path, expanded, _ = a_star(
            current,
            goal,
            known_blocked,
            h,
            prefer_smaller_g_tie_break=prefer_smaller,
        )

        total_expanded += len(expanded)
        if planned_path is None:
            return False, path_taken, total_expanded, replans

        blocked_found = False
        for step in planned_path[1:]:
            observe_adjacent_blocked(actual_maze, current, known_blocked)
            step_row, step_col = step
            if step in known_blocked or actual_maze[step_row][step_col] == 1:
                known_blocked.add(step)
                blocked_found = True
                break
            current = step
            path_taken.append(step)
            observe_adjacent_blocked(actual_maze, current, known_blocked)
            if current == goal:
                break

        if not blocked_found and current == goal:
            break

    return current == goal, path_taken, total_expanded, replans


def show_astar_search(win: pygame.Surface, actual_maze: List[List[int]], algo: str, fps: int = 240, step_delay_ms: int = 0, save_path: Optional[str] = None) -> None:
    # [BONUS] TODO: Place your visualization code here.
    # This function should display the maze used, the agent's knowledge, and the search process as the agent plans and executes.
    # As a reference, this function takes pygame Surface 'win' to draw on, the actual maze grid, the algorithm name for labeling,
    # and optional parameters for controlling the visualization speed and saving a screenshot.
    # You are free to use other visualization libraries other than pygame.
    # You can call repeated_forward_astar with visualize_callbacks that update the Pygame display as the agent plans and executes.
    # In the end it should store the visualization as a PNG file if save_path is provided, or default to "vis_{algo}.png".
    # print(f"[{algo}] found={found}  executed_steps={len(executed)-1}  expanded={expanded}  replans={replans}")


    if save_path is None:
        save_path = f"vis_{algo}.png"

    tie_breaking = "min_g" if algo.lower() == "min_g" else "max_g"
    found, executed, expanded, replans = repeated_forward_astar(
        actual_maze=actual_maze,
        start=START_NODE,
        goal=END_NODE,
        tie_breaking=tie_breaking,
    )

    def build_maze_image(maze, s, g, path=None):
        height = len(maze)
        width = len(maze[0])
        image_data = np.zeros((height, width, 3), dtype=np.uint8)

        colors = {
            0: WHITE,
            1: BLACK,
        }

        for row_idx, row in enumerate(maze):
            for col_idx, cell in enumerate(row):
                image_data[row_idx, col_idx] = colors.get(cell, WHITE)

        if path:
            for row_idx, col_idx in path:
                if 0 <= row_idx < height and 0 <= col_idx < width:
                    image_data[row_idx, col_idx] = PATH

        sr, sc = s
        gr, gc = g
        if 0 <= sr < height and 0 <= sc < width:
            image_data[sr, sc] = YELLOW
        if 0 <= gr < height and 0 <= gc < width:
            image_data[gr, gc] = BLUE

        return image_data

    image_base = build_maze_image(actual_maze, START_NODE, END_NODE)
    image_solved = build_maze_image(
        actual_maze,
        START_NODE,
        END_NODE,
        path=executed if found else None,
    )

    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    axes[0].imshow(image_base, interpolation="nearest")
    axes[0].set_title("Ground Truth Maze")
    axes[0].axis("off")

    path_length = len(executed) - 1 if found else -1
    axes[1].imshow(image_solved, interpolation="nearest")
    axes[1].set_title(
        f"{algo} | found={found}, path={path_length}, expanded={expanded}, replans={replans}"
    )
    axes[1].axis("off")

    fig.tight_layout()
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    print(f"Saved the visualization -> {save_path}")
    plt.show()
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Q2: Repeated Forward A*")
    parser.add_argument("--maze_file", type=str, required=True,
                        help="Path to input JSON file containing a list of mazes")
    parser.add_argument("--output", type=str, default="results_q2.json",
                        help="Path to output JSON results file")
    parser.add_argument("--tie_braking", type=str, choices=["max_g", "min_g", "both"], default="both",
                        help="Tie-breaking variant to run (default: both)")
    parser.add_argument("--show_vis", action="store_true",
                        help="[Bonus] If set, show Pygame visualization for the selected maze")
    parser.add_argument("--maze_vis_id", type=int, default=0,
                        help="[Bonus] maze_id (index) 0 ... 49 among 50 grid worlds")
    parser.add_argument("--save_vis_path", type=str, default="q2-vis-max-g.png",
                        help="[Bonus] If set, save visualization to this PNG file")
    args = parser.parse_args()

    mazes = readMazes(args.maze_file)
    results: List[Dict] = []

    for maze_id in tqdm(range(len(mazes)), desc="Processing mazes"):
        entry: Dict = {"maze_id": maze_id}

        if args.tie_braking in ("max_g", "both"):
            t0 = time.perf_counter()
            found, executed, expanded, replans = repeated_forward_astar(
                actual_maze=mazes[maze_id],
                start=START_NODE,
                goal=END_NODE,
                tie_breaking=args.tie_braking
            )
            t1 = time.perf_counter()

            entry["max_g"] = {
                "found": found,
                "path_length": len(executed) - 1 if found else -1,
                "expanded": expanded,
                "replans": replans,
                "runtime_ms": (t1 - t0) * 1000,
            }

        if args.tie_braking in ("min_g", "both"):
            t0 = time.perf_counter()
            found, executed, expanded, replans = repeated_forward_astar(
                actual_maze=mazes[maze_id],
                start=START_NODE,
                goal=END_NODE,
                tie_breaking=args.tie_braking
            )
            t1 = time.perf_counter()

            entry["min_g"] = {
                "found": found,
                "path_length": len(executed) - 1 if found else -1,
                "expanded": expanded,
                "replans": replans,
                "runtime_ms": (t1 - t0) * 1000,
            }

        results.append(entry)

    if args.show_vis:
        # In case, PyGame is used for visualization, this code initializes a window and runs the visualization for the selected maze and algorithm.
        # Feel free to modify this code if you use a different visualization library or approach.
        pygame.init()
        win = pygame.display.set_mode((WINDOW_W, WINDOW_H))
        pygame.display.set_caption("Repeated Forward A* Visualization")
        clock = pygame.time.Clock()
        selected_maze = mazes[args.maze_vis_id]
        current_algo = "max_g"
        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
        running = True
        while running:
            clock.tick(30)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        current_algo = "max_g"
                        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
                    elif event.key == pygame.K_1:
                        current_algo = "max_g"
                        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
                    elif event.key == pygame.K_2:
                        current_algo = "min_g"
                        show_astar_search(win, selected_maze, algo=current_algo, fps=240, step_delay_ms=0, save_path=args.save_vis_path)
            pygame.display.flip()

        pygame.quit()

    with open(args.output, "w") as fp:
        json.dump(results, fp, indent=2)
    print(f"Results for {len(results)} mazes written to {args.output}")


if __name__ == "__main__":
    main()
