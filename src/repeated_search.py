
import time
from src.result import Result
from src.utils import get_start_and_goal
from src.utils import a_star, manhattan


# Uses repeated A* search to find a path from start to goal in the given maze.
def search(maze, start=None, goal=None, prefer_smaller_g_tie_break=False):
    if not maze or not maze[0]:
        return Result(
            maze=maze,
            path=[],
            explored_nodes=[],
            total_expanded=0,
            time_spent=0.0,
        )

    start_time = time.perf_counter()
    maze_start, maze_goal = get_start_and_goal(maze)

    if start is None:
        start = maze_start
    if goal is None:
        goal = maze_goal

    if start is None or goal is None:
        return Result(
            maze=maze,
            path=[],
            explored_nodes=[],
            total_expanded=0,
            time_spent=0.0,
        )

    rows = len(maze)
    cols = len(maze[0])

    h = [[0] * cols for _ in range(rows)]
    for row in range(rows):
        for col in range(cols):
            h[row][col] = manhattan((row, col), goal)

    g_score = [[0] * cols for _ in range(rows)]
    parent = [[None] * cols for _ in range(rows)]
    closed = [[False] * cols for _ in range(rows)]

    known_blocked = set()
    explored_nodes = []
    total_expanded = 0
    path_taken = [start]

    current = start
    while current != goal:
        planned_path, expanded = a_star(
            current,
            goal,
            known_blocked,
            h,
            g_score,
            parent,
            closed,
            prefer_smaller_g_tie_break=prefer_smaller_g_tie_break,
        )

        explored_nodes.extend(expanded)
        total_expanded += len(expanded)
        if planned_path is None:
            break

        blocked_found = False
        for step in planned_path[1:]:
            step_row, step_col = step
            if maze[step_row][step_col] == 1:
                known_blocked.add(step)
                blocked_found = True
                break
            current = step
            path_taken.append(step)
            if current == goal:
                break

        if not blocked_found and current == goal:
            break

    elapsed = time.perf_counter() - start_time
    return Result(
        maze=maze,
        path=path_taken,
        explored_nodes=explored_nodes,
        total_expanded=total_expanded,
        time_spent=elapsed,
    )
    
