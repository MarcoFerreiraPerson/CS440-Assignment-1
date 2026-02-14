
import time
from itertools import count

from src.binary_heap import BinaryHeap
from src.result import Result
from src.utils import get_start_and_goal

from src.utils import manhattan
from src.utils import neighbors


def _reconstruct_path(parent, start, goal):
    path = []
    current = goal
    while current is not None and current != start:
        path.append(current)
        row, col = current
        current = parent[row][col]
    if current != start:
        return None
    path.append(start)
    path.reverse()
    return path


def _a_star(start, goal, known_blocked, h, g_score, parent, closed):
    rows = len(h)
    cols = len(h[0])

    for row in range(rows):
        for col in range(cols):
            g_score[row][col] = None
            parent[row][col] = None
            closed[row][col] = False

    g_score[start[0]][start[1]] = 0

    open_heap = BinaryHeap()
    push_id = count()
    f_start = h[start[0]][start[1]]
    open_heap.push((f_start, 0, next(push_id), start))

    expanded = []
    while open_heap:
        f_score, neg_cost, _, current = open_heap.pop()
        row, col = current
        if closed[row][col]:
            continue
        closed[row][col] = True
        expanded.append(current)

        if current == goal:
            break

        for neighbor in neighbors(current, rows, cols):
            if neighbor in known_blocked:
                continue
            neighbor_row, neighbor_col = neighbor
            if closed[neighbor_row][neighbor_col]:
                continue
            tentative_score = g_score[row][col] + 1
            if g_score[neighbor_row][neighbor_col] is None or tentative_score < g_score[neighbor_row][neighbor_col]:
                g_score[neighbor_row][neighbor_col] = tentative_score
                parent[neighbor_row][neighbor_col] = current
                f_score = tentative_score + h[neighbor_row][neighbor_col]
                open_heap.push((f_score, -tentative_score, next(push_id), neighbor))

    if g_score[goal[0]][goal[1]] is None:
        return None, expanded

    path = _reconstruct_path(parent, start, goal)
    return path, expanded


# Uses adaptive A* search to find a path from the start to the goal in the given maze.
def search(maze):
    if not maze or not maze[0]:
        return Result(
            maze=maze,
            path=[],
            explored_nodes=[],
            total_expanded=0,
            time_spent=0.0,
        )

    start_time = time.perf_counter()
    start, goal = get_start_and_goal(maze)
    
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
        planned_path, expanded = _a_star(
            current,
            goal,
            known_blocked,
            h,
            g_score,
            parent,
            closed,
        )

        explored_nodes.extend(expanded)
        total_expanded += len(expanded)
        if planned_path is None:
            break

        goal_score = g_score[goal[0]][goal[1]]
        for cell in expanded:
            row, col = cell
            h[row][col] = goal_score - g_score[row][col]

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
    
