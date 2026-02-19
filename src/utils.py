import random

from src.binary_heap import BinaryHeap


def get_start_and_goal(maze):
    start = None
    goal = None
    for row_idx, row in enumerate(maze):
        for col_idx, cell in enumerate(row):
            if cell == "A":
                start = (row_idx, col_idx)
            elif cell == "T":
                goal = (row_idx, col_idx)
    return start, goal


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


def reconstruct_path(parent, start, goal):
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


def a_star(
    start,
    goal,
    known_blocked,
    h,
    g_score,
    parent,
    closed,
    prefer_smaller_g_tie_break=False,
):
    rows = len(h)
    cols = len(h[0])

    for row in range(rows):
        for col in range(cols):
            g_score[row][col] = None
            parent[row][col] = None
            closed[row][col] = False

    g_score[start[0]][start[1]] = 0

    open_heap = BinaryHeap()
    f_start = h[start[0]][start[1]]
    open_heap.push((f_start, 0, random.random(), start))

    expanded = []
    while open_heap:
        f_score, _, _, current = open_heap.pop()
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
                tie_break = tentative_score if prefer_smaller_g_tie_break else -tentative_score
                open_heap.push((f_score, tie_break, random.random(), neighbor))

    if g_score[goal[0]][goal[1]] is None:
        return None, expanded

    path = reconstruct_path(parent, start, goal)
    return path, expanded
