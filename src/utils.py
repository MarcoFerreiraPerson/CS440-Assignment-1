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


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def neighbors(cell, rows, cols):
    rows, cols = cell
    if rows > 0:
        yield (rows - 1, cols)
    if rows + 1 < rows:
        yield (rows + 1, cols)
    if cols > 0:
        yield (rows, cols - 1)
    if cols + 1 < cols:
        yield (rows, cols + 1)