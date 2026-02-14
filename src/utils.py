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
