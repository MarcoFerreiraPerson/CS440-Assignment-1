
class Result:
    def __init__(self, maze, path, explored_nodes, total_expanded, time_spent):
        self.maze = maze
        self.path = path
        self.explored_nodes = explored_nodes
        self.total_expanded = total_expanded
        self.time_spent = time_spent
