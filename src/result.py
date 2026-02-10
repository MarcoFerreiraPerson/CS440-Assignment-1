
class Result:
    def __init__(self, maze, path, explored_nodes, time_spent):
        self.maze = maze
        self.path = path
        self.explored_nodes = explored_nodes
        self.time_spent = time_spent