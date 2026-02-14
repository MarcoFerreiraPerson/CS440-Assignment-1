class BinaryHeap:
    def __init__(self):
        self.data = []

    def __len__(self):
        return len(self.data)

    def push(self, item):
        self.data.append(item)
        self._up(len(self.data) - 1)

    def pop(self):
        if not self.data:
            raise IndexError("pop from empty heap")

        self.data[0], self.data[-1] = self.data[-1], self.data[0]
        smallest = self.data.pop()
        if self.data:
            self._down(0)
        return smallest

    def _up(self, idx):
        while idx > 0:
            parent = (idx - 1) // 2
            if self.data[parent] <= self.data[idx]:
                break
            self.data[parent], self.data[idx] = self.data[idx], self.data[parent]
            idx = parent

    def _down(self, idx):
        size = len(self.data)
        while True:
            left = 2 * idx + 1
            if left >= size:
                break

            right = left + 1
            child = left
            if right < size and self.data[right] < self.data[left]:
                child = right
            if self.data[idx] <= self.data[child]:
                break

            self.data[idx], self.data[child] = self.data[child], self.data[idx]
            idx = child
