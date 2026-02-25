

from __future__ import annotations

import random


class BinaryHeap:
    def __init__(self):
        self.data = []

    def __len__(self):
        return len(self.data)

    def __bool__(self):
        return len(self.data) > 0

    def push(self, item):
        self.data.append(item)
        self._up(len(self.data) - 1)

    def pop(self):
        self.data[0], self.data[-1] = self.data[-1], self.data[0]
        smallest = self.data.pop()
        if self.data:
            self._down(0)
        return smallest

    #private
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


class CustomPQ_maxG:

    def __init__(self):
        self._heap = BinaryHeap()

    def __len__(self):
        return len(self._heap)

    def __bool__(self):
        return bool(self._heap)

    def push(self, f, g, cell):
        self._heap.push((f, -g, random.random(), cell))

    def pop(self):
        f, neg_g, _, cell = self._heap.pop()
        return f, -neg_g, cell


class CustomPQ_minG:

    def __init__(self):
        self._heap = BinaryHeap()

    def __len__(self):
        return len(self._heap)

    def __bool__(self):
        return bool(self._heap)

    def push(self, f, g, cell):
        self._heap.push((f, g, random.random(), cell))

    def pop(self):
        f, g, _, cell = self._heap.pop()
        return f, g, cell
