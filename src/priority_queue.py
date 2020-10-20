import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def insert(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def update(self, item, priority):
        self.delete(item)
        heapq.heappush(self.elements, (priority, item))

    def top(self):
        item = heapq.heappop(self.elements)
        return item[1]

    def topKey(self):
        return heapq.nsmallest(1, self.elements)[0][0]

    def delete(self, node):
        self.elements = [e for e in self.elements if e[1] is not node]
        heapq.heapify(self.elements)

    def __iter__(self):
        for key, node in self.elements:
            yield node
