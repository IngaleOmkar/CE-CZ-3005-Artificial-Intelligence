# Create a python class wih a data structure of a min binary heap.
# The class will have the following methods:
# - insert(val)
# - remove()
# - peek()
# - size()
# - is_empty()

class heap:

    # The heap will be represented as a list.
    def __init__(self) -> None:
        self.heap = []
    
    # This function is to insert a value into the heap. 
    # The function will use a helper function: __bubble_up
    def insert(self, val: int) -> None:
        self.heap.append(val)
        self.__bubble_up(len(self.heap) - 1)
    
    def remove(self) -> int:
        if self.is_empty():
            raise Exception("Heap is empty")
        val = self.heap[0]
        self.heap[0] = self.heap[-1]
        self.heap.pop()
        self.__bubble_down(0)
        return val
    
    def peek(self) -> int:
        if self.is_empty():
            raise Exception("Heap is empty")
        return self.heap[0]
    
    def size(self) -> int:
        return len(self.heap)
    
    def is_empty(self) -> bool:
        return len(self.heap) == 0
    
    def __bubble_up(self, index: int) -> None:
        parent = (index - 1) // 2
        if parent >= 0 and self.heap[index] < self.heap[parent]:
            self.heap[index], self.heap[parent] = self.heap[parent], self.heap[index]
            self.__bubble_up(parent)
    
    def __bubble_down(self, index: int) -> None:
        left = index * 2 + 1
        right = index * 2 + 2
        if left < len(self.heap) and self.heap[index] > self.heap[left]:
            self.heap[index], self.heap[left] = self.heap[left], self.heap[index]
            self.__bubble_down(left)
        if right < len(self.heap) and self.heap[index] > self.heap[right]:
            self.heap[index], self.heap[right] = self.heap[right], self.heap[index]
            self.__bubble_down(right)
        