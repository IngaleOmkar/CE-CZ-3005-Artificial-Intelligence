

class Heap:
    """
    A class used to implement a min binary heap.

    Methods
    -------
    insert(val=int, dist=int)
        Inserts a value into the heap. 
    remove()
        Removes the source from the heap, and does in-place rearrangement of the heap.
    peek()
        Returns the root node (source) of the heap, if the heap has at least 1 element.
    size()
        Returns the number of elements in the heap.
    is_empty()
        Checks whether the heap is empty, and returns the answer.
    __bubble_up(index=int)
        Helper function to recursively sort an element at a target index in the heap, upwards.
    __bubble_down(index=int)
        Helper function to recursively sort an element at a target index in the heap, downwards.
    """

    # The heap will be represented as a list.
    def __init__(self) -> None:
        self.heap = []
    
    def insert(self, val: int, dist: int) -> None:
        """Inserts a value into the heap. 

        The function will use a helper function: __bubble_up to do in-place sorting for the inserted element.

        Parameters
        ----------
        val : int
            The value of the element to be inserted into the minimizing heap.

        dist : int
            The current distance from the source to this inserted vertex (labelled by val).
        """

        self.heap.append([val, dist])
        self.__bubble_up(len(self.heap) - 1)
    
    def remove(self) -> int:
        """Removes the source from the heap, and does in-place rearrangement of the heap. 
        
        Returns this source element that was removed, or an Exception.
        
        Raises
        ------
        Exception
            If the heap is empty.
        """

        if self.is_empty():
            raise Exception("Heap is empty")
        
        node = self.heap[0]
        # Bring the last element in the heap to the top, then sort in-place downwards.
        self.heap[0] = self.heap[-1]
        self.heap.pop()
        self.__bubble_down(0)

        return node[0]
    
    def peek(self) -> int:
        """Returns the root node (source) of the heap, if the heap has at least 1 element.
        
        Raises
        ------
        Exception
            If the heap is empty.
        """

        if self.is_empty():
            raise Exception("Heap is empty")
        return self.heap[0][0]
    
    def size(self) -> int:
        """Returns the number of elements in the heap."""

        return len(self.heap)

    def is_empty(self) -> bool:
        """Checks whether the heap is empty, and returns the answer."""

        return len(self.heap) == 0
    
    def __bubble_up(self, index: int) -> None:
        """Helper function to recursively sort an element at a target index in the heap, upwards.
        
        This function is made for a minimizing heap, and compares the element with its parent to check if the distance is smaller.

        Parameters
        ----------
        index : int
            The index of the element to be bubble sorted upwards, in the minimizing heap.
        """

        parent = (index - 1) // 2
        if parent >= 0 and self.heap[index][1] < self.heap[parent][1]:
            self.heap[index], self.heap[parent] = self.heap[parent], self.heap[index]
            self.__bubble_up(parent)
    
    def __bubble_down(self, index: int) -> None:
        """Helper function to recursively sort an element at a target index in the heap, downwards (based on distance).
        
        Parameters
        ----------
        index : int
            The index of the element to be bubble sorted downwards, in the minimizing heap.
        """

        left = index * 2 + 1
        right = index * 2 + 2
        if left < len(self.heap) and self.heap[index][1] > self.heap[left][1]:
            self.heap[index], self.heap[left] = self.heap[left], self.heap[index]
            self.__bubble_down(left)
        if right < len(self.heap) and self.heap[index][1] > self.heap[right][1]:
            self.heap[index], self.heap[right] = self.heap[right], self.heap[index]
            self.__bubble_down(right)