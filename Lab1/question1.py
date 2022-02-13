# Question one of this assignment will be solved using the UCS algorithm. 

from binary_heap import heap

class question_one:
    
    # solve using the dijkstra algorithm
    def __init__(self, coord, dist, g):
        print("======================== QUESTION 1 ========================\n")
        self.priority_queue = heap()
        self.coord = coord
        self.dist = dist
        self.g = g

    def solve(self, start, target):
        visited = [0] * len(self.coord)
        visited_from = [-1] * len(self.coord)
        distance = [float('inf')] * len(self.coord)
        
        self.priority_queue.insert(start)
        visited[start] = 1
        distance[start] = 0
        visited_from[start] = -2

        while not self.priority_queue.is_empty():
            current = self.priority_queue.remove()
            if current == target:
                break
            for vertex in self.g[str(current)]:
                vertex = int(vertex)
                if(visited[vertex] == 0 and distance[vertex] > distance[current] + self.dist[str(current) + "," + str(vertex)]):
                    distance[vertex] = distance[current] + self.dist[str(current) + "," + str(vertex)]
                    visited_from[vertex] = current
                    self.priority_queue.insert(vertex)
                    visited[vertex] = 1
            
        # retrace the path of the target
        destination = target 

        path = [target]
        while visited_from[target] != -2:
            target = visited_from[target]
            path.append(target)
        path.reverse()

        print("Shortest Path:", end=" ")
        for index in range(len(path)-1):
            print(path[index], end = "->")
        print(str(path[-1]) + ".")

        print("\nDistance:", distance[destination])