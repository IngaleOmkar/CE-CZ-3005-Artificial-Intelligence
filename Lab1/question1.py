from queue import PriorityQueue

class question_one:
    
    # solve using the dijkstra algorithm
    def __init__(self, coord, dist, g):
        print("======================== QUESTION 1 ========================\n")
        self.coord = coord
        self.dist = dist
        self.g = g

    def solve(self, start, target):

        # initialize the queue
        q = PriorityQueue()

        # initialize the distance to the start node
        distance = {x :float('inf') for x in range(1, 264347)}

        # initialize the parent node
        parent = {}

        # initialize the visited nodes
        visited = {x: False for x in range(1, 264347)}

        distance[start] = 0
        parent[start] = None
        visited[start] = True

        # add the start node to the queue
        q.put((0, start))

        # while the queue is not empty
        while not q.empty():

            current = q.get()[1]

            if(current == target):
                break

            for vertex in self.g[str(current)]:

                vertex = int(vertex)

                if(not visited[vertex] or distance[vertex] > distance[current] + self.dist[str(current) + "," + str(vertex)]):

                    distance[vertex] = distance[current] + self.dist[str(current) + "," + str(vertex)]

                    parent[vertex] = current

                    q.put((distance[vertex], vertex))

                    visited[vertex] = True

        current = target
        path = []

        while(parent[current] != None):
                
            path.append(current)

            current = parent[current]

        path.append(start)

        path.reverse()

        for node in path:
            if(node == path[-1]):
                print(node)
            else:
                print(node, end = "->")
        
        print("Distance: ", distance[target])