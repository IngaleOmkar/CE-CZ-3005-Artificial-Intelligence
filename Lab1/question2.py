from queue import PriorityQueue

class question_two:
    
    # solve using the dijkstra algorithm
    def __init__(self, coord, dist, g, cost):
        print("======================== QUESTION 2 ========================\n")
        self.coord = coord
        self.dist = dist
        self.g = g
        self.travel_cost = cost

    def solve(self, start, target, budget):

        # initialize the queue
        q = PriorityQueue()

        # initialize the distance to the start node
        distance = {x :float('inf') for x in range(1, 264347)}

        # initialize the parent node
        parent = {}

        # initialize the visited nodes
        visited = {x: False for x in range(1, 264347)}

        # initialize the energy spent to reach node
        cost = {}

        distance[start] = 0
        parent[start] = None
        visited[start] = True
        cost[start] = 0

        # add the start node to the queue
        q.put((0, start))

        # while the queue is not empty
        while not q.empty():

            current = q.get()[1]

            if(current == target):
                break

            for vertex in self.g[str(current)]:

                vertex = int(vertex)

                path_cost = cost[current] + self.travel_cost[str(current) + "," + str(vertex)]

                if((not visited[vertex] or distance[vertex] > distance[current] + self.dist[str(current) + "," + str(vertex)]) and path_cost <= budget):

                    distance[vertex] = distance[current] + self.dist[str(current) + "," + str(vertex)]

                    parent[vertex] = current

                    q.put((distance[vertex], vertex))

                    visited[vertex] = True

                    cost[vertex] = path_cost

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
        print("Energy: ", cost[target])