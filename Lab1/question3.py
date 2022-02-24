from heuristic import *
from queue import PriorityQueue

class question_three:
    
    # solve using the dijkstra algorithm
    def __init__(self, coord, dist, g, cost):
        print("======================== QUESTION 3 ========================\n")
        self.coord = coord
        self.dist = dist
        self.g = g
        self.travel_cost = cost

    def solve(self, start, target, budget, heuristic_function):

        function = heuristics()

        # initialize the queue
        q = PriorityQueue()

        # initialize the distance to the start node
        distance = {x :float('inf') for x in range(1, 264347)}

        # initialize the heuristics to the start node
        optimization_values = {x :float('inf') for x in range(1, 264347)}

        # initialize the parent node
        parent = {}

        # initialize the visited nodes
        visited = {x: False for x in range(1, 264347)}

        # initialize the energy spent to reach node
        cost = {}

        optimization_values[start] = 0
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

                # Distance travelled + projected distance to target
                vertex_distance = (distance[current] + self.dist[str(current) + "," + str(vertex)])
                projected_distance = 0

                if(heuristic_function == "coordinate"):
                    projected_distance = function.coordinate_distance(self.coord[str(vertex)], self.coord[str(target)])
                elif(heuristic_function == "manhattan"):
                    projected_distance = function.manhattan_distance(self.coord[str(vertex)], self.coord[str(target)])
                else:
                    projected_distance = function.chebyshev_distance(self.coord[str(vertex)], self.coord[str(target)])
                
                # THe actual a* function
                a = projected_distance + vertex_distance

                if((not visited[vertex] or optimization_values[vertex] > a) and path_cost <= budget):

                    optimization_values[vertex] = a

                    distance[vertex] = distance[current] + self.dist[str(current) + "," + str(vertex)]

                    parent[vertex] = current

                    q.put((optimization_values[vertex], vertex))

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