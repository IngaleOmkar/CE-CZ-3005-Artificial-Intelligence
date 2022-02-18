# Question one of this assignment will be solved using the UCS algorithm.

import queue


class question_two:
    def __init__(self, coord, cost, dist, g):
        print("======================== QUESTION 2 ========================\n")
        self.priority_queue = queue.PriorityQueue()
        self.coord = coord
        self.cost = cost
        self.dist = dist
        self.g = g

    # Reconstruct the path from the Dict of explored nodes {node : parentNode}
    # Intuition : Backtrack from the goal node by checking successive parents

    def reconstruct_path(self, explored, start, goal):
        start = str(start)             # Convert to string to match data type
        goal = str(goal)               # Convert to string to match data type
        currentNode = goal             # start at the goal node
        path = []                      # initiate the blank path

        # stop when backtrack reaches start node
        while currentNode != start:
            # grow the path backwards and backtrack
            path.append(currentNode)
            currentNode = explored[currentNode]

        path.append(start)             # append start node for completeness

        # reverse the path and get the formated route
        route = path.pop()
        while len(path):
            route += "->" + path.pop()
        return route

    # Uniform-Cost-Search (UCS) with Priority Queue
    def uniform_cost_search(self, start, goal, budget):
        ''' Function to perform UCS to find path in a graph
            Input  : Graph with the start and goal vertices
            Output : Dict of explored vertices in the graph
        '''

        # initialization
        start = str(start)
        goal = str(goal)

        # Dict of explored nodes {node : parentNode}, start node has no parent node
        explored = {(0, start): None}
        # Dict of distance cost from start to node, start cost is zero
        path_distance = {start: 0}
        # Dict of energy cost
        path_energy = {start: 0}
        # Priority Queue for Frontier
        frontier = self.priority_queue
        # Add the start node to frontier
        frontier.put((0, (0, start)))

        while not frontier.empty():
            # Get next node from frontier
            distance, (energy_cost, currentNode) = frontier.get()
            currentNode = str(currentNode)

            # Stop when goal is reached
            if currentNode == str(goal):
                route = self.reconstruct_path(explored, start, goal)
                return distance, energy_cost, route

            # Explore every single neighbor of current node
            for nextNode in self.g[currentNode]:
                nextNode = str(nextNode)

                # compute the new cost for the node based on the current node
                newDistance = distance + self.dist[currentNode + "," + nextNode]
                newCost = energy_cost + self.cost[currentNode + "," + nextNode]
                
                # consider if not yet explored or if the new distance is lower
                if ((nextNode not in explored) or (newDistance <= path_distance[nextNode])) and (newCost <= budget):

                    # set priority as newDistance
                    priority = newDistance

                    # put new node in frontier with priority
                    frontier.put((priority, (newCost, nextNode)))

                    # assign current node as parent
                    explored[nextNode] = currentNode

                    # keep track of the updated path cost
                    path_distance[nextNode] = newDistance
                    path_energy[nextNode] = newCost

        return None, None, None