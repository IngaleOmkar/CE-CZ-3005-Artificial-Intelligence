# Question three of this assignment will be solved using the UCS algorithm.

import queue


class question_three:
    def __init__(self, coord, cost, dist, g):
        print("======================== QUESTION 3 ========================\n")
        self.priority_queue = queue.PriorityQueue()
        self.coord = coord
        self.cost = cost
        self.dist = dist
        self.g = g

    # Reconstruct the path from the Dict of explored nodes {node : parentNode}
    # Intuition : Backtrack from the goal node by checking successive parents

    def reconstruct_path(self, explored, energy_cost, currentNode):
        var = (energy_cost, currentNode)  # Convert to dataformat for explored
        path = []                        # Initiate the blank path

        # stop when backtrack when explored is empty or if it reaches start node
        while explored is not None:

            # grow the path backwards and backtrack
            currentNode = var[1]
            path.append(currentNode)

            # Terminating case
            if currentNode == '1':
                break

            # Update to the next node
            var = explored[var]

        # reverse the path and get the formated route
        route = path.pop()
        while len(path):
            route += "->" + path.pop()
        return route

    # Heuristic function for Distance (Pythagoras)

    def heuristic(self, nodeA, nodeB):
        (xA, yA) = self.coord[nodeA]
        (xB, yB) = self.coord[nodeB]
        distance = ((xA - xB)**2 + (yA - yB)**2)**0.5
        return distance

    # A*-Search (A*S) with Priority Queue

    def astar_search(self, start, goal, budget):
        ''' Function to perform astar search to find path in a graph
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
        frontier.put((0, 0, (0, start)))

        # Dict to reduce run time and memory usage for iteration
        min_energy = {}
        min_distance = {}

        while not frontier.empty():
            # Get next node from frontier
            priority, distance, (energy_cost, currentNode) = frontier.get()
            currentNode = str(currentNode)

            # Stop when goal is reached
            if currentNode == str(goal):
                route = self.reconstruct_path(
                    explored, energy_cost, currentNode)
                return distance, energy_cost, route

            ################# Efficiency in skipping ########################
            if (currentNode in min_distance) and (currentNode in min_energy):
                if (distance >= min_distance[currentNode]) and (energy_cost >= min_energy[currentNode]):
                    continue  # Skip this node
            ################# Data dict for efficiency #####################
            if (currentNode not in min_distance) or (distance < min_distance[currentNode]):
                min_distance[currentNode] = distance
            if currentNode not in min_energy or (energy_cost < min_energy[currentNode]):
                min_energy[currentNode] = energy_cost
            ################################################################

            # Explore every single neighbor of current node
            for nextNode in self.g[currentNode]:
                nextNode = str(nextNode)

                # compute the new cost for the node based on the current node
                newDistance = distance + \
                    self.dist[currentNode + "," + nextNode]
                newCost = energy_cost + self.cost[currentNode + "," + nextNode]

                # Storage
                storage = (newCost, nextNode)

                # consider if not yet explored or if the new distance is lower
                if ((storage not in explored) or (newDistance <= path_distance[nextNode])) and (newCost <= budget):

                    # set priority as newcost + distance from goal
                    priority = newDistance + self.heuristic(nextNode, goal)

                    # put new node in frontier with priority
                    frontier.put((priority, newDistance, storage))

                    # assign current node as parent
                    explored[storage] = (energy_cost, currentNode)

                    # keep track of the updated path cost
                    path_distance[nextNode] = newDistance
                    path_energy[nextNode] = newCost

        return None, None, None
