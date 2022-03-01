# Question one of this assignment will be solved using the UCS algorithm.

import queue


class question_one:
    def __init__(self, coord, dist, g):
        print("======================== QUESTION 1 ========================\n")
        self.priority_queue = queue.PriorityQueue()
        self.coord = coord
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
    def uniform_cost_search(self, start, goal):
        ''' Function to perform UCS to find path in a graph
            Input  : Graph with the start and goal vertices
            Output : Dict of explored vertices in the graph
        '''

        # initialization
        start = str(start)
        goal = str(goal)

        # Dict of explored nodes {node : parentNode}, start node has no parent node
        explored = {start: None}
        # Dict of cost from start to node, start cost is zero
        pathcost = {start: 0}
        # Priority Queue for Frontier
        frontier = self.priority_queue
        # Add the start node to frontier
        frontier.put((0, start))
        
        # Number of processed nodes
        counts = 0

        while not frontier.empty():
            counts += 1

            # Get next node from frontier
            distance, currentNode = frontier.get()
            currentNode = str(currentNode)

            # Stop when goal is reached
            if currentNode == str(goal):
                route = self.reconstruct_path(explored, start, goal)
                return distance, route, counts

            # Explore every single neighbor of current node
            for nextNode in self.g[currentNode]:
                nextNode = str(nextNode)
                # compute the new cost for the node based on the current node
                newcost = distance + self.dist[currentNode + "," + nextNode]

                # consider if not yet explored or if the new cost is lower
                if (nextNode not in explored) or (newcost < pathcost[nextNode]):

                    # set priority as newcost
                    priority = newcost

                    # put new node in frontier with priority
                    frontier.put((priority, nextNode))

                    # assign current node as parent
                    explored[nextNode] = currentNode

                    # keep track of the updated path cost
                    pathcost[nextNode] = newcost

        return None, None