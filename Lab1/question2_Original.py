import queue


class question_two:
    """Task two solves the NYC instance, with an energy constraint imposed.
    
    This class exists to search for the shortest distance between a start and goal node in a given graph, given a constraint in place.
    
    For this task, the pure version of Uniform-Cost Search (UCS) algorithm is used, without any custom optimization.
    """
    def __init__(self, coord, cost, dist, g):
        """Constructor that initializes the essential variables for instances of the question_two class.

        Args:
            coord (dict of str: list of int): JSON file containing every vertex and its corresponding x and y coordinates.
            cost (dict of str: int): JSON file containing the energy cost between two nodes.
            dist (dict of str: int): JSON file containing the distance between two nodes.
            g (dict of str: list of str): JSON file containing every vertex, and the node(s) the vertex is connected to if any.
        """
        print("======================== QUESTION 2 (Original) ========================\n")
        self.priority_queue = queue.PriorityQueue()
        self.coord = coord
        self.cost = cost
        self.dist = dist
        self.g = g

    def reconstruct_path(self, explored, start, goal):
        """Reconstructs the path from a given dictionary of explored nodes.

        This is done by backtracking from the goal node by checking its successive parents, up till the start node is reached.

        Args:
            explored (dict of str: int): Dictionary of explored nodes, to reconstruct a path from.
            start (str): Starting node to begin path at, in the dictionary of explored nodes.
            goal (str): Target node where the path stops, in the dictionary of explored nodes.

        Returns:
            route (str): Path from the start node to goal node, with nodes separated by "->".
        """
        start = str(start)  # Convert to string to match data type
        goal = str(goal)    # Convert to string to match data type
        currentNode = goal  # Start at the goal node
        path = []           # Initialize the path array

        # Stop when backtracking reaches the start node
        while currentNode != start:
            # Append the node into the path, and then look for its parent in the explored dictionary
            path.append(currentNode)
            currentNode = explored[currentNode]

        path.append(start) # The start node must also be added in after the loop, to complete the path

        # Reverse the path, and format the route as a prettified string
        route = path.pop()
        while len(path):
            route += "->" + path.pop()
        return route

    def uniform_cost_search(self, start, goal, budget):
        """Performs the Uniform-Cost Search (UCS) algorithm to find the shortest path between two nodes in a given graph.

        This implementation is for Task Two, and considers an energy cost constraint when looking for the shortest distance.

        Note: Python's default queue module is employed here, to facilitate the Priority Queue where nodes with distance as the priority.

        Args:
            start (int): Starting node within the graph.
            goal (int): Target node we want to reach within the shortest distance.
            budget (int): Maximum energy cost that any path is allowed to incur, before the path cannot be considered as a solution.

        Returns:
            distance (int): Shortest distance found between the start and goal nodes given, using the algorithm.
            energy_cost (int): Total energy cost for the path yielding the shortest distance found.
                This energy cost must fall below the budget argument given.
            route (str): Path from the start node to goal node, with nodes separated by "->".
            counts (int): Number of nodes visited in total during the algorithm's runtime.
        """

        # Initialize start and goal nodes by casting to string
        start = str(start)
        goal = str(goal)

        # Initialize the dictionary of explored nodes {(cost, node) : parentNode}. The start node has no parent node
        explored = {(0, start): None}

        # Initialize the dictionary of distance to a node from the start node. There is zero cost involved for the start node
        path_distance = {start: 0}

        # Initialize the dictionary of energy costs incurred to get to a particular node. The start node has zero energy cost involved
        path_energy = {start: 0}

        # Initialize the priority queue using Python's queue module
        frontier = self.priority_queue

        # Enqueue the starting node with highest priority (0 cost)
        frontier.put((0, (0, start)))
        
        # Initialize count variable to track the total number of processed nodes
        counts = 0

        while not frontier.empty():
            counts += 1 # A new node is being processed

            # Get the next node with highest priority from the priority queue
            distance, (energy_cost, currentNode) = frontier.get()
            currentNode = str(currentNode)

            # If the goal node is reached, we will end the algorithm and reconstruct the path from start to goal nodes, and return the relevant values
            if currentNode == str(goal):
                route = self.reconstruct_path(explored, start, goal)
                return distance, energy_cost, route, counts

            # For the current node, check each of its adjacent nodes using the dictionary g
            for nextNode in self.g[currentNode]:
                nextNode = str(nextNode)

                # Compute the new distance and energy cost for this adjacent node, based on the current node
                newDistance = distance + self.dist[currentNode + "," + nextNode]
                newCost = energy_cost + self.cost[currentNode + "," + nextNode]

                # Enqueue this adjacent node if the new distance to it is lower, or is has never been explored. 
                # The total cost must be lower than the budget given.
                if ((nextNode not in explored) or (newDistance <= path_distance[nextNode])) and (newCost <= budget):
                    # Priorities of nodes are based on the cumulative distances to them
                    priority = newDistance

                    # Put this adjacent node into the priority queue
                    frontier.put((priority, (newCost, nextNode)))

                    # Assign current node as parent to this adjacent node
                    explored[nextNode] = currentNode

                    # Keep track of the updated path distance and cost to this adjacent node
                    path_distance[nextNode] = newDistance
                    path_energy[nextNode] = newCost

        return None, None, None 