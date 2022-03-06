import queue


class question_one:
    """Task one solves a relaxed version of the NYC instance (no energy constraint).
    
    This class exists to search for the shortest distance between a start and goal node in a given graph.
    
    For this task, the Uniform-Cost Search (UCS) algorithm is used, without a constraint.
    """
    def __init__(self, coord, dist, g):
        """Constructor that initializes the essential variables for instances of the question_one class.

        Args:
            coord (dict of str: list of int): JSON file containing every vertex and its corresponding x and y coordinates.
            dist (dict of str: int): JSON file containing the distance between each vertex, and its adjacent nodes if any.
            g (dict of str: list of str): JSON file containing every vertex, and the node(s) the vertex is connected to if any.
        """
        print("======================== QUESTION 1 ========================\n")
        self.priority_queue = queue.PriorityQueue()
        self.coord = coord
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

    def uniform_cost_search(self, start, goal):
        """Performs the Uniform-Cost Search (UCS) algorithm to find the shortest path between two nodes in a given graph.

        This implementation is for Task One, and hence does not consider an energy cost constraint when looking for the shortest distance.

        Note: Python's default queue module is employed here, to facilitate the Priority Queue where nodes with distance as the priority.

        Args:
            start (int): Starting node within the graph.
            goal (int): Target node we want to reach within the shortest distance.

        Returns:
            distance (int): Shortest distance found between the start and goal nodes given, using the algorithm.
            route (str): Path from the start node to goal node, with nodes separated by "->".
            counts (int): Number of nodes visited in total during the algorithm's runtime.
        """

        # Initialize start and goal nodes by casting to string
        start = str(start)
        goal = str(goal)

        # Initialize the dictionary of explored nodes {node : parentNode}. The start node has no parent node
        explored = {start: None}

        # Initialize the dictionary of costs of travelling to a node from the start node. There is zero cost involved for the start node
        pathcost = {start: 0}

        # Initialize the priority queue using Python's queue module
        frontier = self.priority_queue

        # Enqueue the starting node with highest priority (0)
        frontier.put((0, start))
        
        # Initialize count variable to track the total number of processed nodes
        counts = 0

        while not frontier.empty():
            counts += 1 # A new node is being processed

            # Get the next node with highest priority from the priority queue
            distance, currentNode = frontier.get()
            currentNode = str(currentNode)

            # If the goal node is reached, we will end the algorithm and reconstruct the path from start to goal node, and return the relevant values
            if currentNode == str(goal):
                route = self.reconstruct_path(explored, start, goal)
                return distance, route, counts

            # For the current node, check each of its adjacent nodes using the dictionary g
            for nextNode in self.g[currentNode]:
                nextNode = str(nextNode)

                # Compute the new cost for this adjacent node, based on the current node
                newcost = distance + self.dist[currentNode + "," + nextNode]

                # Enqueue this adjacent node if the new total cost to it is lower, or is has never been explored
                if (nextNode not in explored) or (newcost < pathcost[nextNode]):
                    # Priorities of nodes are based on the cumulative distances to them
                    priority = newcost

                    # Put this adjacent node into the priority queue
                    frontier.put((priority, nextNode))

                    # Assign current node as parent to this adjacent node
                    explored[nextNode] = currentNode

                    # Keep track of the updated path cost to this adjacent node
                    pathcost[nextNode] = newcost

        return None, None