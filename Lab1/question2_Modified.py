import queue


class question_two:
    """Task two solves the NYC instance, with an energy constraint imposed.
    
    This class exists to search for the shortest distance between a start and goal node in a given graph, given a constraint in place.
    
    For this task, the modified version of Uniform-Cost Search (UCS) algorithm is used with custom optimizations.
    """
    def __init__(self, coord, cost, dist, g):
        """Constructor that initializes the essential variables for instances of the question_two class.

        Args:
            coord (dict of str: list of int): JSON file containing every vertex and its corresponding x and y coordinates.
            cost (dict of str: int): JSON file containing the energy cost between two nodes.
            dist (dict of str: int): JSON file containing the distance between two nodes.
            g (dict of str: list of str): JSON file containing every vertex, and the node(s) the vertex is connected to if any.
        """
        print("======================== QUESTION 2 (Modified) ========================\n")
        self.priority_queue = queue.PriorityQueue()
        self.coord = coord
        self.cost = cost
        self.dist = dist
        self.g = g

    def reconstruct_path(self, explored, energy_cost, currentNode):
        """Reconstructs the path from a given dictionary of explored nodes.

        This is done by backtracking from the goal node by checking its successive parents, up till the start node is reached.

        Args:
            explored (dict of (int, str): int): Dictionary of explored nodes, to reconstruct a path from.
            energy_cost (int): Energy cost of reaching the node being passed in to start from, denoted by the currentNode parameter.
            currentNode (str): Node being passed in to start backtracking the path from.

        Returns:
            route (str): Path from the start node to goal node, with nodes separated by "->".
        """
        var = (energy_cost, currentNode) # Used as the key to query the explored dictionary
        path = []                        # Initialize the path array

        # Stop when backtracking reaches the start node, or winds up empty
        while explored is not None:
            # Append the node into the path
            currentNode = var[1]
            path.append(currentNode)

            # Terminating condition, as start node will always be "1"
            if currentNode == '1':
                break

            # Update var by looking for its parent in the explored dictionary
            var = explored[var]

        # Reverse the path, and format the route as a prettified string
        route = path.pop()
        while len(path):
            route += "->" + path.pop()
        return route

    def uniform_cost_search(self, start, goal, budget):
        """Performs the Uniform-Cost Search (UCS) algorithm to find the shortest path between two nodes in a given graph.

        This implementation is for Task Two, and considers an energy cost constraint when looking for the shortest distance.

        Note: Python's default queue module is employed here, to facilitate the Priority Queue where nodes with distance as the priority.
        Note: This implementation adds in additional optimizations that finds a more optimal answer as compared to the pure UCS implementation. 

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

        # Enqueue the starting node with highest priority (0 distance)
        frontier.put((0, (0, start)))

        # Initialize optimization dictionaries for energy and distance, to reduce runtime and memory usage
        min_energy = {}
        min_distance = {}
        
        # Initialize count variable to track the total number of processed nodes
        counts = 0

        while not frontier.empty():
            counts += 1 # A new node is being processed

            # Get the next node with highest priority from the priority queue
            distance, (energy_cost, currentNode) = frontier.get()
            currentNode = str(currentNode)

            # If the goal node is reached, we will end the algorithm and reconstruct the path from start to goal nodes, and return the relevant values
            if currentNode == str(goal):
                route = self.reconstruct_path(
                    explored, energy_cost, currentNode)
                return distance, energy_cost, route, counts

            ''' If the current node is already in the optimization dictionaries, 
                but does not bring a lower distance/energy cost than the minimal found, 
                there is no point exploring it. Skip this node.
            '''
            if (currentNode in min_distance) and (currentNode in min_energy):
                if (distance >= min_distance[currentNode]) and (energy_cost >= min_energy[currentNode]):
                    continue

            ''' If this current node has not been recorded in either optimization dictionaries,
                or brings a new lower distance / energy cost to the table,
                we are interested to explore this node since it may yield a potentially better result.
                Add this node into the dictionaries to record the new minimum/node, 
                and we will explore this node along with its adjacent nodes.
            '''
            if (currentNode not in min_distance) or (distance < min_distance[currentNode]):
                min_distance[currentNode] = distance
            if currentNode not in min_energy or (energy_cost < min_energy[currentNode]):
                min_energy[currentNode] = energy_cost

            # For the current node, check each of its adjacent nodes using the dictionary g
            for nextNode in self.g[currentNode]:
                nextNode = str(nextNode)

                # Compute the new distance and energy cost for this adjacent node, based on the current node
                newDistance = distance + self.dist[currentNode + "," + nextNode]
                newCost = energy_cost + self.cost[currentNode + "," + nextNode]

                # Create the cost/node pair, to record this unique permutation being explored
                storage = (newCost, nextNode)

                # Enqueue this adjacent node if the new distance to it is lower, or this permutation of cost/node has never been explored. 
                # The total cost must be lower than the budget given.
                if ((storage not in explored) or (newDistance <= path_distance[nextNode])) and (newCost <= budget):
                    # Priorities of nodes are based on the cumulative distances to them
                    priority = newDistance

                    # Put this adjacent cost/node combination into the priority queue
                    frontier.put((priority, storage))

                    # Assign current node as parent to this energy/adjacent node permutation
                    explored[storage] = (energy_cost, currentNode)

                    # Keep track of the updated path distance and cost to this adjacent node
                    path_distance[nextNode] = newDistance
                    path_energy[nextNode] = newCost

        return None, None, None
