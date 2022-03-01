from queue import PriorityQueue
from heuristic import *

class question_three:
    
    # solve using the dijkstra algorithm
    def __init__(self, coord, dist, g, cost):
        print("======================== QUESTION 2 ========================\n")
        self.coord = coord
        self.dist = dist
        self.g = g
        self.travel_cost = cost

    def solve(self, start, target, budget, heuristic_type):

        """Task one solves a relaxed version of the NYC instance (no energy constraint).
        
        Required Data
        -------------
        source : str
            Starting vertex.
        
        target : str
            Goal state, and the vertex we are trying to find the shortest distance to, from the source vertex.
        coord
            JSON file, to find the children of a node that is being visited.
        
        dist
            JSON file, to measure direct distances between connected nodes.
        g
            JSON file, to get the total number of nodes.
        cost
            JSON file, to measure the energy cost of travelling between connected nodes.
        energy_budget : int
            Maximum permissible energy that we are allowed to use to travel from the source node to target node. Used as the constraint in this problem to adhere to, while finding the minimum distance possible.
        """

        # We will find shortest path using the Uniform-Cost Search (UCS) Algorithm
        # UCS is very effective for graphs which are very large
        #* Note that UCS will consider path costs in making its decision, compared to BFS which must go through each level iteratively

        h = heuristics()

        # Initialize the priority queue
        priority_queue = PriorityQueue()

        # Push the source node into the heap, mark distance and cost as 0, and mark it as visited -> O(1)
        s = int(start)
        # The order is: f_n, vertex, cost, distance
        priority_queue.put((0, s, 0, 0))
        
        # Initialize the distance and cost dictionaries, and push the source node in first
        distances = {s: 0}

        # Initialize the parent dictionary, and push the source node in first
        # Parent array is given by (energy_cost_to_node, node) : (energy_cost_to_parent, parent)
        parent = {(0, s): None} # Source node has no parent, and energy cost to source node is 0 so far (beginning)

        # Initialize the minimum distance and minimum cost dictionaries, to help optimize searches in the while loop
        min_dist = {}
        min_cost = {}

        # Initialize the path array, to be filled when we reach the target node
        path = []

        # While the priority queue still has any elements, dequeue the element at the front of the queue (least path cost
        # Enqueue this element's children after marking them as visited, if not visited yet
        while not priority_queue.empty():
            # Dequeue the frontmost element from the queue
            [f_n, current_node, current_cost, current_dist] = priority_queue.get()

            # Stop if we have reached the Target Node
            if (current_node == int(target)):
                # We need to reconstruct the path to be printed out, along with the shortest distance from source to target
                target_pair = (current_cost, current_node) # Last pair in the path built, let's work backwards from here
                while target_pair is not None:
                    path.append(target_pair[1])
                    target_pair = parent[target_pair]
                break

            # Skip this node if this node neither has a new lower distance found, new lower cost found but exists in the min dictionaries
            if (
                current_node in min_dist and 
                current_node in min_cost 
            ):
                if (
                    current_dist >= min_dist[current_node] and 
                    current_cost >= min_cost[current_node]
                ):
                    continue

            # If the current node was either not found before, or we have dequeued a path with this node having a shorter distance, we will explore it
            if (
                current_node not in min_dist or
                current_dist < min_dist[current_node]
            ):
                min_dist[current_node] = current_dist
            
            # If the current node was either not found before, or we have dequeued a path with this node having a shorter cost, we will explore it
            if (
                current_node not in min_cost or
                current_cost < min_cost[current_node]
            ):
                min_cost[current_node] = current_cost

            # Go through all children nodes of the dequeued node, using Coord.json
            # child variable comes as a string since it is taken from G.json directly (raw)
            for child in self.g[str(current_node)]:
                int_child = int(child)
                
                # Calculate the total distance to the child from the parent node (current distance + distance to this child from parent)
                distance_to_child = current_dist + self.dist[str(current_node) + "," + child]
                # Calculate the total energy cost to the child, so we can ensure that the update follows the constraint
                cost_to_child = current_cost + self.travel_cost[str(current_node) + "," + child]

                a  = 0

                if(heuristic_type == "coordinate"):
                    a = h.coordinate_distance(self.coord[str(current_node)], self.coord[child])
                elif(heuristic_type == "manhattan"):
                    a = h.manhattan_distance(self.coord[str(current_node)], self.coord[child])
                else:
                    a = h.chebyshev_distance(self.coord[str(current_node)], self.coord[child])
                
                f_n = distance_to_child + a # f(n) = g(n) + h(n)

                # Create the cost and node pair to check if this combination of energy-node has already been found
                energy_node = (cost_to_child, int_child)

                # If this combination has not been explored OR the new distance of this combination is now found to be lower, we will save the details and enqueue it
                # This cost must still fulfill the energy requirements/constraint given
                if ((cost_to_child <= budget) and
                    ((energy_node not in parent) or (distance_to_child <= distances[int_child]))):
                    # Insert this child into the priority_queue
                    priority_queue.put((f_n, int_child, cost_to_child, distance_to_child))

                    # Assign the current node and cost as the parent to the child that will be explored
                    parent[energy_node] = (current_cost, current_node)

                    # Override the distance and cost to the child that will be explored
                    distances[int_child] = distance_to_child

        print("Shortest path: ", end="")
        for node in reversed(path):
            if (node != int(target)):
                print(str(node), end="->")
            else:
                print(node)
        print("Shortest distance:", current_dist)
        print("Total energy cost:", current_cost)
        print("(Bonus) Number of nodes in shortest path (including source and target):", len(path))