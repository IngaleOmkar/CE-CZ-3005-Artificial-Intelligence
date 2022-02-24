import json
from binary_heap import Heap # Our custom Minimizing Heap class


def task_one(source: str, target: str, coord, dist, g, cost):
    """Task one solves a relaxed version of the NYC instance (no energy constraint).
    
    Required Data
    -------------
    source
        Starting vertex.
    
    target
        Goal state, and the vertex we are trying to find the shortest distance to, from the source vertex.

    coord
        JSON file, to find the children of a node that is being visited.
    
    dist
        JSON file, to measure direct distances between connected nodes.

    g
        JSON file, to get the total number of nodes.

    cost
        JSON file, to measure the energy cost of travelling between connected nodes.
    """

    # We will find shortest path using the Uniform-Cost Search (UCS) Algorithm
    # UCS is very effective for graphs which are very large
    #* Note that UCS will consider path costs in making its decision, compared to BFS which must go through each level iteratively
    num_nodes = len(g.keys()) # 264346
    # Initialize the visited array, which tracks the visited status of the node, and its parent respectively
    visited = [False] * (num_nodes + 1) # 1-indexed

    # Initialize the parent array, that holds the parent node of a visited node
    parent = [-1] * (num_nodes + 1) # 1-indexed

    # Initialize all distances to infinity
    distances = [9223372036854775807] * (num_nodes + 1)  # 1-indexed => total distance from the source node, to the node at the index

    # Initialize all energy costs to infinity
    costs = [9223372036854775807] * (num_nodes + 1)

    # Initialize the priority queue
    priority_queue = Heap()

    # Push the source node into the heap, mark distance and cost as 0, and mark it as visited -> O(1)
    s = int(source)
    priority_queue.insert(s, 0)
    distances[s] = 0
    costs[s] = 0
    visited[s] = True
    parent[s] = s

    # While the priority queue still has any elements, dequeue the element at the front of the queue (least path cost
    # Enqueue this element's children after marking them as visited, if not visited yet
    while not priority_queue.is_empty():
        # Dequeue the frontmost element from the queue
        current_node = priority_queue.remove()
        # print("Current Node:", current_node, "-", str(distances[current_node]))

        # Stop if we have reached the Target Node
        if (current_node == int(target)):
            break

        # Go through all children nodes of the dequeued node, using Coord.json
        # child comes as a string since it is taken from G.json directly (raw)
        for child in g[str(current_node)]:
            int_child = int(child)
            
            # Calculate the total distance to the child from the parent node (current distance + distance to this child from parent)
            distance_to_child = distances[current_node] + dist[str(current_node) + "," + child]

            # If the new total distance to the child is smaller or the child has not been visited before, we will update the distances array and the parent of this child
            if (distance_to_child < distances[int_child]) or (visited[int_child] == False):
                priority_queue.insert(int_child, distance_to_child) # Insert this child into the priority_queue
                visited[int_child] = True                           # Mark this child as visited
                distances[int_child] = distance_to_child            # Save the new distance from the source to this child  
                parent[int_child] = current_node                    # Set the parent of this child as the current_node variable being explored
                # Calculate the total energy cost to the child, and set the child's cost to it
                costs[int_child] = costs[current_node] + cost[str(current_node) + "," + child]

    # We are out of the while loop => All shortest distances have been calculated from the source to target node
    # We need to reconstruct the path and print it out, along with the shortest distance from source to target
    # We will start from the target node, and work backwards using the parent array
    #* O(n)
    path = []
    start = int(target)
    while(start != s):
        path.append(start)
        start = parent[start]
    path.append(start)

    # print("Number of nodes in shortest path (including source and target):", len(path))
    print("\nTask 1: Relaxed version of NYC instance (no constraint)")
    print("========================================================")
    print("Shortest path: ", end="")
    for node in reversed(path):
        if (node != int(target)):
            print(str(node), end="->")
        else:
            print(node)
    print("Shortest distance:", distances[int(target)])
    print("Total energy cost:", costs[int(target)])
    print("(Bonus) Number of nodes in shortest path (including source and target):", len(path))


def task_two(source: str, target: str, coord, dist, g, cost, energy_budget: int):
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
    num_nodes = len(g.keys()) # 264346
    # Initialize the visited array, which tracks the visited status of the node, and its parent respectively
    visited = [False] * (num_nodes + 1) # 1-indexed

    # Initialize the parent array, that holds the parent node of a visited node
    parent = [-1] * (num_nodes + 1) # 1-indexed

    # Initialize all distances to infinity
    distances = [9223372036854775807] * (num_nodes + 1)  # 1-indexed => total distance from the source node, to the node at the index

    # Initialize all energy costs to infinity
    costs = [9223372036854775807] * (num_nodes + 1)

    # Initialize the priority queue
    priority_queue = Heap()

    # Push the source node into the heap, mark distance and cost as 0, and mark it as visited -> O(1)
    s = int(source)
    priority_queue.insert(s, 0)
    distances[s] = 0
    costs[s] = 0
    visited[s] = True
    parent[s] = s

    # While the priority queue still has any elements, dequeue the element at the front of the queue (least path cost
    # Enqueue this element's children after marking them as visited, if not visited yet
    while not priority_queue.is_empty():
        # Dequeue the frontmost element from the queue
        current_node = priority_queue.remove()
        # print("Current Node:", current_node, "-", str(distances[current_node]))

        # Stop if we have reached the Target Node
        if (current_node == int(target)):
            break

        # Go through all children nodes of the dequeued node, using Coord.json
        # child comes as a string since it is taken from G.json directly (raw)
        for child in g[str(current_node)]:
            int_child = int(child)
            
            # Calculate the total distance to the child from the parent node (current distance + distance to this child from parent)
            distance_to_child = distances[current_node] + dist[str(current_node) + "," + child]
            # Calculate the total energy cost to the child, so we can ensure that the update follows the constraint
            cost_to_child = costs[current_node] + cost[str(current_node) + "," + child]

            # If the new total distance to the child is smaller or the child has not been visited before, we will update the distances array and the parent of this child
            if (
                cost_to_child < energy_budget and 
                ((distance_to_child < distances[int_child]) or (visited[int_child] == False))
            ):
                priority_queue.insert(int_child, distance_to_child) # Insert this child into the priority_queue
                visited[int_child] = True                           # Mark this child as visited
                distances[int_child] = distance_to_child            # Save the new distance from the source to this child 
                costs[int_child] = cost_to_child                    # Save the new cost from the source to this child 
                parent[int_child] = current_node                    # Set the parent of this child as the current_node variable being explored                

    # We are out of the while loop => All shortest distances have been calculated from the source to target node
    # We need to reconstruct the path and print it out, along with the shortest distance from source to target
    # We will start from the target node, and work backwards using the parent array
    #* O(n)
    path = []
    start = int(target)
    while(start != s):
        path.append(start)
        start = parent[start]
    path.append(start)

    # print("Number of nodes in shortest path (including source and target):", len(path))
    print("\nTask 2: Uninformed Search Algorithm (UCS) with constraint")
    print("==========================================================")
    print("Shortest path: ", end="")
    for node in reversed(path):
        if (node != int(target)):
            print(str(node), end="->")
        else:
            print(node)
    print("Shortest distance:", distances[int(target)])
    print("Total energy cost:", costs[int(target)])
    print("(Bonus) Number of nodes in shortest path (including source and target):", len(path))


def task_three(source: str, target: str, coord, dist, g, cost, energy_budget: int):
    """Task three solves the NYC problem instance using A* Search.
    
    A heuristic function will be included and implemented.

    Required Data
    -------------
    source : str
        Starting vertex.
    
    target : str
        Goal state, and the vertex we are trying to find the shortest distance to, from the source vertex.

    coord
        JSON file, to find the children of a node that is being visited, and also help with calculating distances between them, for the Heuristic Function.
    
    dist
        JSON file, to measure direct distances between connected nodes.

    g
        JSON file, to get the total number of nodes.

    cost
        JSON file, to measure the energy cost of travelling between connected nodes.

    energy_budget : int
        Maximum permissible energy that we are allowed to use to travel from the source node to target node. Used as the constraint in this problem to adhere to, while finding the minimum distance possible.
    """

    '''
    Past Distance + Distance from current node to destination node
    Manhattan Distance  => x2 - x1 + y2 - y1
    Coordinate Distance => Pythagoras' Theorem (look into this, see which one is better to use)
    Try 2 heuristic functions:
        1. Past distance (dijkstra) + distance from current node to destination node (using Coordinate Distances) <- Currently being attempted
        2. Past distance (dijkstra) + distance from current node to destination node (using Manhattan Distances)
        Then, compare the functions (for deeper analysis)
    '''

    #* A* Search will consider both the current distance together with the heuristic function, and the energy cost, to decide 
    num_nodes = len(g.keys()) # 264346
    # Initialize the visited array, which tracks the visited status of the node, and its parent respectively
    visited = [False] * (num_nodes + 1) # 1-indexed

    # Initialize the parent array, that holds the parent node of a visited node
    parent = [-1] * (num_nodes + 1) # 1-indexed

    # Initialize all distances to infinity
    distances = [9223372036854775807] * (num_nodes + 1)  # 1-indexed => total distance from the source node, to the node at the index

    # Initialize the array of saved heuristic distances
    f_ns = [9223372036854775807] * (num_nodes + 1)

    # Initialize all energy costs to infinity
    costs = [9223372036854775807] * (num_nodes + 1)

    # Initialize the priority queue
    priority_queue = Heap()

    # Get the target node's coordinates, for reference by contending child coordinates later
    target_coord = coord[target]

    # Push the source node into the heap, mark distance and cost as 0, and mark it as visited -> O(1)
    s = int(source)

    # Calculate the heuristic distance of the source node to be inserted
    source_coord = coord[source]
    source_f_n = ((target_coord[0] - source_coord[0])**2 + (target_coord[1] - source_coord[1])**2)**0.5 # Pythagoras' Theorem

    # Insert the source node, and source node details
    priority_queue.insert(s, source_f_n)
    distances[s] = 0
    f_ns[s] = source_f_n
    costs[s] = 0
    visited[s] = True
    parent[s] = s

    # While the priority queue still has any elements, dequeue the element at the front of the queue (least path cost
    # Enqueue this element's children after marking them as visited, if not visited yet
    while not priority_queue.is_empty():
        # Dequeue the frontmost element from the queue
        current_node = priority_queue.remove()
        # print("Current Node:", current_node, "-", str(distances[current_node]))

        # Stop if we have reached the Target Node
        if (current_node == int(target)):
            break

        # Go through all children nodes of the dequeued node, using Coord.json
        # child comes as a string since it is taken from G.json directly (raw)
        for child in g[str(current_node)]:
            int_child = int(child)
            
            # Calculate the total distance to the child from the parent node (current distance + distance to this child from parent)
            distance_to_child = distances[current_node] + dist[str(current_node) + "," + child]

            # New: Calculate the cost of travelling from the child node to the target node
            # Calculation is based on Coordinate Distance, for the heuristic function => h(n)
            #* This will be factored into the COST of travelling to the child, not the distance to the child.
            child_coord = coord[child]
            child_to_goal = ((target_coord[0] - child_coord[0])**2 + (target_coord[1] - child_coord[1])**2)**0.5 # Pythagoras' Theorem

            f_n = distance_to_child + child_to_goal # f(n) = g(n) + h(n)

            # Calculate the total energy cost to the child => g(n)
            cost_to_child = costs[current_node] + cost[str(current_node) + "," + child]

            # If the new estimated total distance to the child is smaller or the child has not been visited before, we will update this child's information
            if (
                cost_to_child < energy_budget and 
                ((f_n < f_ns[int_child]) or (visited[int_child] == False))
            ):
                # The former is the final result, and the latter is the value used to rank nodes
                priority_queue.insert(int_child, f_n)    # Insert this child into the priority_queue, but use the heuristic distance to prioritize the nodes
                visited[int_child] = True                # Mark this child as visited
                distances[int_child] = distance_to_child # Save the new (actual, non-heuristic) distance from the source to this child 
                f_ns[int_child] = f_n                    # Save the heuristic distance for the child
                costs[int_child] = cost_to_child         # Save the cost of travelling to the child
                parent[int_child] = current_node         # Set the parent of this child as the current_node variable being explored                

    # We are out of the while loop => All shortest distances have been calculated from the source to target node
    # We need to reconstruct the path and print it out, along with the shortest distance from source to target
    # We will start from the target node, and work backwards using the parent array
    #* O(n)
    path = []
    start = int(target)
    while(start != s):
        path.append(start)
        start = parent[start]
    path.append(start)

    print("\nTask 3: A* Search with constraint")
    print("==================================")
    print("Shortest path: ", end="")
    for node in reversed(path):
        if (node != int(target)):
            print(str(node), end="->")
        else:
            print(node)
    print("Shortest distance:", distances[int(target)])
    print("Total energy cost:", costs[int(target)])
    print("(Bonus) Number of nodes in shortest path (including source and target):", len(path))


def main():
    """The main function will take in inputs, and based on them, carry out tasks one, two or three.   
    
    Inputs
    ------
        The allowed inputs will be documented here.

    Raises
    ------
        The exceptions that will be raised will be documented here.
    """

    coord = json.load(open("Lab1\coord.json"))
    dist = json.load(open("Lab1\dist.json"))
    g = json.load(open("Lab1\g.json"))
    cost = json.load(open("Lab1\cost.json"))
    start_node: str = "1"
    end_node: str = "50"
    energy_budget: int = 287932

    task_one(start_node, end_node, coord, dist, g, cost)
    task_two(start_node, end_node, coord, dist, g, cost, energy_budget)
    task_three(start_node, end_node, coord, dist, g, cost, energy_budget)


if (__name__ == "__main__"):
    main()