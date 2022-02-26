import json
from min_heap import MinHeap # New custom Minimizing Heap Class


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
    priority_queue = MinHeap()

    # Push the source node into the heap, mark distance and cost as 0, and mark it as visited -> O(1)
    s = int(source)
    priority_queue.insert(s, 0, 0, 0)
    distances[s] = 0
    costs[s] = 0
    visited[s] = True
    parent[s] = s

    # While the priority queue still has any elements, dequeue the element at the front of the queue (least path cost
    # Enqueue this element's children after marking them as visited, if not visited yet
    while not priority_queue.is_empty():
        # Dequeue the frontmost element from the queue
        current_node, current_dist, current_cost, f_n = priority_queue.remove()

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
                priority_queue.insert(int_child, distance_to_child, 0, 0) # Insert this child into the priority_queue
                visited[int_child] = True                                 # Mark this child as visited
                distances[int_child] = distance_to_child                  # Save the new distance from the source to this child  
                parent[int_child] = current_node                          # Set the parent of this child as the current_node variable being explored
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
    print("=======================================================")
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

    # Initialize the priority queue
    priority_queue = MinHeap()

    # Push the source node into the heap, mark distance and cost as 0, and mark it as visited -> O(1)
    s = int(source)
    priority_queue.insert(s, 0, 0, 0)
    
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
    while not priority_queue.is_empty():
        # Dequeue the frontmost element from the queue
        [current_node, current_dist, current_cost, f_n] = priority_queue.remove()

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
        for child in g[str(current_node)]:
            int_child = int(child)
            
            # Calculate the total distance to the child from the parent node (current distance + distance to this child from parent)
            distance_to_child = current_dist + dist[str(current_node) + "," + child]
            # Calculate the total energy cost to the child, so we can ensure that the update follows the constraint
            cost_to_child = current_cost + cost[str(current_node) + "," + child]

            # Create the cost and node pair to check if this combination of energy-node has already been found
            energy_node = (cost_to_child, int_child)

            # If this combination has not been explored OR the new distance of this combination is now found to be lower, we will save the details and enqueue it
            # This cost must still fulfill the energy requirements/constraint given
            if ((cost_to_child <= energy_budget) and
                ((energy_node not in parent) or (distance_to_child <= distances[int_child]))):
                # Insert this child into the priority_queue
                priority_queue.insert(int_child, distance_to_child, cost_to_child, 0)

                # Assign the current node and cost as the parent to the child that will be explored
                parent[energy_node] = (current_cost, current_node)

                # Override the distance and cost to the child that will be explored
                distances[int_child] = distance_to_child
    
    print("\nTask 2: Uninformed Search Algorithm (UCS) with constraint")
    print("=========================================================")
    print("Shortest path: ", end="")
    for node in reversed(path):
        if (node != int(target)):
            print(str(node), end="->")
        else:
            print(node)
    print("Shortest distance:", current_dist)
    print("Total energy cost:", current_cost)
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

    # We will find shortest path using the Uniform-Cost Search (UCS) Algorithm
    # UCS is very effective for graphs which are very large
    #* Note that UCS will consider path costs in making its decision, compared to BFS which must go through each level iteratively
    num_nodes = len(g.keys()) # 264346

    # Initialize the priority queue
    priority_queue = MinHeap()

    # Push the source node into the heap, mark distance and cost as 0, and mark it as visited -> O(1)
    s = int(source)
    priority_queue.insert(s, 0, 0, 0)

    # Get the target node's coordinates, for reference by contending child coordinates later
    target_coord = coord[target]

    # Calculate the heuristic distance of the source node to be inserted
    source_coord = coord[source]
    source_f_n = ((target_coord[0] - source_coord[0])**2 + (target_coord[1] - source_coord[1])**2)**0.5 # Pythagoras' Theorem
    
    # Initialize the f(n) dictionary, and push the source node in first
    f_ns = {s: source_f_n}

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
    while not priority_queue.is_empty():
        # Dequeue the frontmost element from the queue
        # For this implementation, the arguments of the distance and heuristic distances are swapped so the heap can rank correctly
        [current_node, f_n, current_cost, current_dist] = priority_queue.remove()

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
        for child in g[str(current_node)]:
            int_child = int(child)
            
            # Calculate the total distance to the child from the parent node (current distance + distance to this child from parent)
            distance_to_child = current_dist + dist[str(current_node) + "," + child]
            
            # New: Calculate the cost of travelling from the child node to the target node
            # Calculation is based on Coordinate Distance, for the heuristic function => h(n)
            #* This will be factored into the COST of travelling to the child, not the distance to the child.
            child_coord = coord[child]
            child_to_goal = ((target_coord[0] - child_coord[0])**2 + (target_coord[1] - child_coord[1])**2)**0.5 # Pythagoras' Theorem

            f_n = distance_to_child + child_to_goal # f(n) = g(n) + h(n)
            
            # Calculate the total energy cost to the child, so we can ensure that the update follows the constraint
            cost_to_child = current_cost + cost[str(current_node) + "," + child]

            # Create the cost and node pair to check if this combination of energy-node has already been found
            energy_node = (cost_to_child, int_child)

            # If this combination has not been explored OR the new distance of this combination is now found to be lower, we will save the details and enqueue it
            # This cost must still fulfill the energy requirements/constraint given
            if ((cost_to_child <= energy_budget) and
                ((energy_node not in parent) or (f_n <= f_ns[int_child]))):
                # Insert this child into the priority_queue
                priority_queue.insert(int_child, f_n, cost_to_child, distance_to_child)

                # Assign the current node and cost as the parent to the child that will be explored
                parent[energy_node] = (current_cost, current_node)

                # Override the f(n) to the child that will be explored
                f_ns[int_child] = f_n
    
    print("\nTask 3: A* Search with constraint")
    print("==================================")
    print("Shortest path: ", end="")
    for node in reversed(path):
        if (node != int(target)):
            print(str(node), end="->")
        else:
            print(node)
    print("Shortest distance:", current_dist)
    print("Total energy cost:", current_cost)
    print("(Bonus) Number of nodes in shortest path (including source and target):", len(path))

'''
def tester(dist):
    path = "1->1363->1358->1357->1356->1276->1273->1277->1269->1267->1268->1284->1283->1282->1255->1253->1260->1259->1249->1246->963->964->962->1002->952->1000->998->994->995->996->987->988->979->980->969->977->989->990->991->2465->2466->2384->2382->2385->2379->2380->2445->2444->2405->2406->2398->2395->2397->2142->2141->2125->2126->2082->2080->2071->1979->1975->1967->1966->1974->1973->1971->1970->1948->1937->1939->1935->1931->1934->1673->1675->1674->1837->1671->1828->1825->1817->1815->1634->1814->1813->1632->1631->1742->1741->1740->1739->1591->1689->1585->1584->1688->1579->1679->1677->104->5680->5418->5431->5425->5424->5422->5413->5412->5411->66->5392->5391->5388->5291->5278->5289->5290->5283->5284->5280->50"
    path_arr = path.split("->")
    distance = 0
    for idx, node in enumerate(path_arr):
        if (idx != len(path_arr) - 1):
            distance += dist[node + "," + path_arr[idx+1]]
    
    print(distance)
'''

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
    # tester(dist)


if (__name__ == "__main__"):
    main()