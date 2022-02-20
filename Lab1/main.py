import json
from binary_heap import Heap # Our custom Minimizing Heap class


def task_one(source: str, target: str, coord, dist, g):
    """Task one solves a relaxed version of the NYC instance (no energy constraint).
    
    Required Data
    -------------
    source
        Starting vertex.
    
    target
        Goal state, and the vertex we are trying to find the shortest distance to, from the source vertex.

    coord.json
        To find the children of a node that is being visited.
    
    dist.json
        To measure direct distances between connected nodes.

    g.json
        To get the total number of nodes.
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

    # Initialize the priority queue
    priority_queue = Heap()

    # Push the source node into the heap, mark distance as 0, and mark it as visited -> O(1)
    s = int(source)
    priority_queue.insert(s, 0)
    distances[s] = 0
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

    print("Number of nodes in shortest path (including source and target):", len(path))
    '''
    for node in reversed(path):
        if (node != int(target)):
            print(str(node), end="->")
        else:
            print(node)
    '''
    print("Shortest distance found:", distances[int(target)])


def task_two():
    """Task one solves the NYC problem instance with the energy constraint included."""
    
    print("Task 2")


def task_three():
    """Task one solves the NYC problem instance using A* Search.
    
    A heuristic function will be included and implemented.
    """
    # past distance + distance from current node to destination node
    # manhattan distance  -> x2 - x1 + y2 - y1
    # coordinate distance -> look into this, see which one is better to use
    # Try 2 heuristic functions
    # 1. past distance (dijkstra) + distance from current node to destination node (using coordinate distances)
    # 2. past distance (dijkstra) + distance from current node to destination node (using manhattan distances)
    # Compare the functions (deeper analysis)

    print("Task 3")


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

    task_one("1", "50", coord, dist, g)
    # task_two()
    # task_three()


if (__name__ == "__main__"):
    main()