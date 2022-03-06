import json

from question1 import question_one
from question2_Modified import question_two as question_two_M
from question2_Original import question_two as question_two_O
from question3 import question_three

# Load the input data as JSON objects
coord = json.load(open("Lab1/coord.json"))
dist = json.load(open("Lab1/dist.json"))
g = json.load(open("Lab1/g.json"))
cost = json.load(open("Lab1/cost.json"))

# Initialize the source and target vertices 
source = 1
target = 50

# Task 1 (UCS, no constraint)
q1 = question_one(coord, dist, g)
distance, route, counts = q1.uniform_cost_search(1, 50)
print("Shortest Path:", route)
print("\nShortest Distance:", distance)
print("\nNodes Explored:", counts, end="\n\n")

# Task 2 (UCS)
q2_O = question_two_O(coord, cost, dist, g)
distance, energy, route, counts = q2_O.uniform_cost_search(1, 50, 287932)
print("Shortest Path:", route)
print("\nShortest Distance:", distance)
print("\nCost:", energy)
print("\nNodes Explored:", counts, end="\n\n")

# Task 2 (Modified Version of UCS, with optimization)
q2_M = question_two_M(coord, cost, dist, g)
distance, energy, route, counts = q2_M.uniform_cost_search(1, 50, 287932)
print("Shortest Path:", route)
print("\nShortest Distance:", distance)
print("\nCost:", energy)
print("\nNodes Explored:", counts, end="\n\n")

# Define the heuristics we are testing out for Task 3 (A* Search)
heuristicTypes = ['Coordinate', 'Chebyshev', 'Manhattan']
# Task 3 (Ran once for each of the 3 heuristics we are attempting)
for type in heuristicTypes:
    q3 = question_three(coord, cost, dist, g, type)
    distance, energy, route, counts = q3.astar_search(1, 50, 287932)
    print("Heuristic Approach:", type)
    print("\nShortest Path:", route)
    print("\nShortest Distance:", distance)
    print("\nCost:", energy)
    print("\nNodes Explored:", counts, end="\n\n")