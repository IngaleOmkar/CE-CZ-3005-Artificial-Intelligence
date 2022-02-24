from question1 import question_one
from question2 import question_two
from question3 import question_three
import json

# Input data
coord = json.load(open("coord.json"))
dist = json.load(open("dist.json"))
g = json.load(open("g.json"))
cost = json.load(open("cost.json"))

# Source and target vertices 
source = 1
target = 50

q1 = question_one(coord, dist, g)
distance, route = q1.uniform_cost_search(1, 50)
print("Shortest Path:", route)
print("\nShortest Distance:", distance)

q2 = question_two(coord, cost, dist, g)
distance, energy, route = q2.uniform_cost_search(1, 50, 287932)
print("Shortest Path:", route)
print("\nShortest Distance:", distance)
print("\nCost:", energy)

q3 = question_three(coord, cost, dist, g)
distance, energy, route = q3.astar_search(1, 50, 287932)
print("Shortest Path:", route)
print("\nShortest Distance:", distance)
print("\nCost:", energy)