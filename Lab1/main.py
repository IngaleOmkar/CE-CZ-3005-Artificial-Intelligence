from question1 import question_one
from question2 import question_two
import json

# Input data
coord = json.load(open("Lab1/coord.json"))
dist = json.load(open("Lab1/dist.json"))
g = json.load(open("Lab1/g.json"))
cost = json.load(open("Lab1/cost.json"))

# Source and target vertices 
source = 1
target = 50

q1 = question_one(coord, dist, g)
distance, route = q1.uniform_cost_search(1, 50)
print("Shortest Path:", route)
print("\nShortest Distance:", distance)

q2 = question_two(coord, cost, dist, g)
distance, cost, route = q2.uniform_cost_search(1, 50, 287932)
print("Shortest Path:", route)
print("\nShortest Distance:", distance)
print("\nCost:", cost)