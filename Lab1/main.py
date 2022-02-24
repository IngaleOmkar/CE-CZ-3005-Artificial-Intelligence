from question1 import question_one
from question2 import question_two
from question3 import question_three
import json

# Input data
coord = json.load(open('coord.json'))
dist = json.load(open("dist.json"))
g = json.load(open("g.json"))
cost = json.load(open("cost.json"))

# Source and target vertices 
source = 1
target = 50

q1 = question_one(coord, dist, g)
q1.solve(1, 50)

q2 = question_two(coord, dist, g, cost)
q2.solve(1, 50, 287932)

q3 = question_three(coord, dist, g, cost)
q3.solve(1, 50, 287932, "chebyshev")