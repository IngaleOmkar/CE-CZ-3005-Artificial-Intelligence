from question1 import question_one
from question2 import question_two
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