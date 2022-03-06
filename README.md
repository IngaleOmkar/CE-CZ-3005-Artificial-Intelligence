# CE-CZ-3005-Artificial-Intelligence
This is the repository for Team AM's implementation of Lab 1, for the Nanyang Technological University's module "CE/CZ3005: Artificial Intelligence".

It contains code for each of the three tasks given, written as classes and called within the module `main.py`. Alternative implementations with optimizations can also be found within this repository.


# Data Provided
## 1. coord.json
JSON file containing the coordinates of each node within the graph.

## 2. cost.json
JSON file containing the energy cost of travelling between two nodes in the graph.
This file is used for adhering to the energy constraints outlined for tasks two and three.

## 3. dist.json
JSON file containing the distance between two nodes in the graph.
This file is used to help with calculating cumulative distances from a starting node to a target node, for choosing nodes to explore.

## 4. g.json
JSON file containing every node within the graph, along with its adjacent nodes represented by a list of strings.