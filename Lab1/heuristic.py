class heuristics:

    def __init__(self) -> None:
        pass

    def coordinate_distance(self, coord1, coord2):
        return ((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)**0.5
    
    def manhattan_distance(self, coord1, coord2):
        return abs(coord1[0] - coord2[0]) + abs(coord1[1] - coord2[1])
    
    def chebyshev_distance(self, coord1, coord2):
        return max(abs(coord1[0] - coord2[0]), abs(coord1[1] - coord2[1]))
