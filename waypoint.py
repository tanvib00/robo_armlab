class Waypoint:
    def __init__(self, x, y, neighbors=[], parent=None):
        self.x = x
        self.y = y
        self.neighbors = neighbors
        self.parent = parent
    
    def __eq__(self, other):
        return (isinstance(other, Waypoint) and (self.x == other.x) and (self.y == other.y))
    
    def getTuple(self):
        return tuple((self.x, self.y))