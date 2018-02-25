class Node:
    def __init__(self, position, parent):
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0
    def __cmp__(self, other):
        return cmp(self.f, other.f)
    def __str__(self):
        return str(self.position) # change back to f
    def __hash__(self):
        return hash(self.position)
    def __eq__(self, other):
        return self.position == other.position

