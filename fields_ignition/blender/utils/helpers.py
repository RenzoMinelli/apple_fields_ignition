import numpy as np


class Node:
    def __init__(self, type, x, y, z, alpha, r):
        self.x = x
        self.y = y
        self.z = z
        self.alpha = alpha
        self.r = r
        self.type = type

    def __str__(self):
        return str((self.x, self.y, self.z))

    def __ge__(self, other):
        return self.vector() >= other.vector()

    def vector(self):
        return np.array([self.x, self.y, self.z])

    def next(self, x, y, z, alpha):
        return Node(self.x + x, self.y + y, self.z + z, self.alpha + alpha)


def create_ring_verts(node, DIV):
    verts = []
    for i in range(DIV):
        a = (np.pi * 2) * (i / DIV)
        x = node.x + np.cos(a) * node.r
        y = node.y + np.sin(a) * node.r
        verts.append((x, y, node.z))
    return verts
