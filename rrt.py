from PIL import Image, ImageDraw
import numpy as np
import math


def distance(a, b):
    return math.sqrt(math.pow(a[0] - b[0], 2) + math.pow(a[1] - b[1], 2))


class Vertex:
    def __init__(self, state):
        self.state = state
        self.neighbors = []



class RRT:
    def __init__(self, img):
        self.img = img

    def build(self, x_init, K, delta):
        self.T = []
        self.T.append(Vertex(x_init))
        for k in range(0, K):
            x_rand = self._random_state()
            self._extend(x_rand, delta)

    def nearest_neighbor(self, x):
        minDist = None
        result = None
        for v in self.T:
            dist = distance(x, v.state)
            if minDist is None or dist < minDist:
                minDist = dist
                result = v
        return result

    def _random_state(self):
        # width, height = self.img.size
        # self.width, self.height
        x = np.random.uniform(0, self.img.width)
        y = np.random.uniform(0, self.img.height)
        return np.array([x,y])

    def _extend(self, x, delta):
        x_near = self.nearest_neighbor(x)
        dist = distance(x, x_near.state)
        if dist < delta:
            x_new = x
        else:
            x_new = x_near.state + (x - x_near.state) / dist * delta
        if not self.img.has_obstacle(x_new[0], x_new[1]):
            self.T.append(Vertex(x_new))
            x_near.neighbors.append(self.T[-1])

    def shortest_path(self, goal):
        dist = dict()
        prev = dict()
        Q = []

        for v in self.T:
            dist[v] = float("inf")
            prev[v] = None
            Q.append(v)

        dist[self.T[0]] = 0

        while len(Q) > 0:
            # find vertex with min dist
            minDist = float("inf")
            minU = None
            minIdx = None
            for idx in range(0, len(Q)):
                u = Q[idx]
                if dist[u] < minDist:
                    minU = u
                    minDist = dist[u]
                    minIdx = idx
            u = minU
            del Q[minIdx]

            for v in u.neighbors:
                if v in Q:
                    alt = dist[u] + distance(u.state, v.state)
                    if alt < dist[v]:
                        dist[v] = alt
                        prev[v] = u

        # find solution
        S = []
        u = goal
        while prev[u] is not None:
            S.insert(0, u)
            u = prev[u]
        S.insert(0, u)
        return S
