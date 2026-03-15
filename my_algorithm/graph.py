from aerohacks.core.models import Position2D
from geometry import dist, crosses_poly
import heapq
import math


class VisGraph:
    """
    Node 0 = start, node 1 = goal, nodes 2..N = obstacle boundary vertices.
    Edges between all mutually visible pairs, weighted by dist * cost_mult.
    Hard obstacles block edges; soft obstacles inflate weights.
    """

    def __init__(self):
        self.nodes = []
        self.obs   = []
        self.adj   = {}

    def build(self, start: Position2D, goal: Position2D, obstacles: list):
        self.obs   = [o for o in obstacles if o is not None]
        self.nodes = [start, goal]
        for o in self.obs:
            self.nodes.extend(o.verts)

        n        = len(self.nodes)
        self.adj = {i: [] for i in range(n)}

        for i in range(n):
            for j in range(i + 1, n):
                a, b      = self.nodes[i], self.nodes[j]
                blocked   = False
                cost_mult = 1.0
                for o in self.obs:
                    if o.blocks(a, b):
                        if o.is_hard:
                            blocked = True
                            break
                        cost_mult = max(cost_mult, o.cost_mult)
                if not blocked:
                    w = dist(a, b) * cost_mult
                    self.adj[i].append((j, w))
                    self.adj[j].append((i, w))

    def astar(self) -> list | None:
        """Returns path as list of Position2D, or None."""
        if len(self.nodes) < 2:
            return None

        goal = self.nodes[1]
        heap = [(0.0, 0.0, 0, -1)]
        seen = {}
        cost = {0: 0.0}

        while heap:
            _, cg, cur, par = heapq.heappop(heap)
            if cur in seen:
                continue
            seen[cur] = par

            if cur == 1:
                path, node = [], 1
                while node != -1:
                    path.append(self.nodes[node])
                    node = seen[node]
                path.reverse()
                return path

            for nb, w in self.adj.get(cur, []):
                ng = cg + w
                if ng < cost.get(nb, math.inf):
                    cost[nb] = ng
                    h = dist(goal, self.nodes[nb])
                    heapq.heappush(heap, (ng + h, ng, nb, cur))

        return None
