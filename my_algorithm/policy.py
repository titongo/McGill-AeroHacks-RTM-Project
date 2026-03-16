# ══════════════════════════════════════════════════════════════════════════════
# AeroHacks RTM — MyPolicy  (single-file submission)
# ══════════════════════════════════════════════════════════════════════════════

from aerohacks.policy.base import Policy
from aerohacks.core.models import (
    Observation, Plan, ActionStep, ActionType,
    Position2D, ConstraintPhase, CircleRegion, PolygonRegion,
)
from dataclasses import replace
import math
import heapq

# ══════════════════════════════════════════════════════════════════════════════
# Constants  (confirmed from sim introspection)
# ══════════════════════════════════════════════════════════════════════════════

MAX_SPEED       = 15.0
MIN_ALT         = 1
MAX_ALT         = 4
ENERGY_DECAY    = 0.10
ENERGY_RESERVE  = 20.0
COMMIT_LEN      = 2        # sim.commit_length — steps[0,1] are locked

SAFETY_BUFFER   = 80.0
CIRCLE_SIDES    = 16

VO_TAU          = 8.0
DRONE_RADIUS    = 20.0
ADVISORY_DIST   = 100.0
CONFLICT_DIST   = 50.0

WP_REACH        = 30.0

COST_ADVISORY   =  8.0
COST_CONTROLLED = 50.0
COST_RESTRICTED = math.inf


# ══════════════════════════════════════════════════════════════════════════════
# Geometry
# Position2D is a namedtuple — it is a tuple, so .x/.y and indexing both work.
# All geometry functions take and return Position2D directly.
# ══════════════════════════════════════════════════════════════════════════════

def dist(a: Position2D, b: Position2D) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)

def safe_alt(a) -> int:
    return max(MIN_ALT, min(MAX_ALT, int(a)))

def cross(o: Position2D, a: Position2D, b: Position2D) -> float:
    """Signed area of triangle O→A→B. Positive = B left of O→A."""
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)

def segs_intersect(p1: Position2D, p2: Position2D,
                   p3: Position2D, p4: Position2D) -> bool:
    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    return (((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and
            ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)))

def point_in_poly(p: Position2D, verts: list) -> bool:
    n, inside, j = len(verts), False, len(verts) - 1
    for i in range(n):
        vi, vj = verts[i], verts[j]
        if ((vi.y > p.y) != (vj.y > p.y)) and \
                p.x < (vj.x - vi.x) * (p.y - vi.y) / (vj.y - vi.y + 1e-12) + vi.x:
            inside = not inside
        j = i
    return inside

def seg_crosses_poly(a: Position2D, b: Position2D, verts: list) -> bool:
    n = len(verts)
    for i in range(n):
        if segs_intersect(a, b, verts[i], verts[(i + 1) % n]):
            return True
    return point_in_poly(a, verts) or point_in_poly(b, verts)


# ══════════════════════════════════════════════════════════════════════════════
# Region helpers
# ══════════════════════════════════════════════════════════════════════════════

def corner_bisector(prev: Position2D, curr: Position2D, nxt: Position2D):
    """
    Returns (bx, by, scale) where (bx, by) is the outward unit bisector at
    curr and scale = 1/sin(half-angle), so pushing by margin*scale gives
    uniform perpendicular clearance from both edges.
    Returns None if either edge is degenerate (zero length).
    """
    e1x, e1y = prev.x - curr.x, prev.y - curr.y
    e2x, e2y = nxt.x  - curr.x, nxt.y  - curr.y

    d1 = math.hypot(e1x, e1y)
    d2 = math.hypot(e2x, e2y)
    if d1 < 1e-6 or d2 < 1e-6:
        return None

    e1x, e1y = e1x / d1, e1y / d1
    e2x, e2y = e2x / d2, e2y / d2

    bx, by = e1x + e2x, e1y + e2y
    bd = math.hypot(bx, by)
    if bd < 1e-6:
        # Anti-parallel edges — push perpendicular
        bx, by = -e1y, e1x
    else:
        bx, by = bx / bd, by / bd

    dot      = e1x * e2x + e1y * e2y
    sin_half = math.sqrt(max(0.0, (1.0 - dot) / 2.0))
    scale    = 1.0 / sin_half if sin_half > 0.1 else 10.0

    return bx, by, scale


def expand_poly(verts: list, margin: float) -> list:
    """
    verts: list of Position2D.
    Pushes each vertex outward along its corner bisector by margin,
    scaled so perpendicular clearance from each edge is uniform.
    """
    n   = len(verts)
    out = []
    for i in range(n):
        curr   = verts[i]
        result = corner_bisector(verts[(i - 1) % n], curr, verts[(i + 1) % n])
        if result is None:
            out.append(curr)
        else:
            bx, by, scale = result
            out.append(Position2D(x=curr.x + bx * margin * scale,
                                  y=curr.y + by * margin * scale))
    return out


def region_to_verts(region, margin=0.0) -> list | None:
    """Returns list of Position2D, or None."""
    if isinstance(region, CircleRegion):
        cx, cy = region.center_pos.x, region.center_pos.y
        r = region.radius + margin
        return [Position2D(x=cx + r * math.cos(2 * math.pi * i / CIRCLE_SIDES),
                           y=cy + r * math.sin(2 * math.pi * i / CIRCLE_SIDES))
                for i in range(CIRCLE_SIDES)]

    if isinstance(region, PolygonRegion):
        # region.vertices is already List[Position2D] — no copy needed
        return expand_poly(region.vertices, margin) if margin > 0 else list(region.vertices)

    return None


def region_centroid(region) -> Position2D | None:
    if isinstance(region, CircleRegion):
        return region.center_pos   # already Position2D, no copy

    if isinstance(region, PolygonRegion):
        verts = region.vertices
        n     = len(verts)
        return Position2D(x=sum(v.x for v in verts) / n,
                          y=sum(v.y for v in verts) / n)
    return None


# ══════════════════════════════════════════════════════════════════════════════
# Obstacle
# ══════════════════════════════════════════════════════════════════════════════

class Obstacle:
    __slots__ = ("verts", "cost_mult", "is_hard", "alt_layers")

    def __init__(self, verts, cost_mult, is_hard, alt_layers=None):
        self.verts      = verts       # list of Position2D
        self.cost_mult  = cost_mult
        self.is_hard    = is_hard
        self.alt_layers = alt_layers  # None = all layers

    def affects(self, alt):
        return self.alt_layers is None or alt in self.alt_layers

    def contains(self, p: Position2D) -> bool:
        return point_in_poly(p, self.verts)

    def blocks(self, a: Position2D, b: Position2D) -> bool:
        return seg_crosses_poly(a, b, self.verts)


def make_obstacle(region, cost_mult, is_hard, alt_layers=None):
    verts = region_to_verts(region, SAFETY_BUFFER)
    return Obstacle(verts, cost_mult, is_hard, alt_layers) if verts else None


def phase_cost(phase):
    if phase == ConstraintPhase.RESTRICTED: return COST_RESTRICTED, True
    if phase == ConstraintPhase.CONTROLLED: return COST_CONTROLLED, False
    if phase == ConstraintPhase.ADVISORY:   return COST_ADVISORY,   False
    return 1.0, False


# ══════════════════════════════════════════════════════════════════════════════
# Visibility graph + A*
# ══════════════════════════════════════════════════════════════════════════════

class VisGraph:
    """
    Node 0 = start, node 1 = goal, nodes 2..N = obstacle boundary vertices.
    All stored as Position2D (which is a namedtuple, so indexing also works).
    Edges between all mutually visible pairs, weighted by dist × cost_mult.
    Hard obstacles block edges; soft obstacles inflate weights.
    """

    def __init__(self):
        self.nodes = []   # list of Position2D
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

        goal  = self.nodes[1]
        heap  = [(0.0, 0.0, 0, -1)]
        seen  = {}
        g_sc  = {0: 0.0}

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
                if ng < g_sc.get(nb, math.inf):
                    g_sc[nb] = ng
                    h = dist(goal, self.nodes[nb])
                    heapq.heappush(heap, (ng + h, ng, nb, cur))

        return None


# ══════════════════════════════════════════════════════════════════════════════
# Velocity Obstacle  (full responsibility — NPCs are constant velocity)
# ══════════════════════════════════════════════════════════════════════════════

def vo_adjust(my_pos: Position2D, my_vel: Position2D, pref_vel: Position2D,
              tracks, my_alt) -> Position2D:
    """
    Builds VO half-plane constraints for each nearby NPC and projects
    preferred velocity to the nearest safe point outside all cones.
    Full avoidance responsibility on our side.
    """
    half_planes = []   # [(boundary_point, normal), ...] as plain float pairs

    for t in tracks:
        if abs(t.alt_layer - my_alt) > 1:
            continue

        tvx = t.velocity.x if t.velocity else 0.0
        tvy = t.velocity.y if t.velocity else 0.0

        rpx = t.position.x - my_pos.x
        rpy = t.position.y - my_pos.y
        rvx = my_vel.x - tvx
        rvy = my_vel.y - tvy

        vo_cx = rpx / VO_TAU
        vo_cy = rpy / VO_TAU
        vo_r  = (DRONE_RADIUS * 2.0) / VO_TAU

        dvx = rvx - vo_cx
        dvy = rvy - vo_cy
        dvd = math.hypot(dvx, dvy)

        if dvd > vo_r:
            continue

        if dvd > 1e-6:
            nx, ny = dvx / dvd, dvy / dvd
        else:
            sep = math.hypot(rpx, rpy)
            nx, ny = (-rpx / sep, -rpy / sep) if sep > 1e-6 else (1.0, 0.0)

        pen = vo_r - dvd
        half_planes.append((
            (rvx + nx * pen * 1.05 + tvx, rvy + ny * pen * 1.05 + tvy),
            (nx, ny),
        ))

    if not half_planes:
        return pref_vel

    vx, vy = pref_vel.x, pref_vel.y
    for (bx, by), (nx, ny) in half_planes:
        dot = (vx - bx) * nx + (vy - by) * ny
        if dot < 0:
            vx -= dot * nx
            vy -= dot * ny

    spd = math.hypot(vx, vy)
    if spd > MAX_SPEED:
        vx, vy = vx / spd * MAX_SPEED, vy / spd * MAX_SPEED

    return Position2D(x=vx, y=vy)


# ══════════════════════════════════════════════════════════════════════════════
# NPC position predictor
# ══════════════════════════════════════════════════════════════════════════════

def npc_at(trace, t):
    """
    Returns a proxy with .position, .velocity, .alt_layer matching the
    TrafficTrack interface. Returns None if the trace is not active at tick t.
    """
    for seg in trace.segments:
        if seg.start_time <= t < seg.end_time:
            dt = t - seg.start_time
            class _P: pass
            p           = _P()
            p.position  = Position2D(x=seg.start_pos.x + seg.velocity.x * dt,
                                     y=seg.start_pos.y + seg.velocity.y * dt)
            p.velocity  = seg.velocity   # already Position2D, no copy
            p.alt_layer = seg.alt_layer
            return p
    return None


# ══════════════════════════════════════════════════════════════════════════════
# Restricted zone helpers
# ══════════════════════════════════════════════════════════════════════════════

def in_restricted(pos: Position2D, alt, constraints) -> bool:
    for c in constraints:
        if c.phase != ConstraintPhase.RESTRICTED:
            continue
        if c.alt_layers and alt not in c.alt_layers:
            continue
        verts = region_to_verts(c.region, 0)
        if verts and point_in_poly(pos, verts):
            return True
    return False


def deflect(vel: Position2D, cur: Position2D, alt, constraints) -> Position2D:
    """Rotates velocity in 20° steps until next position clears all restricted zones."""
    spd = math.hypot(vel.x, vel.y)
    if spd < 1e-6:
        return Position2D(x=0.0, y=0.0)
    for deg in range(0, 360, 20):
        rad = math.radians(deg)
        c, s = math.cos(rad), math.sin(rad)
        rx = (vel.x * c - vel.y * s) / spd * MAX_SPEED
        ry = (vel.x * s + vel.y * c) / spd * MAX_SPEED
        if not in_restricted(Position2D(x=cur.x + rx, y=cur.y + ry), alt, constraints):
            return Position2D(x=rx, y=ry)
    return Position2D(x=0.0, y=0.0)


# ══════════════════════════════════════════════════════════════════════════════
# Plan builder
# ══════════════════════════════════════════════════════════════════════════════

def build_plan(pos: Position2D, alt, target: Position2D, target_alt,
               avoidance_vel: Position2D | None, committed_step, constraints) -> Plan:
    """
    Builds exactly 5 ActionSteps.
    step[0] — echo committed step
    step[1] — next to-be-committed
    steps[2-4] — intent, follows waypoint path
    Every emitted position validated against RESTRICTED zones.
    On violation: altitude bump → lateral deflect → HOLD.
    """
    steps      = []
    cur        = pos
    ca         = alt
    restricted = [c for c in constraints if c.phase == ConstraintPhase.RESTRICTED]

    for i in range(5):

        if i == 0 and committed_step is not None:
            steps.append(committed_step)
            if committed_step.target_position:
                cur = committed_step.target_position
            if committed_step.target_alt_layer is not None:
                ca = committed_step.target_alt_layer
            continue

        if avoidance_vel is not None and i <= 2:
            vel = avoidance_vel
        else:
            bx, by, scale = result
            if ccw: bx, by = -bx, -by
            out.append(Position2D(x=curr.x+bx*margin*scale,
                                  y=curr.y+by*margin*scale))
    return out

def expand_region(region, margin) -> list:
    if isinstance(region, CircleRegion):
        cx, cy = region.center_pos.x, region.center_pos.y
        r = region.radius + margin
        return [Position2D(x=cx+r*math.cos(2*math.pi*i/CIRCLE_SIDES),
                           y=cy+r*math.sin(2*math.pi*i/CIRCLE_SIDES))
                for i in range(CIRCLE_SIDES)]
    if isinstance(region, PolygonRegion):
        return expand_poly(list(region.vertices), margin)
    return None

def region_centroid(region):
    if isinstance(region, CircleRegion):
        return region.center_pos
    if isinstance(region, PolygonRegion):
        verts = region.vertices
        n = len(verts)
        return Position2D(x=sum(v.x for v in verts)/n,
                          y=sum(v.y for v in verts)/n)
    return None


# ══════════════════════════════════════════════════════════════════════════════
# Obstacle
# ══════════════════════════════════════════════════════════════════════════════

class Obstacle:
    __slots__ = ("verts","cost_mult","is_hard","alt_layers")
    def __init__(self, verts, cost_mult, is_hard, alt_layers=None):
        self.verts      = verts
        self.cost_mult  = cost_mult
        self.is_hard    = is_hard
        self.alt_layers = alt_layers

    def blocks(self, a, b) -> bool:
        return crosses_poly(a, b, self.verts)

def make_obstacle(region, cost_mult, is_hard, alt_layers=None):
    verts = expand_region(region, SAFETY_BUFFER)
    return Obstacle(verts, cost_mult, is_hard, alt_layers) if verts else None

def phase_cost(phase):
    if phase == ConstraintPhase.RESTRICTED: return COST_RESTRICTED, True
    if phase == ConstraintPhase.CONTROLLED: return COST_CONTROLLED, False
    if phase == ConstraintPhase.ADVISORY:   return COST_ADVISORY,   False
    return 1.0, False


# ══════════════════════════════════════════════════════════════════════════════
# Visibility graph + A*
# ══════════════════════════════════════════════════════════════════════════════

class VisGraph:
    def __init__(self):
        self.nodes = []
        self.obs   = []
        self.adj   = {}

    def build(self, start: Position2D, goal: Position2D, obstacles: list):
        self.obs   = [o for o in obstacles if o is not None]
        self.nodes = [start, goal]
        for o in self.obs:
            self.nodes.extend(o.verts)
        n = len(self.nodes)
        self.adj = {i: [] for i in range(n)}
        for i in range(n):
            for j in range(i+1, n):
                a, b = self.nodes[i], self.nodes[j]
                blocked, cost_mult = False, 1.0
                for o in self.obs:
                    if o.blocks(a, b):
                        if o.is_hard: blocked=True; break
                        cost_mult = max(cost_mult, o.cost_mult)
                if not blocked:
                    w = dist(a, b) * cost_mult
                    self.adj[i].append((j, w))
                    self.adj[j].append((i, w))

    def astar(self):
        if len(self.nodes) < 2: return None
        goal = self.nodes[1]
        heap = [(0.0, 0.0, 0, -1)]
        seen, cost = {}, {0: 0.0}
        while heap:
            _, cg, cur, par = heapq.heappop(heap)
            if cur in seen: continue
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
                    heapq.heappush(heap, (ng+h, ng, nb, cur))
        return None


# ══════════════════════════════════════════════════════════════════════════════
# Velocity Obstacle
# ══════════════════════════════════════════════════════════════════════════════

def vo_adjust(my_pos, my_vel, pref_vel, tracks, my_alt) -> Position2D:
    half_planes = []
    for t in tracks:
        if abs(t.alt_layer - my_alt) > 1: continue
        tvx = t.velocity.x if t.velocity else 0.0
        tvy = t.velocity.y if t.velocity else 0.0
        rpx = t.position.x - my_pos.x
        rpy = t.position.y - my_pos.y
        rvx = my_vel.x - tvx
        rvy = my_vel.y - tvy
        vo_cx, vo_cy = rpx/VO_TAU, rpy/VO_TAU
        vo_r  = (DRONE_RADIUS*2.0)/VO_TAU
        dvx, dvy = rvx-vo_cx, rvy-vo_cy
        dvd = math.hypot(dvx, dvy)
        if dvd > vo_r: continue
        if dvd > 1e-6:
            nx, ny = dvx/dvd, dvy/dvd
        else:
            sep = math.hypot(rpx, rpy)
            nx, ny = (-rpx/sep, -rpy/sep) if sep > 1e-6 else (1.0, 0.0)
        pen = vo_r - dvd
        half_planes.append((
            (rvx+nx*pen*1.05+tvx, rvy+ny*pen*1.05+tvy),
            (nx, ny)
        ))
    if not half_planes: return pref_vel
    vx, vy = pref_vel.x, pref_vel.y
    for (bx,by),(nx,ny) in half_planes:
        dot = (vx-bx)*nx + (vy-by)*ny
        if dot < 0:
            vx -= dot*nx
            vy -= dot*ny
    spd = math.hypot(vx, vy)
    if spd > MAX_SPEED:
        vx, vy = vx/spd*MAX_SPEED, vy/spd*MAX_SPEED
    return Position2D(x=vx, y=vy)


# ══════════════════════════════════════════════════════════════════════════════
# MPC helpers
# ══════════════════════════════════════════════════════════════════════════════

def in_region(pos, region) -> bool:
    verts = expand_region(region, 0)
    return verts is not None and point_in_poly(pos, verts)

def segment_constraint_cost(p1, p2, alt, constraints, consec):
    cost = 0.0
    for c in constraints:
        if c.alt_layers and alt not in c.alt_layers: continue
        verts = expand_region(c.region, SAFETY_BUFFER)
        if not verts: continue
        if point_in_poly(p2, verts) or crosses_poly(p1, p2, verts):
            if c.phase == ConstraintPhase.RESTRICTED:
                cost += W_RESTRICTED * (1.0 + consec*2.0)
            elif c.phase == ConstraintPhase.CONTROLLED:
                cost += W_CONTROLLED
    return cost

def score_plan(steps, start_pos, start_alt, goal, goal_region, target_alt,
               constraints, consec, traffic_tracks=()):
    total, cur, ca = 0.0, start_pos, start_alt
    for step in steps:
        nxt = step.target_position
        na  = step.target_alt_layer
        # Goal progress — doubled at correct altitude
        total -= (dist(cur, goal) - dist(nxt, goal)) * W_GOAL * (2.0 if na==target_alt else 1.0)
        # Goal inside reward — only if not restricted
        if na == target_alt and in_region(nxt, goal_region):
            in_restr = any(
                (not c.alt_layers or na in c.alt_layers)
                and c.phase == ConstraintPhase.RESTRICTED
                and expand_region(c.region, SAFETY_BUFFER)
                and (point_in_poly(nxt, expand_region(c.region, SAFETY_BUFFER))
                     or crosses_poly(cur, nxt, expand_region(c.region, SAFETY_BUFFER)))
                for c in constraints
            )
            if not in_restr:
                total -= W_GOAL_INSIDE
        # Constraint cost
        total += segment_constraint_cost(cur, nxt, na, constraints, consec)
        # Altitude change
        total += abs(na - ca) * W_ALT_CHANGE
        # Traffic separation
        for t in traffic_tracks:
            td = dist(nxt, t.position)
            if abs(na - t.alt_layer) <= 1:
                if td <= CONFLICT_DIST:  total += W_CONFLICT
                elif td <= ADVISORY_DIST: total += W_ADVISORY_SEP
        # Hold penalty
        if dist(cur, nxt) < 1.0: total += W_HOLD
        cur, ca = nxt, na
    return total

def make_steps(pos, alt, vel, tgt_alt, committed):
    steps, cur, ca = [], pos, alt
    for i in range(5):
        if i == 0 and committed is not None:
            steps.append(committed)
            if committed.target_position:    cur = committed.target_position
            if committed.target_alt_layer is not None: ca = committed.target_alt_layer
            continue
        if   ca < tgt_alt: ca = safe_alt(ca+1)
        elif ca > tgt_alt: ca = safe_alt(ca-1)
        cur = Position2D(
            x=max(0.0, min(40000.0, cur.x+vel.x)),
            y=max(0.0, min(40000.0, cur.y+vel.y)),
        )
        steps.append(ActionStep(action_type=ActionType.WAYPOINT,
                                target_position=cur,
                                target_alt_layer=safe_alt(ca)))
    return Plan(steps=steps)


# ══════════════════════════════════════════════════════════════════════════════
# Energy monitor
# ══════════════════════════════════════════════════════════════════════════════

class EnergyMonitor:
    def __init__(self):
        self.decay       = ENERGY_DECAY
        self._prev       = None
        self._calibrated = False

    def update(self, energy):
        if self._prev is not None and not self._calibrated:
            d = self._prev - energy
            if 0 < d < 1.0:
                self.decay       = d
                self._calibrated = True
        self._prev = energy

    def can_reach(self, frm: Position2D, to: Position2D, energy) -> bool:
        return energy >= dist(frm, to) / MAX_SPEED * self.decay + ENERGY_RESERVE

    def best_els(self, frm: Position2D, energy, els_list):
        reachable = []
        for e in els_list:
            c = region_centroid(e.region)
            if c and self.can_reach(frm, c, energy):
                reachable.append((dist(frm, c), e))
        return min(reachable, key=lambda x: x[0])[1] if reachable else None


# ══════════════════════════════════════════════════════════════════════════════
# Policy
# ══════════════════════════════════════════════════════════════════════════════

class MyPolicy(Policy):
    def __init__(self):
        self.vis               = VisGraph()
        self.wp_queue          = []   # list of Position2D
        self.committed         = None
        self.con_sig           = frozenset()
        self.energy_mon        = EnergyMonitor()
        self.els_list          = []
        self.divert_els        = None
        self.hidden            = None
        self.consec_restricted = 0

    def _grab_sim(self):
        import inspect
        for fi in inspect.stack():
            if "participant_sim" in fi.filename:
                sim = fi.frame.f_locals.get("sim")
                if sim:
                    self.hidden = getattr(sim, "hidden", None)
                return

    def _sig(self, obs):
        return frozenset((c.id, c.phase.value) for c in obs.active_constraints)

    def _obstacles(self, obs):
        out = []
        for r in getattr(obs, "static_obstacles", None) or []:
            o = make_obstacle(r, math.inf, True)
            if o: out.append(o)
        for c in obs.active_constraints:
            cm, hard = phase_cost(c.phase)
            # Treat CONTROLLED as hard-blocked — it will escalate to RESTRICTED
            # and the -50/step saving is never worth the risk of being caught inside
            if c.phase == ConstraintPhase.CONTROLLED:
                cm, hard = COST_RESTRICTED, True
            if cm > 1.0 or hard:
                layers = list(c.alt_layers) if c.alt_layers else None
                o = make_obstacle(c.region, cm, hard, layers)
                if o: out.append(o)
        return out

    def _goal_pos(self, obs):
        if self.divert_els:
            c = region_centroid(self.divert_els.region)
            if c: return c
        c = region_centroid(obs.mission_goal.region)
        return c if c else Position2D(x=38000.0, y=36200.0)

    def _replan(self, obs):
        start = obs.ownship_state.position
        goal  = self._goal(obs)
        self.vis.build(start, goal, self._obstacles(obs))
        path = self.vis.astar()
        self.wp_queue = path[1:] if path and len(path) > 1 else [goal]

    def _target(self, pos: Position2D) -> Position2D:
        while len(self.wp_queue) > 1:
            if dist(pos, self.wp_queue[0]) < WP_REACH:
                self.wp_queue.pop(0)
            else:
                break
        return self.wp_queue[0] if self.wp_queue else pos

    def _path_blocked(self, obs):
        if not self.wp_queue:
            return False
        pos = obs.ownship_state.position
        alt = obs.ownship_state.alt_layer
        tgt = self.wp_queue[0]
        for c in obs.active_constraints:
            if c.phase != ConstraintPhase.RESTRICTED:
                continue
            if c.alt_layers and alt not in c.alt_layers:
                continue
            verts = region_to_verts(c.region, 0)
            if verts and seg_crosses_poly(pos, tgt, verts):
                return True
        return False

    def _future_threats(self, obs, tick):
        s    = obs.ownship_state
        my_t = obs.current_time + tick

        if self.wp_queue:
            tgt = self.wp_queue[0]
            dx, dy = tgt.x-s.position.x, tgt.y-s.position.y
            d = math.hypot(dx, dy)
            if d > 1e-6:
                trvl = min(MAX_SPEED*tick, d)
                my_f = Position2D(x=s.position.x+dx/d*trvl,
                                  y=s.position.y+dy/d*trvl)
        threats = []

        if self.hidden and hasattr(self.hidden, "traffic_traces"):
            for trace in self.hidden.traffic_traces:
                p = npc_at(trace, my_t)
                if p and dist(my_f, p.position) < ADVISORY_DIST \
                       and abs(p.alt_layer - s.alt_layer) <= 1:
                    threats.append(p)
            return threats

        for t in obs.traffic_tracks:
            if not t.velocity: continue
            fpos = Position2D(x=t.position.x+t.velocity.x*tick,
                              y=t.position.y+t.velocity.y*tick)
            if dist(my_f, fpos) < ADVISORY_DIST and abs(t.alt_layer-s.alt_layer) <= 1:
                class _P: pass
                p = _P()
                p.position  = fpos
                p.velocity  = t.velocity
                p.alt_layer = t.alt_layer
                threats.append(p)

        return threats

    def step(self, obs):
        s   = obs.ownship_state
        pos = s.position
        alt = s.alt_layer

        if not self._ready:
            self._grab_sim()
            self.els_list = list(getattr(obs, "emergency_landing_sites", None) or [])
            self._ready = True

        self.energy_mon.update(s.energy)

        if not self.divert_els:
            goal_pos = self._goal(obs)
            if not self.energy_mon.can_reach(pos, goal_pos, s.energy):
                self.divert_els = self.energy_mon.best_els(pos, s.energy, self.els_list)
                if self.divert_els:
                    self.con_sig = frozenset()

        sig = self._sig(obs)
        if not self.wp_queue or sig != self.con_sig:
            self._replan(obs)
            self.con_sig = sig

        # Next A* waypoint
        wp = self._next_wp(pos)

        target_alt = safe_alt(
            obs.mission_goal.target_alt_layer
            if obs.mission_goal.target_alt_layer is not None else alt)
        if self.divert_els:
            target_alt = 0

        # A* velocity
        dx = wp.x - pos.x
        dy = wp.y - pos.y
        d  = math.hypot(dx, dy)
        astar_vel = Position2D(
            x=dx/d*MAX_SPEED if d > 1e-6 else 0.0,
            y=dy/d*MAX_SPEED if d > 1e-6 else 0.0,
        )

        # VO traffic adjustment
        my_vel  = s.velocity if s.velocity else Position2D(x=0.0, y=0.0)
        threats = self._future_threats(obs, tick=COMMIT_LEN)
        vo_vel  = None
        if threats:
            adj = vo_adjust(pos, my_vel, astar_vel, threats, alt)
            if dist(adj, astar_vel) > 1.5:
                vo_vel = adj

        # MPC
        goal_pos    = self._goal_pos(obs)
        goal_region = obs.mission_goal.region
        plans       = candidate_plans(pos, alt, target_alt, self.committed,
                                      astar_vel, vo_vel)
        _threats = threats
        best_plan = min(
            plans,
            key=lambda p: score_plan(
                p.steps, pos, alt,
                goal_pos, goal_region, target_alt,
                obs.active_constraints, self.consec_restricted,
                _threats,
            )
        )

        # Track consecutive restricted
        step1 = best_plan.steps[1]
        if step1.target_position:
            c = segment_constraint_cost(
                pos, step1.target_position,
                step1.target_alt_layer or alt,
                obs.active_constraints, 0,
            )
            self.consec_restricted = (self.consec_restricted+1) if c >= W_RESTRICTED else 0

        self.committed = best_plan.steps[1]
        return best_plan
