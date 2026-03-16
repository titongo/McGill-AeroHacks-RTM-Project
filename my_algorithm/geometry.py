from aerohacks.core.models import Position2D, CircleRegion, PolygonRegion
from dataclasses import replace
import math
from consts import *


def dist(a: Position2D, b: Position2D) -> float:
    return math.hypot(b.x - a.x, b.y - a.y)

def safe_alt(a, min_alt=MIN_ALT, max_alt=MAX_ALT) -> int:
    return max(min_alt, min(max_alt, int(a)))

def outerprod(o: Position2D, a: Position2D, b: Position2D) -> float:
    """Signed area of triangle O→A→B. Positive = B left of O→A."""
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)

def crosses(p1: Position2D, p2: Position2D,
            p3: Position2D, p4: Position2D) -> bool:
    d1 = outerprod(p3, p4, p1)
    d2 = outerprod(p3, p4, p2)
    d3 = outerprod(p1, p2, p3)
    d4 = outerprod(p1, p2, p4)
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

def crosses_poly(p1: Position2D, p2: Position2D, verts: list) -> bool:
    n = len(verts)
    for i in range(n):
        if crosses(p1, p2, verts[i], verts[(i + 1) % n]):
            return True
    return point_in_poly(p1, verts) or point_in_poly(p2, verts)

def corner_bisector(prev: Position2D, curr: Position2D, nxt: Position2D):
    """
    Returns (bx, by, scale) — outward unit bisector at curr and
    1/sin(half-angle) scale so margin*scale gives uniform edge clearance.
    Returns None if either edge is degenerate.
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
        bx, by = -e1y, e1x   # anti-parallel — push perpendicular
    else:
        bx, by = bx / bd, by / bd

    dot      = e1x * e2x + e1y * e2y
    sin_half = math.sqrt(max(0.0, (1.0 - dot) / 2.0))
    scale    = 1.0 / sin_half if sin_half > 0.1 else 10.0

    return bx, by, scale

def signed_area(verts) -> float:
    """Positive = CCW, negative = CW (standard math coords, y-up)."""
    n, a = len(verts), 0.0
    for i in range(n):
        j = (i + 1) % n
        a += verts[i].x * verts[j].y - verts[j].x * verts[i].y
    return a * 0.5


def expand_poly(verts: list, margin: float) -> list:
    """
    Expand each vertex outward along the corner bisector by margin.
    Detects polygon winding order so the bisector always points away
    from the interior, regardless of how the vertices are wound.
    For CCW polygons the raw (e1+e2) bisector points inward — it is negated.
    """
    ccw = signed_area(verts) > 0
    n   = len(verts)
    out = []
    for i in range(n):
        curr   = verts[i]
        result = corner_bisector(verts[(i - 1) % n], curr, verts[(i + 1) % n])
        if result is None:
            out.append(curr)
        else:
            bx, by, scale = result
            if ccw:
                bx, by = -bx, -by   # flip to outward for CCW winding
            out.append(Position2D(x=curr.x + bx * margin * scale,
                                  y=curr.y + by * margin * scale))
    return out

def expand_circle(circle: CircleRegion, margin: float) -> CircleRegion:
    return replace(circle, radius=circle.radius + margin)

def circle_to_verts(circle: CircleRegion) -> list:
    cx, cy = circle.center_pos.x, circle.center_pos.y
    r = circle.radius
    return [Position2D(x=cx + r * math.cos(2 * math.pi * i / CIRCLE_SIDES),
                       y=cy + r * math.sin(2 * math.pi * i / CIRCLE_SIDES))
            for i in range(CIRCLE_SIDES)]

def expand_region(region, margin: float) -> list | None:
    if isinstance(region, CircleRegion):
        return circle_to_verts(expand_circle(region, margin))
    if isinstance(region, PolygonRegion):
        return expand_poly(list(region.vertices), margin)
    return None

def region_centroid(region) -> Position2D | None:
    if isinstance(region, CircleRegion):
        return region.center_pos
    if isinstance(region, PolygonRegion):
        verts = region.vertices
        n     = len(verts)
        return Position2D(x=sum(v.x for v in verts) / n,
                          y=sum(v.y for v in verts) / n)
    return None
