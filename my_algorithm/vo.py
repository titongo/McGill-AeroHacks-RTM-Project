from aerohacks.core.models import Position2D
from consts import *
import math


def vo_adjust(my_pos: Position2D, my_vel: Position2D, pref_vel: Position2D,
              tracks, my_alt) -> Position2D:
    """
    For each traffic in threat range compute the VO cone half-plane constraint,
    then project preferred velocity to the nearest safe point.
    """
    half_planes = []

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
