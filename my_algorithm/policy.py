from aerohacks.policy.base import Policy #type: ignore
from aerohacks.core.models import Observation, Plan, ActionStep, ActionType, Position2D #type: ignore
import math

# Proposed changes:
# - If altitude change is needed:
#   - Determine max vertical limit for the next step (e.g. 1 layer per step)
#   - Calculate number of steps needed to reach target altitude at max vertical rate
#   - Get distance to first point of contact with traffic / constraint at current altitude along direct path
#   - Divide distance by number of steps needed to get step increment toward the goal that also allows for gradual altitude change to avoid traffic/constraints rather than trying to jump to a different altitude immediately which may not be possible within one step
#   - Create step toward the goal with the calculated increment and altitude change, and save distance and remaining altitude change for the next step to continue adjusting in subsequent steps
# - When detouring around traffic, step to the side (perpendicular to the direct path) rather than just stopping, to more quickly get around the hazard.
# - If both direct and perpendicular paths are blocked, HOLD in place but keep checking for openings each subsequent step rather than giving up entirely. This allows for dynamic response to changing traffic and constraints rather than getting stuck in a HOLD state


MIN_NORMAL_ALT = 1
MAX_NORMAL_ALT = 4
ADVISORY_SEPARATION = 180.0 # in meters, this is the threshold for triggering a detour around traffic
DETOUR_OFFSET = 320.0 # in meters, how far to the side to step when detouring around traffic
MAX_HORIZONTAL_SPEED = 15.0 # in m/s, maximum horizontal speed for calculating step increments
MAX_VERTICAL_LIMIT = 1 # in altitude layers per step, maximum vertical change allowed in a single step

def clamp_normal_alt(alt_layer: int) -> int:
    return max(MIN_NORMAL_ALT, min(MAX_NORMAL_ALT, int(alt_layer)))

def distance_2D(a: Position2D, b: Position2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)

def near_traffic(pos: Position2D, alt: int, traffic_tracks) -> bool:
    # Return True if pos is within ADVISORY_SEPARATION of any traffic at the same altitude layer.
    for t in traffic_tracks:
        if t.alt_layer == alt and distance_2D(pos, t.position) <= ADVISORY_SEPARATION:
            return True
    return False

def point_in_polygon(p: Position2D, vertices) -> bool:
    # Ray-casting point-in-polygon test.
    inside = False
    n = len(vertices)
    if n < 3:
        return False

    px, py = p.x, p.y
    j = n - 1
    for i in range(n):
        xi, yi = vertices[i].x, vertices[i].y
        xj, yj = vertices[j].x, vertices[j].y

        intersects = ((yi > py) != (yj > py)) and (
            px < (xj - xi) * (py - yi) / ((yj - yi) if (yj - yi) != 0 else 1e-9) + xi
        )
        if intersects:
            inside = not inside
        j = i

    return inside

def point_in_region(p: Position2D, region) -> bool:
    # Runtime objects may expose either dataclass fields or a type name; handle both safely.
    region_type = getattr(region, "type", region.__class__.__name__)
    if isinstance(region_type, str):
        region_type = region_type.lower()

    if "circle" in str(region_type).lower():
        center = getattr(region, "center_pos", None)
        radius = getattr(region, "radius", None)
        if center is None or radius is None:
            return False
        return distance_2D(p, center) <= float(radius)

    vertices = getattr(region, "vertices", None)
    if not vertices:
        return False
    return point_in_polygon(p, vertices)

def in_active_constraint(pos: Position2D, alt: int, active_constraints) -> bool:
    # Return true if position is within any active constraint that applies to the given altitude layer
    for constraint in active_constraints:
        alt_layers = getattr(constraint, "alt_layers", [])
        if alt not in alt_layers:
            continue
        region = getattr(constraint, "region", None)
        if region is not None and point_in_region(pos, region):
            return True
    return False

def choose_safe_alt(pos: Position2D, preferred_alt: int, traffic_tracks, active_constraints):
    # Choose a safe altitude layer for the given position; give preference to the preferred_alt but check all layers if needed
    alt_order = [preferred_alt] + [a for a in range(MIN_NORMAL_ALT, MAX_NORMAL_ALT + 1) if a != preferred_alt]
    for alt in alt_order:
        if near_traffic(pos, alt, traffic_tracks):
            continue
        if in_active_constraint(pos, alt, active_constraints):
            continue
        return alt
    return None

class MyPolicy(Policy):
    """
    NEW ALGORITHM:
    Flies toward the goal while actively avoiding nearby traffic.
    On each tick:
      - Direct waypoint is used if clear of traffic.
      - Left/right perpendicular detours are tried if direct path is blocked.
      - HOLD is used as a last resort when all lateral options are also blocked.
    """

    def step(self, obs: Observation) -> Plan:
        steps = []
        current_pos = obs.ownship_state.position
        goal_pos = obs.mission_goal.region.center()
        traffic = obs.traffic_tracks
        active_constraints = obs.active_constraints

        desired_alt = (
            obs.mission_goal.target_alt_layer
            if obs.mission_goal.target_alt_layer is not None
            else obs.ownship_state.alt_layer
        )
        preferred_alt = clamp_normal_alt(desired_alt)

        dx = goal_pos.x - current_pos.x
        dy = goal_pos.y - current_pos.y
        dist = math.hypot(dx, dy)
        speed = min(MAX_HORIZONTAL_SPEED, dist)
        vertical_speed = min(MAX_VERTICAL_LIMIT, abs(preferred_alt - obs.ownship_state.alt_layer))

        # Build unit vector toward the goal for detour direction calculations
        if dist > 0:
            ux, uy = dx / dist, dy / dist
        else:
            ux, uy = 1.0, 0.0

        perp_left = (-uy, ux)
        perp_right = (uy, -ux)

        next_pos = current_pos
        # Plan up to 5 steps ahead, then adjust based on traffic
        for _ in range(5):
            remaining_dx = goal_pos.x - next_pos.x
            remaining_dy = goal_pos.y - next_pos.y
            remaining_dist = math.hypot(remaining_dx, remaining_dy)
            remaining_alt = preferred_alt - obs.ownship_state.alt_layer

            if remaining_dist > 0:
                step_dist = min(speed, remaining_dist)
                rdx, rdy = remaining_dx / remaining_dist, remaining_dy / remaining_dist
                candidate = Position2D(
                    x=next_pos.x + rdx * step_dist,
                    y=next_pos.y + rdy * step_dist,
                )
            else:
                candidate = Position2D(x=goal_pos.x, y=goal_pos.y)

            candidate_alt = choose_safe_alt(candidate, preferred_alt, traffic, active_constraints)
            if candidate_alt is not None:
                # Direct path is clear -> proceed toward the goal
                next_pos = candidate
                steps.append(ActionStep(
                    action_type=ActionType.WAYPOINT,
                    target_position=next_pos,
                    target_alt_layer=candidate_alt,
                ))
            else:
                # Direct path blocked —> try perpendicular detours
                left_pos = Position2D(
                    x=next_pos.x + ux * speed + perp_left[0] * DETOUR_OFFSET,
                    y=next_pos.y + uy * speed + perp_left[1] * DETOUR_OFFSET,
                )
                right_pos = Position2D(
                    x=next_pos.x + ux * speed + perp_right[0] * DETOUR_OFFSET,
                    y=next_pos.y + uy * speed + perp_right[1] * DETOUR_OFFSET,
                )
                left_alt = choose_safe_alt(left_pos, preferred_alt, traffic, active_constraints)
                right_alt = choose_safe_alt(right_pos, preferred_alt, traffic, active_constraints)
                # Check left detour first, then right
                if left_alt is not None:
                    next_pos = left_pos
                    steps.append(ActionStep(
                        action_type=ActionType.WAYPOINT,
                        target_position=next_pos,
                        target_alt_layer=left_alt,
                    ))
                elif right_alt is not None:
                    next_pos = right_pos
                    steps.append(ActionStep(
                        action_type=ActionType.WAYPOINT,
                        target_position=next_pos,
                        target_alt_layer=right_alt,
                    ))
                else:
                    # Both detours also blocked — hold in place this step
                    steps.append(ActionStep(action_type=ActionType.HOLD))
                    # next_pos remains unchanged, so try again from the same spot

        return Plan(steps=steps)