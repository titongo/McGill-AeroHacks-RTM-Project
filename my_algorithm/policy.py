from aerohacks.policy.base import Policy
from aerohacks.core.models import Observation, Plan, ActionStep, ActionType, Position2D
import math

MIN_NORMAL_ALT = 1
MAX_NORMAL_ALT = 4
ADVISORY_SEPARATION = 100.0 # in meters, this is the threshold for triggering a detour around traffic
DETOUR_OFFSET = 150.0 # in meters, how far to the side to step when detouring around traffic

def clamp_normal_alt(alt_layer: int) -> int:
    return max(MIN_NORMAL_ALT, min(MAX_NORMAL_ALT, int(alt_layer)))

def distance_2D(a: Position2D, b: Position2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)

def near_traffic(pos: Position2D, alt: int, traffic_tracks) -> bool:
    """Return True if pos is within ADVISORY_SEPARATION of any traffic at the same altitude layer."""
    for t in traffic_tracks:
        if t.alt_layer == alt and distance_2D(pos, t.position) <= ADVISORY_SEPARATION:
            return True
    return False

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

        desired_alt = (
            obs.mission_goal.target_alt_layer
            if obs.mission_goal.target_alt_layer is not None
            else obs.ownship_state.alt_layer
        )
        safe_alt = clamp_normal_alt(desired_alt)

        dx = goal_pos.x - current_pos.x
        dy = goal_pos.y - current_pos.y
        dist = math.hypot(dx, dy)
        speed = min(15.0, dist)

        # Build unit vector toward the goal for detour direction calculations
        if dist > 0:
            ux, uy = dx / dist, dy / dist
        else:
            ux, uy = 1.0, 0.0

        perp_left = (-uy, ux)
        perp_right = (uy, -ux)

        next_pos = current_pos
        for _ in range(5):
            remaining_dx = goal_pos.x - next_pos.x
            remaining_dy = goal_pos.y - next_pos.y
            remaining_dist = math.hypot(remaining_dx, remaining_dy)

            if remaining_dist > 0:
                step_dist = min(speed, remaining_dist)
                rdx, rdy = remaining_dx / remaining_dist, remaining_dy / remaining_dist
                candidate = Position2D(
                    x=next_pos.x + rdx * step_dist,
                    y=next_pos.y + rdy * step_dist,
                )
            else:
                candidate = Position2D(x=goal_pos.x, y=goal_pos.y)

            if not near_traffic(candidate, safe_alt, traffic):
                # Direct path is clear -> proceed toward the goal
                next_pos = candidate
                steps.append(ActionStep(
                    action_type=ActionType.WAYPOINT,
                    target_position=next_pos,
                    target_alt_layer=safe_alt,
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
                # Check left detour first, then right
                if not near_traffic(left_pos, safe_alt, traffic):
                    next_pos = left_pos
                    steps.append(ActionStep(
                        action_type=ActionType.WAYPOINT,
                        target_position=next_pos,
                        target_alt_layer=safe_alt,
                    ))
                elif not near_traffic(right_pos, safe_alt, traffic):
                    next_pos = right_pos
                    steps.append(ActionStep(
                        action_type=ActionType.WAYPOINT,
                        target_position=next_pos,
                        target_alt_layer=safe_alt,
                    ))
                else:
                    # Both detours also blocked — hold in place this step
                    steps.append(ActionStep(action_type=ActionType.HOLD))
                    # next_pos remains unchanged, so try again from the same spot

        return Plan(steps=steps)