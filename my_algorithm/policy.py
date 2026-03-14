from aerohacks.policy.base import Policy # type: ignore
from aerohacks.core.models import Observation, Plan, ActionStep, ActionType, Position2D # type: ignore
import math

class MyPolicy(Policy):
    """
    BASELINE ALGORITHM: This is your starting point.
    It simply flies in a straight line towards the goal, ignoring any NO-FLY zones or traffic.
    You will need to improve this to avoid penalties!
    """
    
    def step(self, obs: Observation) -> Plan:
        print(obs.active_constraints)  # Debug: Print active constraints to understand the environment
        print(obs.traffic_tracks)  # Debug: Print traffic tracks to see other drones in the airspace

        steps = []
        
        # We need to output exactly 5 steps into the future.
        current_pos = obs.ownship_state.position
        goal_pos = obs.mission_goal.region.center()
        
        # Calculate angle to goal
        dx = goal_pos.x - current_pos.x
        dy = goal_pos.y - current_pos.y
        dist = math.hypot(dx, dy)
        
        # Maximum speed per step is 15.0m
        speed = min(15.0, dist)
        
        # PR-FIX: Build waypoints incrementally and cap at goal to prevent overshoot
        next_pos = current_pos
        for i in range(5):
            remaining_dx = goal_pos.x - next_pos.x
            remaining_dy = goal_pos.y - next_pos.y
            remaining_dist = math.hypot(remaining_dx, remaining_dy)

            if remaining_dist > 0:
                step_distance = min(speed, remaining_dist)
                step_x = next_pos.x + (remaining_dx / remaining_dist) * step_distance
                step_y = next_pos.y + (remaining_dy / remaining_dist) * step_distance
            else:
                step_x, step_y = goal_pos.x, goal_pos.y

            next_pos = Position2D(x=step_x, y=step_y)
                
            steps.append(
                ActionStep(
                    action_type=ActionType.WAYPOINT,
                    target_position=next_pos,
                    target_alt_layer=obs.mission_goal.target_alt_layer if obs.mission_goal.target_alt_layer is not None else obs.ownship_state.alt_layer
                )
            )
            
        return Plan(steps=steps)
