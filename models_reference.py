"""
AeroHacks RTM — Models Reference (READ ONLY)

This file contains the data type definitions used by the simulator.
Use these for type hints and IDE autocompletion in your policy.py.

DO NOT MODIFY THIS FILE — it is provided for reference only.
The actual types are bundled inside the AeroHacksSim binary.

Your policy.py should import types like:
    from aerohacks.policy.base import Policy
    from aerohacks.core.models import Observation, Plan, ActionStep, ActionType, Position2D
"""

# ==============================================================================
# The following are the key types you need to know:
# ==============================================================================

# ─── Position2D ───────────────────────────────────────────────────────────────
# @dataclass
# class Position2D:
#     x: float
#     y: float

# ─── State ────────────────────────────────────────────────────────────────────
# @dataclass
# class State:
#     position: Position2D
#     alt_layer: int          # Discrete altitude layer (0 = ground)
#     energy: float           # Remaining battery energy
#     velocity: Position2D    # Current velocity vector (optional)
#     heading: float          # Current heading in radians (optional)

# ─── ActionType ───────────────────────────────────────────────────────────────
# class ActionType(Enum):
#     WAYPOINT = "WAYPOINT"           # Move to target position/altitude
#     HOLD = "HOLD"                   # Hover in place
#     EMERGENCY_LAND = "EMERGENCY_LAND"  # Descend to ground at current position

# ─── ActionStep ───────────────────────────────────────────────────────────────
# @dataclass
# class ActionStep:
#     action_type: ActionType
#     target_position: Optional[Position2D] = None     # Required for WAYPOINT
#     target_alt_layer: Optional[int] = None            # Optional altitude change

# ─── Plan ─────────────────────────────────────────────────────────────────────
# @dataclass
# class Plan:
#     steps: List[ActionStep]     # Must contain EXACTLY 5 ActionStep items

# ─── ConstraintPhase ──────────────────────────────────────────────────────────
# class ConstraintPhase(Enum):
#     ADVISORY = "ADVISORY"           # Warning only, minor penalty
#     CONTROLLED = "CONTROLLED"       # Active penalty zone
#     RESTRICTED = "RESTRICTED"       # Heavy penalty, 5+ consecutive steps = catastrophic

# ─── Constraint ───────────────────────────────────────────────────────────────
# @dataclass
# class Constraint:
#     id: str
#     region: Region                  # PolygonRegion or CircleRegion
#     alt_layers: List[int]           # Affected altitude layers
#     phase: ConstraintPhase
#     start_time: int
#     end_time: Optional[int]

# ─── TrafficTrack ─────────────────────────────────────────────────────────────
# @dataclass
# class TrafficTrack:
#     id: str
#     position: Position2D            # Current position
#     alt_layer: int                  # Current altitude layer
#     velocity: Optional[Position2D]  # Velocity vector
#     intent: Optional[List[Position2D]]  # Future positions (predicted)

# ─── GoalRegion ───────────────────────────────────────────────────────────────
# @dataclass
# class GoalRegion:
#     region: Region                  # Target area (polygon or circle)
#     target_alt_layer: Optional[int] # Required altitude to complete mission

# ─── Observation (this is what your step() receives) ──────────────────────────
# @dataclass
# class Observation:
#     current_time: int                       # Current simulation tick
#     ownship_state: State                    # Your drone's current state
#     mission_goal: GoalRegion                # Where you need to go
#     active_constraints: List[Constraint]    # Currently active airspace constraints
#     traffic_tracks: List[TrafficTrack]      # Other drones in the airspace

# ─── Policy (base class you must extend) ──────────────────────────────────────
# class Policy(ABC):
#     @abstractmethod
#     def step(self, observation: Observation) -> Plan:
#         """Return a Plan with exactly 5 ActionSteps."""
#         pass

# ==============================================================================
# Scoring Summary
# ==============================================================================
# - Base success score: 1000 points
# - Time bonus: faster completion = more points
# - Penalties:
#     Controlled airspace:    -50 per step
#     Restricted airspace:   -200 per step
#     Advisory separation:    -10 per event
#     Conflict separation:   -150 per event
#     Collision:             -500 (catastrophic, terminates run)
#     Battery depletion:     -500 (catastrophic, terminates run)
# - 5+ consecutive restricted steps = catastrophic violation (score zeroed)
# - Emergency landing at designated site = partial credit (300 points)

# ==============================================================================
# Separation Thresholds
# ==============================================================================
# - Collision:  distance <= 20m AND same altitude layer
# - Conflict:   distance <= 50m AND altitude difference <= 1 layer
# - Advisory:   distance <= 100m AND altitude difference <= 1 layer
