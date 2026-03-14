# AeroHacks RTM Participant Guide (Go-To README)

This document is the **primary reference for participants**.

If you are unsure about anything (policy API, observation format, scoring, visualizer usage, custom scenarios), start here.

---

## 1) RTM challenge overview (why this matters)

RTM means **RPAS Traffic Management**.

In practice, the hardest autonomy problem is not flying one drone. It is coordinating many autonomous aircraft safely and efficiently in shared airspace. This challenge is a realistic mini-version of that problem:

- Dynamic constraints (NOTAM phases)
- Moving traffic with evolving trajectories
- Limited energy and mission urgency
- Strict safety and compliance penalties

Winning solutions are not just fast. They are **safe, robust, and rule-compliant**.

---

## 2) Package contents (full participant package)

```text
participant_package/
├── AeroHacksSim
├── my_algorithm/
│   ├── __init__.py
│   └── policy.py
├── view_playback.py
├── requirements-viz.txt
├── scenarios/
│   ├── public/
│   │   └── example_training.json
│   └── hidden/
│       └── example_training.json
├── models_reference.py
├── playback.json
└── README.md
```

---

## 3) Quick start (first run)

### Step 1: Implement your policy
Edit:

- `my_algorithm/policy.py`

Required interface:

- `class MyPolicy(Policy)`
- `step(observation: Observation) -> Plan`

### Step 2: Run simulation
```bash
# macOS / Linux
./AeroHacksSim --policy ./my_algorithm --scenario example_training

# Windows
AeroHacksSim.exe --policy ./my_algorithm --scenario example_training
```

### Step 3: Check outputs
- Final score/status in console
- `playback.json` with time-series flight state

---

## 4) Exact policy contract (strict)

At every simulation tick:
1. Simulator creates an `Observation` Python object.
2. Simulator calls `MyPolicy.step(observation)`.
3. You must return a `Plan` containing **exactly 5** `ActionStep` items.
4. Simulator executes actions with commitment and kinematic limits, then scores.

If your output is invalid (wrong type, wrong step count, malformed action fields), the run can terminate with catastrophic failure.

---

## 5) Observation format (exact runtime type)

### Important
Your `step()` receives a **Python dataclass object**, not a JSON string.
Use attribute access directly.

### Core fields
| Field | Type | Meaning |
|---|---|---|
| `observation.current_time` | `int` | Current simulation tick |
| `observation.ownship_state` | `State` | Ownship position / altitude / energy / velocity |
| `observation.mission_goal` | `GoalRegion` | Goal region + target altitude |
| `observation.active_constraints` | `List[Constraint]` | Active permanent + NOTAM constraints |
| `observation.traffic_tracks` | `List[TrafficTrack]` | Nearby traffic states and short intents |

### Practical usage example
```python
s = observation.ownship_state
current_pos = s.position
current_alt = s.alt_layer
energy = s.energy

goal = observation.mission_goal
constraints = observation.active_constraints
traffic = observation.traffic_tracks

if traffic:
    intruder = traffic[0]
    intruder_xy = (intruder.position.x, intruder.position.y)
```

---

## 6) What you are allowed to return (`Plan`)

You must return:

- `Plan(steps=[ActionStep, ActionStep, ActionStep, ActionStep, ActionStep])`

Exactly 5 steps every tick.

### `ActionType.WAYPOINT`
Use for directed movement.

Fields:
- `target_position: Position2D | None`
- `target_alt_layer: int | None`

Validation:
- At least one of `target_position` or `target_alt_layer` must be set.

Execution behavior:
- Horizontal motion is clamped by max horizontal speed.
- Vertical change is clamped by max vertical rate.
- If target is far, simulator moves partially toward target this tick.

Example:
```python
ActionStep(
    action_type=ActionType.WAYPOINT,
    target_position=Position2D(x=12000.0, y=8300.0),
    target_alt_layer=2,
)
```

### `ActionType.HOLD`
- Keeps current position/altitude for the tick.
- Energy still decays.

### `ActionType.EMERGENCY_LAND`
- Holds horizontal position.
- Descends toward altitude layer 0.

---

## 7) Graphic visualizer + playback workflow

This is the recommended debugging loop.

### 7.1 Generate playback
Run simulation first, then confirm `playback.json` is updated.

### 7.2 Launch visualizer
```bash
python -m pip install -r requirements-viz.txt
python view_playback.py
```

### 7.3 Launch with full overlays (recommended)
```bash
python view_playback.py \
  --playback playback.json \
  --scenario scenarios/public/example_training.json \
  --hidden scenarios/hidden/example_training.json
```

This overlay mode helps you inspect:
- Goal/start geometry
- Permanent constraints and static obstacles
- Dynamic NOTAM phase changes
- NPC traffic positions and short trails

### 7.4 Viewer controls
- Slider drag: jump to any frame
- Left/Right arrows: frame step
- Home/End: first/last frame

---

## 8) Add your own scenarios (important for robustness)

You can and should create your own test scenarios.

### Required folder structure
```text
scenarios/
├── public/my_case.json
└── hidden/my_case.json
```

Both files must share the same base name (`my_case`).

Important:
- Use the **same scenario name** in both folders (`my_case`),
- but do **not** copy the exact same JSON content into both files.

`public/my_case.json` and `hidden/my_case.json` have different roles/schemas:
- `public`: map boundaries, start state, goal, static constraints, scoring config
- `hidden`: dynamic NOTAM timing + traffic traces

### Minimal shape examples (schema intent)
`public/my_case.json` (high-level fields):
```json
{
  "scenario_id": "my_case",
  "map_boundaries": { "vertices": [] },
  "start_state": { "position": { "x": 0, "y": 0 }, "alt_layer": 1, "energy": 350 },
  "mission_goal": { "region": { "center_pos": { "x": 1000, "y": 1000 }, "radius": 200 }, "target_alt_layer": 1 },
  "permanent_constraints": [],
  "static_obstacles": [],
  "emergency_landing_sites": [],
  "vehicle_limits": {},
  "scoring_config": {}
}
```

`hidden/my_case.json` (high-level fields):
```json
{
  "shrinking_notams": [],
  "traffic_traces": []
}
```

### Run custom scenario
```bash
./AeroHacksSim --policy ./my_algorithm --scenarios-dir ./scenarios --scenario my_case
```

### Override hidden file explicitly
```bash
./AeroHacksSim --policy ./my_algorithm --scenario my_case --hidden ./scenarios/hidden/custom_hidden.json
```

### Why this matters
Teams that only test one easy scenario usually fail on evaluation cases. Use custom scenarios to stress:
- Dense traffic
- Late NOTAM phase changes
- Tight energy margins
- Goal approach under constraint pressure

---

## 9) Scoring (code-aligned)

### Start value
- Simulation starts at `base_success_score = +1000`.

### Positive outcomes
- Goal reached at required target altitude:
  - terminal success
  - time bonus:
  - `time_bonus = time_bonus_multiplier * max(0, max_time - current_step)`
  - default multiplier is `10`
- Safe emergency landing at designated site (alt layer 0):
  - adjusted partial-credit formula:
  - `score = score - base_success_score + safe_emergency_landing_score`
  - default safe emergency landing score is `300`

### Per-step / per-event penalties
- Controlled airspace violation: `-50`
- Restricted airspace violation: `-200`
- Advisory separation loss: `-10`
  - distance `<= 100m` and altitude diff `<= 1`
- Conflict separation loss: `-150`
  - distance `<= 50m` and altitude diff `<= 1`

### Catastrophic conditions
- Collision (`<= 20m` and same altitude)
- Battery depletion outside valid safe-emergency resolution
- `>= 5` consecutive restricted steps
- Out-of-boundary or static-obstacle collision

Important:
- Catastrophic run sets final simulator score to **`0.0`**.

---

## 10) CLI reference

```bash
./AeroHacksSim --policy ./my_algorithm
./AeroHacksSim --policy ./my_algorithm --no-viz
./AeroHacksSim --policy ./my_algorithm --output results/
./AeroHacksSim --policy ./my_algorithm --scenario my_case
./AeroHacksSim --policy ./my_algorithm --scenarios-dir ./scenarios --scenario my_case
./AeroHacksSim --policy ./my_algorithm --scenario my_case --hidden ./scenarios/hidden/custom_hidden.json
```

Notes:
- `--scenario` resolves `<scenarios-dir>/public/<name>.json` plus matching hidden file.
- `--hidden` overrides hidden file path directly.

---

## 11) Minimal policy skeleton

```python
from aerohacks.policy.base import Policy
from aerohacks.core.models import Observation, Plan, ActionStep, ActionType, Position2D

class MyPolicy(Policy):
    def step(self, observation: Observation) -> Plan:
        s = observation.ownship_state
        return Plan(steps=[
            ActionStep(ActionType.WAYPOINT, Position2D(s.position.x + 10.0, s.position.y), s.alt_layer),
            ActionStep(ActionType.HOLD),
            ActionStep(ActionType.HOLD),
            ActionStep(ActionType.HOLD),
            ActionStep(ActionType.HOLD),
        ])
```

---

## 12) Troubleshooting / FAQ

- **"Is observation JSON?"** No. In your policy, it is a Python dataclass object.
- **"MyPolicy not found"** Ensure `class MyPolicy(Policy):` exists in `my_algorithm/policy.py`.
- **Plan-length errors** Return exactly 5 valid `ActionStep`s each tick.
- **No movement** Ensure `WAYPOINT` targets are meaningful and evolve over time.
- **Type/import issues** Use model types from `aerohacks.core.models`; check `models_reference.py`.
- **macOS/Linux execute error** Run: `chmod +x AeroHacksSim`.
