from aerohacks.policy.base import Policy
from aerohacks.core.models import (
    Observation, Plan, ActionStep, ActionType,
    Position2D, ConstraintPhase, PolygonRegion, CircleRegion,
)
import math
from geometry import dist, safe_alt, region_centroid, expand_region, crosses_poly, point_in_poly
from graph import VisGraph
from obstacle import Obstacle, make_obstacle, phase_cost
from vo import vo_adjust
from consts import (
    MAX_SPEED, MIN_ALT, MAX_ALT,
    SAFETY_BUFFER, COMMIT_LEN,
    ADVISORY_DIST, CONFLICT_DIST, WP_REACH,
)

def in_restricted(pos, alt, constraints):
    for c in constraints:
        if c.phase != ConstraintPhase.RESTRICTED:
            continue
        if c.alt_layers and alt not in c.alt_layers:
            continue
        verts = expand_region(c.region, 0)
        if verts and point_in_poly(pos, verts):
            return True
    return False


def deflect(vel, cur, alt, constraints):
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


def build_plan(pos, alt, target, target_alt, avoidance_vel, committed_step, constraints):
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
            dx = target.x - cur.x
            dy = target.y - cur.y
            d  = math.hypot(dx, dy)
            if d > 1e-6:
                s   = min(MAX_SPEED, d)
                vel = Position2D(x=dx / d * s, y=dy / d * s)
            else:
                vel = Position2D(x=0.0, y=0.0)

        if   ca < target_alt: ca = safe_alt(ca + 1)
        elif ca > target_alt: ca = safe_alt(ca - 1)

        nxt = Position2D(x=cur.x + vel.x, y=cur.y + vel.y)

        if restricted and in_restricted(nxt, ca, restricted):
            bumped = safe_alt(ca + 1)
            if bumped != ca and not in_restricted(nxt, bumped, restricted):
                ca = bumped
            else:
                vel = deflect(vel, cur, ca, restricted)
                nxt = Position2D(x=cur.x + vel.x, y=cur.y + vel.y)

        cur = nxt
        steps.append(ActionStep(
            action_type=ActionType.WAYPOINT,
            target_position=cur,
            target_alt_layer=safe_alt(ca),
        ))

    return Plan(steps=steps)

class MyPolicy(Policy):
    def __init__(self):
        self.vis               = VisGraph()
        self.wp_queue          = []
        self.committed         = None
        self.con_sig           = frozenset()
        self.consec_restricted = 0
        self._ready            = False

    def _sig(self, obs):
        return frozenset((c.id, c.phase.value) for c in obs.active_constraints)

    def _obstacles(self, obs):
        out = []
        for r in getattr(obs, "static_obstacles", None) or []:
            o = make_obstacle(r, math.inf, True)
            if o: out.append(o)
        for c in obs.active_constraints:
            cm, hard = phase_cost(c.phase)
            if cm > 1.0 or hard:
                layers = list(c.alt_layers) if c.alt_layers else None
                o = make_obstacle(c.region, cm, hard, layers)
                if o: out.append(o)
        return out

    def _goal(self, obs):
        if self.divert_els:
            c = region_centroid(self.divert_els.region)
            if c: return c
        c = region_centroid(obs.mission_goal.region)
        return c if c else Position2D(x=38000.0, y=36200.0)

    def _replan(self, obs):
        start    = obs.ownship_state.position
        goal     = self._goal(obs)
        obs_list = self._obstacles(obs)

        self.vis.build(start, goal, obs_list)
        path = self.vis.astar()
        if path and len(path) > 1:
            self.wp_queue = path[1:]
            return

        # A* failed — likely inside expanded buffer. Retry with smaller buffers.
        for reduced in (40.0, 20.0, 0.0):
            out = []
            for r in getattr(obs, "static_obstacles", None) or []:
                verts = expand_region(r, reduced)
                if verts: out.append(Obstacle(verts, math.inf, True))
            for c in obs.active_constraints:
                cm, hard = phase_cost(c.phase)
                if cm > 1.0 or hard:
                    verts = expand_region(c.region, reduced)
                    if verts:
                        layers = list(c.alt_layers) if c.alt_layers else None
                        out.append(Obstacle(verts, cm, hard, layers))
            self.vis.build(start, goal, out)
            path = self.vis.astar()
            if path and len(path) > 1:
                self.wp_queue = path[1:]
                return

        # Still nothing — escape away from nearest restricted constraint
        self.wp_queue = [self._escape(obs), goal]

    def _escape(self, obs):
        pos = obs.ownship_state.position
        nearest, nearest_d = None, math.inf
        for c in obs.active_constraints:
            if c.phase != ConstraintPhase.RESTRICTED:
                continue
            cen = region_centroid(c.region)
            if cen:
                d = dist(pos, cen)
                if d < nearest_d:
                    nearest_d, nearest = d, cen
        if nearest is None:
            return self._goal(obs)
        dx = pos.x - nearest.x
        dy = pos.y - nearest.y
        d  = math.hypot(dx, dy)
        if d < 1e-6:
            dx, dy, d = 1.0, 0.0, 1.0
        return Position2D(x=pos.x + dx / d * SAFETY_BUFFER * 2,
                          y=pos.y + dy / d * SAFETY_BUFFER * 2)

    def _target(self, pos):
        while len(self.wp_queue) > 1:
            if dist(pos, self.wp_queue[0]) < WP_REACH:
                self.wp_queue.pop(0)
            else:
                break
        return self.wp_queue[0] if self.wp_queue else pos

    def _path_blocked(self, obs):
        """Only checks constraints newly RESTRICTED since last plan — avoids hover-at-edge loop."""
        if not self.wp_queue:
            return False
        pos = obs.ownship_state.position
        alt = obs.ownship_state.alt_layer
        tgt = self.wp_queue[0]
        new_restricted = [
            c for c in obs.active_constraints
            if c.phase == ConstraintPhase.RESTRICTED
            and (c.id, c.phase.value) not in self.con_sig
        ]
        for c in new_restricted:
            if c.alt_layers and alt not in c.alt_layers:
                continue
            verts = expand_region(c.region, SAFETY_BUFFER)
            if verts and crosses_poly(pos, tgt, verts):
                return True
        return False

    def _future_threats(self, obs, tick):
        s    = obs.ownship_state

        if self.wp_queue:
            tgt  = self.wp_queue[0]
            dx   = tgt.x - s.position.x
            dy   = tgt.y - s.position.y
            d    = math.hypot(dx, dy)
            trvl = min(MAX_SPEED * tick, d)
            my_f = Position2D(x=s.position.x + dx / d * trvl,
                              y=s.position.y + dy / d * trvl) if d > 1e-6 else s.position
        else:
            my_f = s.position

        threats = []
        for t in obs.traffic_tracks:
            if not t.velocity:
                continue
            fpos = Position2D(x=t.position.x + t.velocity.x * tick,
                              y=t.position.y + t.velocity.y * tick)
            if dist(my_f, fpos) < ADVISORY_DIST and abs(t.alt_layer - s.alt_layer) <= 1:
                class _P: pass
                p           = _P()
                p.position  = fpos
                p.velocity  = t.velocity
                p.alt_layer = t.alt_layer
                threats.append(p)
        return threats

    def step(self, obs: Observation) -> Plan:
        s   = obs.ownship_state
        pos = s.position
        alt = s.alt_layer

        if not self._ready:
            self._ready = True

        sig = self._sig(obs)
        if not self.wp_queue or sig != self.con_sig or self._path_blocked(obs):
            self._replan(obs)
            self.con_sig = sig

        target     = self._target(pos)
        target_alt = safe_alt(
            obs.mission_goal.target_alt_layer
            if obs.mission_goal.target_alt_layer is not None else alt)

        if self.consec_restricted >= 3:
            target_alt = safe_alt(alt + 1)

        dx = target.x - pos.x
        dy = target.y - pos.y
        d  = math.hypot(dx, dy)
        pref_vel = Position2D(
            x=dx / d * MAX_SPEED if d > 1e-6 else 0.0,
            y=dy / d * MAX_SPEED if d > 1e-6 else 0.0,
        )
        my_vel = s.velocity if s.velocity else Position2D(x=0.0, y=0.0)

        threats       = self._future_threats(obs, tick=COMMIT_LEN)
        avoidance_vel = None
        alt_bump      = False

        if threats:
            adj = vo_adjust(pos, my_vel, pref_vel, threats, alt)
            if dist(adj, pref_vel) > 1.5:
                avoidance_vel = adj
            if any(dist(pos, t.position) < CONFLICT_DIST for t in threats) and alt < MAX_ALT:
                alt_bump = True

        if alt_bump or self.consec_restricted >= 3:
            target_alt = safe_alt(alt + 1)

        plan = build_plan(
            pos=pos,
            alt=alt,
            target=target,
            target_alt=target_alt,
            avoidance_vel=avoidance_vel,
            committed_step=self.committed,
            constraints=obs.active_constraints,
        )

        step1 = plan.steps[1]
        if (step1.target_position and
                in_restricted(step1.target_position,
                              step1.target_alt_layer or alt,
                              obs.active_constraints)):
            self.consec_restricted += 1
        else:
            self.consec_restricted = 0

        self.committed = plan.steps[1]
        return plan
