from aerohacks.core.models import Position2D, ConstraintPhase
from geometry import *
from consts import *
import math


class Obstacle:
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
        return crosses_poly(a, b, self.verts)


def make_obstacle(region, cost_mult, is_hard, alt_layers=None):
    verts = expand_region(region, SAFETY_BUFFER)
    return Obstacle(verts, cost_mult, is_hard, alt_layers) if verts else None


def phase_cost(phase):
    if phase == ConstraintPhase.RESTRICTED: return COST_RESTRICTED, True
    if phase == ConstraintPhase.CONTROLLED: return COST_CONTROLLED, False
    if phase == ConstraintPhase.ADVISORY:   return COST_ADVISORY,   False
    return 1.0, False
