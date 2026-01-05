# backend/utils/math.py
"""
Common mathematical utility functions.
"""
import math


def clip(v: float, lo: float, hi: float) -> float:
    """Clip value v to range [lo, hi]."""
    return lo if v < lo else hi if v > hi else v


def wrap_pi(a: float) -> float:
    """Wrap angle to [-pi, pi)."""
    while a <= -math.pi:
        a += 2 * math.pi
    while a > math.pi:
        a -= 2 * math.pi
    return a
