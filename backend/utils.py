# backend/utils.py
import math

def clip(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def wrap_pi(a):
    # Wrap to [-pi, pi)
    while a <= -math.pi: a += 2*math.pi
    while a >   math.pi: a -= 2*math.pi
    return a
