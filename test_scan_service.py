
import math
import time
from typing import List, Optional, Tuple

class _StampedSE2:
    __slots__ = ("t","x","y","yaw")
    def __init__(self, t: float, x: float, y: float, yaw: float):
        self.t = t; self.x = x; self.y = y; self.yaw = yaw

def _latest_before(lst: List[_StampedSE2], t: float, start_idx: int = 0) -> tuple[Optional[_StampedSE2], int]:
    if not lst: return None, start_idx
    
    n = len(lst)
    # If list is empty or first element is already after t
    if n == 0 or lst[0].t > t:
        return None, 0
        
    # If last element is before t, return it
    if lst[-1].t <= t:
        return lst[-1], n - 1
        
    # Binary search
    lo, hi = 0, n - 1
    while lo < hi:
        mid = (lo + hi + 1) // 2
        if lst[mid].t <= t:
            lo = mid
        else:
            hi = mid - 1
            
    return lst[lo], lo

def test_performance():
    # Create a large list of StampedSE2
    N = 1000000
    print(f"Generating {N} items...")
    lst = [_StampedSE2(t=float(i)*0.01, x=0, y=0, yaw=0) for i in range(N)]
    
    print("Testing linear search (simulated)...")
    # Simulate old linear search
    start = time.time()
    for i in range(100):
        t = N * 0.01 * 0.9 # 90% through
        idx = 0
        while idx + 1 < N and lst[idx+1].t <= t:
            idx += 1
    end = time.time()
    print(f"Linear search (100 queries): {end - start:.4f}s")
    
    print("Testing binary search...")
    start = time.time()
    for i in range(10000):
        t = N * 0.01 * 0.9
        _latest_before(lst, t)
    end = time.time()
    print(f"Binary search (10000 queries): {end - start:.4f}s")

if __name__ == "__main__":
    test_performance()
