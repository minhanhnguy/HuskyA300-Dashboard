# backend/services/cmd_service.py
from __future__ import annotations

from typing import Optional

from ..models import CmdStatsResponse, CmdInstant, CmdPrefixStats


def get_cmd_stats_at_service(core, t: float) -> CmdStatsResponse:
    """
    Bag-mode cmd_vel stats at absolute bag time t (seconds).
    """
    with core.lock:
        idx = getattr(core, "_bag_cmd_index", None)

    # No cmd data at all
    if idx is None or not getattr(idx, "times", None):
        return CmdStatsResponse(
            instant=None,
            prefix=None,
            bag_t0=None,
            bag_t1=None,
        )

    times = idx.times
    if not times:
        return CmdStatsResponse(instant=None, prefix=None, bag_t0=None, bag_t1=None)

    bag_t0 = idx.bag_t0 if idx.bag_t0 is not None else times[0]
    bag_t1 = idx.bag_t1 if idx.bag_t1 is not None else times[-1]

    # Outside cmd_vel span: tell UI what span exists, but no instant at t
    # RELAXED: If t > times[-1], we still want the cumulative stats (prefix).
    # instant_index_at handles t >= times[-1] by returning the last index.
    # instant_index_at handles t < times[0] by returning None.
    
    # if t < times[0] or t > times[-1]:
    #     return CmdStatsResponse(
    #         instant=None,
    #         prefix=None,
    #         bag_t0=bag_t0,
    #         bag_t1=bag_t1,
    #     )

    # Debug logging
    print(f"[cmd_service] t={t}, times[0]={times[0]}, times[-1]={times[-1]}")

    i: Optional[int] = idx.instant_index_at(t)
    print(f"[cmd_service] instant_index_at({t}) -> {i}")
    
    if i is None:
        print(f"[cmd_service] i is None, returning empty")
        return CmdStatsResponse(
            instant=None,
            prefix=None,
            bag_t0=bag_t0,
            bag_t1=bag_t1,
        )

    instant = CmdInstant(
        t=times[i],
        vx=idx.vx[i],
        vy=idx.vy[i],
        vz=idx.vz[i],
        wx=idx.wx[i],
        wy=idx.wy[i],
        wz=idx.wz[i],
    )

    prefix = CmdPrefixStats(
        t0=bag_t0,
        t1=times[i],
        sample_count=i + 1,
        max_vx_forward=idx.max_vx_forward_prefix[i],
        max_vx_reverse=idx.max_vx_reverse_prefix[i],
        avg_vx=idx.avg_vx_prefix[i],
        max_vy_abs=idx.max_vy_abs_prefix[i],
        avg_vy_abs=idx.avg_vy_abs_prefix[i],
        max_wz_abs=idx.max_wz_abs_prefix[i],
        has_lateral=idx.has_lateral_prefix[i],
        has_ang_xy=idx.has_ang_xy_prefix[i],
    )

    return CmdStatsResponse(
        instant=instant,
        prefix=prefix,
        bag_t0=bag_t0,
        bag_t1=bag_t1,
    )
