// frontend/src/types/cmd.ts

export interface CmdInstant {
    t: number;
    vx: number;
    vy: number;
    vz: number;
    wx: number;
    wy: number;
    wz: number;
}

export interface CmdPrefixStats {
    t0: number;
    t1: number;
    sample_count: number;
    max_vx_forward: number;
    max_vx_reverse: number;
    avg_vx: number;
    max_vy_abs: number;
    avg_vy_abs: number;
    max_wz_abs: number;
    has_lateral: boolean;
    has_ang_xy: boolean;
}

export interface CmdStatsResponse {
    instant?: CmdInstant | null;
    prefix?: CmdPrefixStats | null;
    bag_t0?: number | null;
    bag_t1?: number | null;
}
