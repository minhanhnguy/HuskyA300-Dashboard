// frontend/src/types.ts

export interface MapOrigin { x: number; y: number; yaw: number; }
export interface MapMeta {
  width: number; height: number; resolution: number;
  origin: MapOrigin; version: number; tile_size: number;
}

export interface MapFullResponse { version: number; data: number[]; }
export interface MapFullAtResponse { meta: MapMeta; data: number[]; }

export interface MapDeltaPatch { x: number; y: number; w: number; h: number; data: number[]; }
export interface MapDeltaReset { meta: MapMeta; data: number[]; }
export interface MapDeltaResponse {
  version?: number; patches?: MapDeltaPatch[];
  reset?: MapDeltaReset;
}

export interface PoseHistoryItem { t: number; x: number; y: number; yaw: number; }
export interface PoseHistoryResponse { pose_history: PoseHistoryItem[]; }
export interface PoseHistoryMeta { count: number; t0: number; t1: number; }

// WS: path stream
export type PathWSMessage =
  | { type: "snapshot"; path: [number, number][]; pose: {x:number;y:number;yaw:number}; v:number; w:number; }
  | { type: "append"; append: [number, number][]; pose: {x:number;y:number;yaw:number}; };

// WS: map stream (live)
export type MapWSMessage =
  | { type: "map_meta"; meta: MapMeta }
  | { type: "map_full"; version: number; data: number[] }
  | { type: "map_updates"; updates: { x:number;y:number;w:number;h:number;data:number[] }[] };

// Lidar scan (bag mode)
export interface ScanAtResponse {
  t: number;        // actual scan time used (bag time, seconds)
  frame: string;    // original frame_id
  count: number;    // number of (x,y) points
  points: number[]; // flattened [x0, y0, x1, y1, ...] in MAP frame (meters)
}

// cmd_vel stats (bag mode)
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
