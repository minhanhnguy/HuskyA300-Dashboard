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
