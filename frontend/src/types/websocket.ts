// frontend/src/types/websocket.ts

import type { MapMeta } from './map';

// WS: path stream
export type PathWSMessage =
    | { type: "snapshot"; path: [number, number][]; pose: { x: number; y: number; yaw: number }; v: number; w: number; }
    | { type: "append"; append: [number, number][]; pose: { x: number; y: number; yaw: number }; };

// WS: map stream (live)
export type MapWSMessage =
    | { type: "map_meta"; meta: MapMeta }
    | { type: "map_full"; version: number; data: number[] }
    | { type: "map_updates"; updates: { x: number; y: number; w: number; h: number; data: number[] }[] };
