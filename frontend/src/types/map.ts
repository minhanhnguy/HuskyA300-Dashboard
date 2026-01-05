// frontend/src/types/map.ts

export interface MapOrigin {
    x: number;
    y: number;
    yaw: number;
}

export interface MapMeta {
    width: number;
    height: number;
    resolution: number;
    origin: MapOrigin;
    version: number;
    tile_size: number;
}

export interface MapFullResponse {
    version: number;
    data: number[];
}

export interface MapFullAtResponse {
    meta: MapMeta;
    data: number[];
}

export interface MapDeltaPatch {
    x: number;
    y: number;
    w: number;
    h: number;
    data: number[];
}

export interface MapDeltaReset {
    meta: MapMeta;
    data: number[];
}

export interface MapDeltaResponse {
    version?: number;
    patches?: MapDeltaPatch[];
    reset?: MapDeltaReset;
}
