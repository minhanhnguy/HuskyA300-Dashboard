// frontend/src/types/scan.ts

export interface ScanAtResponse {
    t: number;        // actual scan time used (bag time, seconds)
    frame: string;    // original frame_id
    count: number;    // number of (x,y) points
    points: number[]; // flattened [x0, y0, x1, y1, ...] in MAP frame (meters)
}
