// frontend/src/types/pose.ts

export interface PoseHistoryItem {
    t: number;
    x: number;
    y: number;
    yaw: number;
}

export interface PoseHistoryResponse {
    pose_history: PoseHistoryItem[];
}

export interface PoseHistoryMeta {
    count: number;
    t0: number;
    t1: number;
}

export interface Pose {
    x: number;
    y: number;
    yaw: number;
}

export interface MidpointResponse {
    t: number;
    x: number;
    y: number;
    yaw: number;
    pose_history_distance: number;
    nav_path_distance: number;
    total_distance: number;
    midpoint_distance: number;
}
