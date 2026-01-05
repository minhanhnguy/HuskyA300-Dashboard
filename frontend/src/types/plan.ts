// frontend/src/types/plan.ts

export interface PlanResponse {
    t: number;
    poses: [number, number][]; // list of [x, y]
}

export interface GoalResponse {
    t: number;
    x: number;
    y: number;
    yaw: number;
}
