// frontend/src/state/timeline.ts
/**
 * Timeline state management.
 * Manages pose history and time scrubbing.
 */

import type { PoseHistoryItem, Pose } from '../types';

export interface TimelineState {
    poseHist: PoseHistoryItem[];
    t0: number | null;           // first timestamp
    lastPoseCount: number;

    // Scrub state
    scrubTime: number | null;    // absolute seconds
    scrubPose: Pose | null;

    // Cumulative distances for midpoint calculation
    poseHistDist: Float32Array | null;

    // Current live pose
    path: Array<[number, number]>;
    pose: Pose;
}

export function createInitialTimelineState(): TimelineState {
    return {
        poseHist: [],
        t0: null,
        lastPoseCount: 0,
        scrubTime: null,
        scrubPose: null,
        poseHistDist: null,
        path: [],
        pose: { x: 0, y: 0, yaw: 0 },
    };
}

// Global mutable state
export const timelineState = createInitialTimelineState();

/**
 * Get duration of pose history.
 */
export function getDuration(): number {
    const { poseHist, t0 } = timelineState;
    if (!poseHist.length || t0 == null) return 0;
    const tMax = poseHist[poseHist.length - 1].t;
    return Math.max(0, tMax - t0);
}

/**
 * Interpolate pose at time t.
 */
export function poseAt(t: number): Pose | null {
    const { poseHist } = timelineState;
    if (!poseHist.length) return null;

    if (t <= poseHist[0].t) {
        return { x: poseHist[0].x, y: poseHist[0].y, yaw: poseHist[0].yaw };
    }

    if (t >= poseHist[poseHist.length - 1].t) {
        const last = poseHist[poseHist.length - 1];
        return { x: last.x, y: last.y, yaw: last.yaw };
    }

    // Binary search
    let lo = 0, hi = poseHist.length - 1;
    while (lo < hi) {
        const mid = (lo + hi) >> 1;
        if (poseHist[mid].t < t) lo = mid + 1;
        else hi = mid;
    }

    const i = lo;
    const a = poseHist[i - 1];
    const b = poseHist[i];
    const u = Math.max(0, Math.min(1, (t - a.t) / Math.max(1e-6, b.t - a.t)));

    const x = a.x + (b.x - a.x) * u;
    const y = a.y + (b.y - a.y) * u;

    let dyaw = b.yaw - a.yaw;
    if (dyaw > Math.PI) dyaw -= 2 * Math.PI;
    if (dyaw < -Math.PI) dyaw += 2 * Math.PI;
    const yaw = a.yaw + dyaw * u;

    return { x, y, yaw };
}

/**
 * Get pose at relative time in bag.
 */
export function getBagPoseAtRel(rel: number): Pose {
    const { t0, pose } = timelineState;
    if (!timelineState.poseHist.length || t0 == null) return pose;
    return poseAt(t0 + rel) || pose;
}

/**
 * Update pose history with new data.
 */
export function setPoseHistory(poseHist: PoseHistoryItem[]): void {
    timelineState.poseHist = poseHist;
    if (poseHist.length) {
        timelineState.t0 = poseHist[0].t;
        timelineState.lastPoseCount = poseHist.length;

        // Pre-calculate cumulative distances
        timelineState.poseHistDist = new Float32Array(poseHist.length);
        timelineState.poseHistDist[0] = 0;
        for (let i = 1; i < poseHist.length; i++) {
            const dx = poseHist[i].x - poseHist[i - 1].x;
            const dy = poseHist[i].y - poseHist[i - 1].y;
            const d = Math.sqrt(dx * dx + dy * dy);
            timelineState.poseHistDist[i] = timelineState.poseHistDist[i - 1] + d;
        }
    } else {
        timelineState.t0 = null;
        timelineState.lastPoseCount = 0;
        timelineState.poseHistDist = null;
    }
}

/**
 * Set scrub time and calculate scrub pose.
 */
export function setScrubTime(relT: number | null): void {
    if (relT == null) {
        timelineState.scrubTime = null;
        timelineState.scrubPose = null;
    } else {
        const { t0 } = timelineState;
        if (t0 == null) return;
        timelineState.scrubTime = t0 + relT;
        timelineState.scrubPose = poseAt(timelineState.scrubTime!);
    }
}

/**
 * Clear scrub state (follow live).
 */
export function clearScrub(): void {
    timelineState.scrubTime = null;
    timelineState.scrubPose = null;
}
