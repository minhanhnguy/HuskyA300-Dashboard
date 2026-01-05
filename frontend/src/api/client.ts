// frontend/src/api/client.ts
/**
 * API client for backend communication.
 * Centralizes all fetch calls to the backend.
 */

import type {
    MapMeta,
    MapFullResponse,
    MapFullAtResponse,
    MapDeltaResponse,
    PoseHistoryResponse,
    ScanAtResponse,
    CmdStatsResponse,
    PlanResponse,
    GoalResponse,
    MidpointResponse,
} from '../types';

const BASE_URL = '';  // Same origin

// ========== Map API ==========

export async function fetchMapMeta(): Promise<MapMeta | null> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/map_meta`);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

export async function fetchMapFull(): Promise<MapFullResponse | null> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/map_full`);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

export async function fetchMapFullAt(t: number): Promise<MapFullAtResponse | null> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/map_full_at?t=${encodeURIComponent(t.toFixed(3))}`);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

export async function fetchMapDelta(t0: number, t1: number): Promise<MapDeltaResponse | null> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/map_delta?t0=${t0.toFixed(3)}&t1=${t1.toFixed(3)}`);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

// ========== Costmap API ==========

export async function fetchGlobalCostmapAt(t: number): Promise<MapFullAtResponse | null> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/costmap_global_full_at?t=${encodeURIComponent(t.toFixed(3))}`);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

export async function fetchLocalCostmapAt(t: number): Promise<MapFullAtResponse | null> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/costmap_local_full_at?t=${encodeURIComponent(t.toFixed(3))}`);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

// ========== Pose API ==========

export async function fetchPoseHistory(): Promise<PoseHistoryResponse> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/pose_history`);
        return await r.json();
    } catch {
        return { pose_history: [] };
    }
}

// ========== Scan API ==========

export async function fetchScanAt(
    t: number,
    options: { decim?: number; maxPoints?: number; tol?: number } = {}
): Promise<ScanAtResponse | null> {
    const { decim = 1, maxPoints = 12000, tol = 0.05 } = options;
    try {
        const url = `${BASE_URL}/api/v1/scan_at?t=${t.toFixed(3)}&decim=${decim}&max_points=${maxPoints}&tol=${tol}`;
        const r = await fetch(url);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

// ========== Cmd_vel API ==========

export async function fetchCmdStatsAt(t: number): Promise<CmdStatsResponse | null> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/cmd_stats_at?t=${encodeURIComponent(t.toFixed(3))}`);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

// ========== Plan/Goal API ==========

export async function fetchPlanAt(t: number): Promise<PlanResponse | null> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/plan_at?t=${encodeURIComponent(t.toFixed(3))}`);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

export async function fetchGoalAt(t: number): Promise<GoalResponse | null> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/goal_at?t=${encodeURIComponent(t.toFixed(3))}`);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

export async function fetchMidpoint(atTime?: number): Promise<MidpointResponse | null> {
    try {
        const url = atTime !== undefined
            ? `${BASE_URL}/api/v1/bag/midpoint?t=${atTime}`
            : `${BASE_URL}/api/v1/bag/midpoint`;
        const r = await fetch(url);
        if (!r.ok) return null;
        return await r.json();
    } catch {
        return null;
    }
}

// ========== Bag API ==========

export interface BagFile {
    name: string;
    size: number;
    spoofed: boolean;
}

export async function fetchBagList(): Promise<BagFile[]> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/bag/list`);
        return await r.json();
    } catch {
        return [];
    }
}

export async function playBag(name: string, spoofed: boolean = false): Promise<{ ok: boolean; error?: string }> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/bag/play`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name, spoofed }),
        });
        return await r.json();
    } catch (e) {
        return { ok: false, error: String(e) };
    }
}

export async function spoofBag(name: string): Promise<{ ok: boolean; output_path?: string; error?: string }> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/bag/spoof`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name }),
        });
        return await r.json();
    } catch (e) {
        return { ok: false, error: String(e) };
    }
}

// ========== Robot API ==========

export async function fetchRobotDescription(): Promise<{ urdf: string | null }> {
    try {
        const r = await fetch(`${BASE_URL}/api/v1/robot_description`);
        return await r.json();
    } catch {
        return { urdf: null };
    }
}

// ========== Control API ==========

export async function resetPose(): Promise<void> {
    await fetch(`${BASE_URL}/api/v1/reset`, { method: 'POST' });
}

export async function reverseReplay(): Promise<void> {
    await fetch(`${BASE_URL}/api/v1/reverse_replay`, { method: 'POST' });
}
