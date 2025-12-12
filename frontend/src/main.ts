// frontend/src/main.ts

import * as d3 from "d3";
import {
  MapMeta,
  MapFullResponse,
  MapFullAtResponse,
  MapDeltaResponse,
  PoseHistoryItem,
  PoseHistoryResponse,
  PathWSMessage,
  MapWSMessage,
  ScanAtResponse,
  CmdStatsResponse,
  PlanResponse,
  GoalResponse,
} from "./types";

/**
 * Husky Dashboard — main.ts
 * - Live mode: timeline grows; slider can follow live or scrub.
 * - Bag mode: timeline frozen to bag duration; play/pause + scrub.
 * - Map “time-travel”: in bag mode fetch exact map at slider time.
 * - During play, apply map *patches* between frames for progressive refining.
 * - cmd_vel overlay: bag-mode only, window on top of canvas, driven by /cmd_stats_at.
 */

// Augment window for a cached map meta (set by live WS on connect)
declare global {
  interface Window {
    _latestMapMeta?: MapMeta;
  }
}

// ==============================
// DOM references (strictly typed)
// ==============================
const canvas = document.getElementById("view") as HTMLCanvasElement | null;
if (!canvas) throw new Error("Canvas #view not found");
const ctx = canvas.getContext("2d");
if (!ctx) throw new Error("2D context unavailable");
ctx.imageSmoothingEnabled = false;

const statusDiv = document.getElementById("status") as HTMLDivElement | null;
const slider = document.getElementById("timeSlider") as HTMLInputElement | null;
const timeLabel = document.getElementById("timeLabel") as HTMLSpanElement | null;
const liveBtn = document.getElementById("liveBtn") as HTMLButtonElement | null; // hidden in bag mode
const liveModeBtn = document.getElementById("liveModeBtn") as HTMLButtonElement | null; // "Go back to Live"
const playBtn = document.getElementById("playBtn") as HTMLButtonElement | null; // Play/Pause for bag mode

// Bag selection panel
const bagPanelBtn = document.getElementById("bagPanelBtn") as HTMLButtonElement | null;
const bagPanel = document.getElementById("bagPanel") as HTMLDivElement | null;
const bagPanelClose = document.getElementById("bagPanelClose") as HTMLButtonElement | null;
const bagPanelStatus = document.getElementById("bagPanelStatus") as HTMLParagraphElement | null;
const bagList = document.getElementById("bagList") as HTMLUListElement | null;

// Analysis Panel elements
// const cmdOverlayToggle = document.getElementById("cmdOverlayToggle") as HTMLButtonElement | null; // Analysis Panel elements
const analysisPanel = document.getElementById("analysisPanel") as HTMLDivElement;
const cmdOverlayCanvas = document.getElementById("cmdOverlayCanvas") as HTMLCanvasElement;
const cmdInstantNumbers = document.getElementById("cmdInstantNumbers") as HTMLDivElement;
const cmdPrefixNumbers = document.getElementById("cmdPrefixNumbers") as HTMLDivElement;
const cmdOverlayCtx = cmdOverlayCanvas ? cmdOverlayCanvas.getContext("2d") : null;

// View Menu Toggles
const toggleGlobalCostmap = document.getElementById("toggleGlobalCostmap") as HTMLInputElement | null;
const toggleLocalCostmap = document.getElementById("toggleLocalCostmap") as HTMLInputElement | null;
const toggleLidar = document.getElementById("toggleLidar") as HTMLInputElement | null;
const togglePlan = document.getElementById("togglePlan") as HTMLInputElement | null;
const toggleGoal = document.getElementById("toggleGoal") as HTMLInputElement | null;
const toggleCmdOverlay = document.getElementById("toggleCmdOverlay") as HTMLInputElement | null;

// Guard essentials
if (!slider || !timeLabel || !liveModeBtn) {
  throw new Error("Required toolbar elements not found");
}

// ==============================
// Canvas + zoom
// ==============================
let W = 0,
  H = 0;
function resize(): void {
  const dpr = window.devicePixelRatio || 1;
  W = canvas.clientWidth;
  H = canvas.clientHeight;
  canvas.width = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
}
window.addEventListener("resize", resize);
resize();

let zoomTransform: d3.ZoomTransform = d3.zoomIdentity.translate(W / 2, H / 2).scale(40);
const zoom = d3
  .zoom<HTMLCanvasElement, unknown>()
  .scaleExtent([2, 400])
  .on("zoom", (ev: d3.D3ZoomEvent<HTMLCanvasElement, unknown>) => {
    zoomTransform = ev.transform;
  });

// TypeScript can be picky about .call signatures; light casts keep it simple.
(d3.select(canvas) as any).call(zoom as any).call((zoom as any).transform, zoomTransform);

// ==============================
// Global state
// ==============================
let isBagMode = false; // true => freeze live growth; use map_full_at
let liveMode = true; // auto-follow end in live mode (ignored in bag mode)

let isPlaying = false; // bag playback
let playRate = 1.0; // 1×
let lastTickTs: number | null = null;

// Progressive refine timer & state
let mapPlayTimer: number | null = null; // drives map_delta polling
let lastAbsTForDelta: number | null = null;
let deltaBusy = false;

let path: Array<[number, number]> = [];
let pose: { x: number; y: number; yaw: number } = { x: 0, y: 0, yaw: 0 };

let poseHist: PoseHistoryItem[] = [];
let t0: number | null = null; // first timestamp
let lastPoseCount = 0;

let scrubTime: number | null = null; // absolute seconds
let scrubPose: { x: number; y: number; yaw: number } | null = null;

// Map tile layer
let mapLayer: MapTiles | null = null;
let currentMapVersion = -999;
let isMapLoading = false; // draw shimmer overlay while fetching

// Lidar scan layer (bag mode)
let scanPoints: Float32Array | null = null;
let lastScanReqAbsT: number | null = null;
let scanReqInFlight = false;

// cmd_vel overlay state (bag mode only)
// Now controls the visibility of the bottom panel tab
let cmdOverlayEnabled = false;
let cmdStats: CmdStatsResponse | null = null;
let lastCmdStatsReqAbsT: number | null = null;
let cmdStatsReqInFlight = false;

// Costmap layers
let globalCostmapLayer: MapTiles | null = null;
let currentGlobalCostmapVersion = -999;
let localCostmapLayer: MapTiles | null = null;
let currentLocalCostmapVersion = -999;

// Visibility flags
let showGlobalCostmap = true;
let showLocalCostmap = true;
let showLidar = true;
let showPlan = true;
let showGoal = true;

// 50% path marker
let halfwayPoint: { x: number; y: number; reached: boolean } | null = null;
let halfwayT: { t: number; reached: boolean } | null = null;
let poseHistDist: Float32Array | null = null;
let fixedHalfway: { t: number; x: number; y: number } | null = null;

// Plan/Goal state
let planPath: [number, number][] | null = null;
let goalPose: { x: number; y: number; yaw: number } | null = null;
let planReqInFlight = false;
let goalReqInFlight = false;
let lastPlanReqAbsT: number | null = null;
let lastGoalReqAbsT: number | null = null;

// Robot Description (URDF)
// Default fallback: Husky A300 (0.99m x 0.698m)
interface RobotShape {
  type: "box" | "cylinder" | "a300";
  length: number; // or chassis_x
  width: number;  // or chassis_y
  wheel_base?: number;
  wheel_track?: number;
  wheel_radius?: number;
  wheel_width?: number;
}

let robotDims: RobotShape | null = {
  type: "a300",
  length: 0.860,
  width: 0.378,
  wheel_base: 0.512,
  wheel_track: 0.562,
  wheel_radius: 0.1651,
  wheel_width: 0.1143,
};

// ... (existing code) ...

async function fetchRobotDescription(): Promise<void> {
  console.log("Fetching robot description...");
  try {
    const r = await fetch("/api/v1/robot_description");
    if (!r.ok) {
      console.error("Failed to fetch robot description:", r.status, r.statusText);
      return;
    }
    const j = (await r.json()) as { urdf: string };
    if (!j.urdf) {
      console.warn("Robot description response has no URDF data");
      return;
    }
    console.log("Received URDF length:", j.urdf.length);

    const parser = new DOMParser();
    const xml = parser.parseFromString(j.urdf, "text/xml");

    // 1. Try to parse Xacro properties for A300
    // Look for <xacro:property name="..." value="..." />
    const props = xml.getElementsByTagName("xacro:property");
    const valMap: Record<string, number> = {};
    for (let i = 0; i < props.length; i++) {
      const name = props[i].getAttribute("name");
      const valStr = props[i].getAttribute("value");
      if (name && valStr) {
        const v = parseFloat(valStr);
        if (!isNaN(v)) valMap[name] = v;
      }
    }

    if (valMap["chassis_x_size"] && valMap["chassis_y_size"]) {
      console.log("Detected A300 properties:", valMap);
      robotDims = {
        type: "a300",
        length: valMap["chassis_x_size"],
        width: valMap["chassis_y_size"],
        wheel_base: valMap["wheel_base"] || 0.512,
        wheel_track: valMap["wheel_track"] || 0.562,
        wheel_radius: valMap["a300_outdoor_wheel_radius"] || 0.1651,
        wheel_width: valMap["a300_outdoor_wheel_width"] || 0.1143,
      };
      return;
    }

    // 1b. Heuristic: Check for A300 mesh paths if properties are missing
    if (j.urdf.includes("meshes/a300")) {
      console.log("Detected A300 via mesh paths (using default dims)");
      robotDims = {
        type: "a300",
        length: 0.860,
        width: 0.378,
        wheel_base: 0.512,
        wheel_track: 0.562,
        wheel_radius: 0.1651,
        wheel_width: 0.1143,
      };
      return;
    }

    // 2. Fallback to standard URDF parsing (existing logic)
    const robot = xml.querySelector("robot");
    if (!robot) {
      console.error("URDF has no <robot> tag");
      return;
    }

    // Try to find base_link or footprint
    let link = xml.querySelector("link[name='base_link']") || xml.querySelector("link[name='base_footprint']");
    if (!link) {
      // Fallback: first link with geometry
      link = xml.querySelector("link visual geometry");
      if (link) link = link.closest("link");
    }

    if (!link) {
      console.error("No suitable link found in URDF");
      return;
    }

    const geometry = link.querySelector("visual geometry");
    if (!geometry) {
      console.error("Link has no visual geometry");
      return;
    }

    const box = geometry.querySelector("box");
    const cylinder = geometry.querySelector("cylinder");

    if (box) {
      const sizeStr = box.getAttribute("size"); // "x y z"
      if (sizeStr) {
        const parts = sizeStr.trim().split(/\s+/).map(Number);
        if (parts.length >= 2) {
          robotDims = { length: parts[0], width: parts[1], type: "box" };
          console.log("Parsed robot dims (box):", robotDims);
        }
      }
    } else if (cylinder) {
      const radius = Number(cylinder.getAttribute("radius"));
      if (!isNaN(radius)) {
        robotDims = { length: radius * 2, width: radius * 2, type: "cylinder" };
        console.log("Parsed robot dims (cylinder):", robotDims);
      }
    }
  } catch (e) {
    console.error("Failed to parse robot description:", e);
  }
}
const sleep = (ms: number) => new Promise<void>((r) => setTimeout(r, ms));

function setStatus(txt: string): void {
  if (statusDiv) statusDiv.textContent = txt;
}

function getDuration(): number {
  if (!poseHist.length || t0 == null) return 0;
  const tMax = poseHist[poseHist.length - 1].t;
  return Math.max(0, tMax - t0);
}

function updateTimeUI(rel: number): void {
  const v = Number.isFinite(rel) ? rel : 0;
  slider!.value = v.toFixed(2);
  timeLabel!.textContent = `t = ${v.toFixed(2)} s`;
}

async function loadPoseHistoryOnce(): Promise<void> {
  let j: PoseHistoryResponse = { pose_history: [] };
  try {
    const r = await fetch("/api/v1/pose_history");
    j = (await r.json()) as PoseHistoryResponse;
  } catch {
    // leave as empty
  }
  poseHist = j.pose_history || [];
  if (poseHist.length) {
    t0 = poseHist[0].t;
    lastPoseCount = poseHist.length;

    // Pre-calculate cumulative distances
    poseHistDist = new Float32Array(poseHist.length);
    poseHistDist[0] = 0;
    for (let i = 1; i < poseHist.length; i++) {
      const dx = poseHist[i].x - poseHist[i - 1].x;
      const dy = poseHist[i].y - poseHist[i - 1].y;
      const d = Math.sqrt(dx * dx + dy * dy);
      poseHistDist[i] = poseHistDist[i - 1] + d;
    }
  } else {
    t0 = null;
    lastPoseCount = 0;
    poseHistDist = null;
  }
}

function setTimelineFromPoseHist(): void {
  const dur = getDuration();
  slider!.min = "0";
  slider!.max = dur.toFixed(2);
  if (dur === 0) updateTimeUI(0);
}

function setTimelineToEnd(): void {
  const dur = getDuration();
  updateTimeUI(dur);
  scrubTime = null;
  scrubPose = null;
}

function poseAt(t: number): { x: number; y: number; yaw: number } | null {
  if (!poseHist.length) return null;
  if (t <= poseHist[0].t) return { x: poseHist[0].x, y: poseHist[0].y, yaw: poseHist[0].yaw };
  if (t >= poseHist[poseHist.length - 1].t)
    return {
      x: poseHist[poseHist.length - 1].x,
      y: poseHist[poseHist.length - 1].y,
      yaw: poseHist[poseHist.length - 1].yaw,
    };

  let lo = 0,
    hi = poseHist.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (poseHist[mid].t < t) lo = mid + 1;
    else hi = mid;
  }
  const i = lo,
    a = poseHist[i - 1],
    b = poseHist[i];
  const u = Math.max(0, Math.min(1, (t - a.t) / Math.max(1e-6, b.t - a.t)));
  const x = a.x + (b.x - a.x) * u;
  const y = a.y + (b.y - a.y) * u;
  let dyaw = b.yaw - a.yaw;
  if (dyaw > Math.PI) dyaw -= 2 * Math.PI;
  if (dyaw < -Math.PI) dyaw += 2 * Math.PI;
  const yaw = a.yaw + dyaw * u;
  return { x, y, yaw };
}

function getBagPoseAtRel(rel: number): { x: number; y: number; yaw: number } {
  if (!poseHist.length || t0 == null) return pose;
  return poseAt(t0 + rel) || pose;
}

// Wait until pose history “stabilizes” (no growth for ~settleMs)
async function waitForStablePoseHistory({
  timeoutMs = 5000,
  settleMs = 600,
  pollMs = 150,
}: {
  timeoutMs?: number;
  settleMs?: number;
  pollMs?: number;
} = {}): Promise<void> {
  const tStart = performance.now();
  let lastCount = -1;
  let lastChange = performance.now();
  while (performance.now() - tStart < timeoutMs) {
    await loadPoseHistoryOnce();
    setTimelineFromPoseHist();
    const count = poseHist.length;
    if (count !== lastCount) {
      lastCount = count;
      lastChange = performance.now();
    } else if (count > 0 && performance.now() - lastChange >= settleMs) {
      break;
    }
    await sleep(pollMs);
  }
}

// ==============================
// cmd_vel overlay helpers
// ==============================
function updateCmdOverlayVisibility(): void {
  if (!analysisPanel) return;
  // Show if enabled AND in bag mode
  if (cmdOverlayEnabled && isBagMode) {
    analysisPanel.classList.remove("hidden");
    // Ensure the tab is active (if we had multiple tabs, we'd select it here)
  } else {
    analysisPanel.classList.add("hidden");
  }
}

function describeInstant(stats: CmdStatsResponse | null): string {
  if (!stats || !stats.instant) {
    if (!isBagMode) {
      return "Overlay available in bag mode.\nChoose a bag and scrub the timeline.";
    }
    return "No cmd_vel yet at this time.";
  }
  const i = stats.instant;
  const vMag = Math.sqrt(i.vx * i.vx + i.vy * i.vy + i.vz * i.vz);
  const yawDir =
    i.wz > 1e-4 ? "CCW" : i.wz < -1e-4 ? "CW" : "no yaw";

  return [
    `t ≈ ${i.t.toFixed(3)} s`,
    `vx = ${i.vx.toFixed(3)} m/s`,
    `vy = ${i.vy.toFixed(3)} m/s`,
    `wz = ${i.wz.toFixed(3)} rad/s (${yawDir})`,
    `‖v‖ = ${vMag.toFixed(3)} m/s`,
  ].join("\n");
}

function describePrefix(stats: CmdStatsResponse | null): string {
  if (!stats || !stats.prefix) {
    return "No cmd_vel samples in bag.";
  }
  const p = stats.prefix;
  const span = (p.t1 - p.t0);
  return [
    `samples: ${p.sample_count}`,
    `span: ${span > 0 ? span.toFixed(2) : "0.00"} s`,
    `max forward vx: ${p.max_vx_forward.toFixed(3)} m/s`,
    `max reverse vx: ${p.max_vx_reverse.toFixed(3)} m/s`,
    `max |wz|: ${p.max_wz_abs.toFixed(3)} rad/s`,
    `lateral cmds: ${p.has_lateral ? "YES" : "no"}`,
    `weird axis ωx/ωy: ${p.has_ang_xy ? "YES" : "no"}`,
  ].join("\n");
}

function updateCmdOverlayNumbers(): void {
  if (!cmdInstantNumbers || !cmdPrefixNumbers) return;

  cmdInstantNumbers.textContent = describeInstant(cmdStats);
  cmdPrefixNumbers.textContent = describePrefix(cmdStats);
}

async function requestCmdStatsAtAbs(absT: number): Promise<void> {
  if (!cmdOverlayEnabled) return;
  if (!isBagMode || t0 == null) return;
  if (cmdStatsReqInFlight) return;
  // if (lastCmdStatsReqAbsT != null && Math.abs(absT - lastCmdStatsReqAbsT) < 0.03) return;

  cmdStatsReqInFlight = true;
  lastCmdStatsReqAbsT = absT;

  try {
    const url = `/api/v1/cmd_stats_at?t=${encodeURIComponent(absT.toFixed(3))}`;
    const r = await fetch(url);
    if (!r.ok) {
      cmdStats = null;
      updateCmdOverlayNumbers();
      return;
    }
    const j = (await r.json()) as CmdStatsResponse;
    cmdStats = j;
    updateCmdOverlayNumbers();
  } catch {
    cmdStats = null;
    updateCmdOverlayNumbers();
  } finally {
    cmdStatsReqInFlight = false;
  }
}

async function requestPublishAtAbs(absT: number): Promise<void> {
  if (!isBagMode || t0 == null) return;
  try {
    await fetch(`/api/v1/bag/publish_at?t=${encodeURIComponent(absT.toFixed(3))}`, { method: "POST" });
  } catch {
    // ignore
  }
}

function resizeCmdOverlayCanvas(): void {
  if (!cmdOverlayCanvas || !cmdOverlayCtx) return;
  const dpr = window.devicePixelRatio || 1;
  const cssW = cmdOverlayCanvas.clientWidth || 220;
  const cssH = cmdOverlayCanvas.clientHeight || 140;
  const w = Math.round(cssW * dpr);
  const h = Math.round(cssH * dpr);
  if (cmdOverlayCanvas.width !== w || cmdOverlayCanvas.height !== h) {
    cmdOverlayCanvas.width = w;
    cmdOverlayCanvas.height = h;
  }
}

function drawCmdOverlayMini(): void {
  if (!analysisPanel || analysisPanel.classList.contains("hidden")) return;
  if (!cmdOverlayCanvas || !cmdOverlayCtx) return;

  resizeCmdOverlayCanvas();
  const dpr = window.devicePixelRatio || 1;
  const w = cmdOverlayCanvas.width;
  const h = cmdOverlayCanvas.height;

  cmdOverlayCtx.save();
  cmdOverlayCtx.setTransform(1, 0, 0, 1, 0, 0);
  cmdOverlayCtx.clearRect(0, 0, w, h);

  // Work in CSS pixels internally
  const cw = w / dpr;
  const ch = h / dpr;
  cmdOverlayCtx.scale(dpr, dpr);

  // Background
  cmdOverlayCtx.fillStyle = "#f9fafb";
  cmdOverlayCtx.fillRect(0, 0, cw, ch);

  const cx = cw * 0.5;
  const cy = ch * 0.6;

  // Robot body (Detailed A300 or fallback)
  // We'll use a fixed scale for the overlay robot to fit nicely
  const robotScale = 0.25 * (cw / 1.0); // Adjust scale to fit canvas width

  cmdOverlayCtx.save();
  cmdOverlayCtx.translate(cx, cy);
  cmdOverlayCtx.scale(robotScale, robotScale);
  // Rotate -90 deg so "up" is forward (screen Y is down, so -90 makes +x point up)
  cmdOverlayCtx.rotate(-Math.PI / 2);

  // Use the same drawing logic as drawRobot but on cmdOverlayCtx
  // Dimensions (default A300 if not set)
  const dims = robotDims || {
    type: "a300",
    length: 0.860,
    width: 0.378,
    wheel_base: 0.512,
    wheel_track: 0.562,
    wheel_radius: 0.1651,
    wheel_width: 0.1143,
  };

  if (dims.type === "a300") {
    const chassisL = dims.length;
    const chassisW = dims.width;
    const wheelBase = dims.wheel_base || 0.512;
    const wheelTrack = dims.wheel_track || 0.562;
    const wheelRad = dims.wheel_radius || 0.1651;
    const wheelW = dims.wheel_width || 0.1143;
    const wheelDiam = wheelRad * 2;

    const colChassis = "#f59e0b";
    const colDark = "#333333";
    const colBumper = "#1f2937";

    // Suspension
    const beamW = 0.05;
    cmdOverlayCtx.fillStyle = colDark;
    cmdOverlayCtx.fillRect(-wheelBase / 2, wheelTrack / 2 - beamW / 2, wheelBase, beamW);
    cmdOverlayCtx.fillRect(-wheelBase / 2, -wheelTrack / 2 - beamW / 2, wheelBase, beamW);

    // Wheels
    cmdOverlayCtx.fillStyle = colDark;
    cmdOverlayCtx.fillRect(wheelBase / 2 - wheelDiam / 2, wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);
    cmdOverlayCtx.fillRect(wheelBase / 2 - wheelDiam / 2, -wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);
    cmdOverlayCtx.fillRect(-wheelBase / 2 - wheelDiam / 2, wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);
    cmdOverlayCtx.fillRect(-wheelBase / 2 - wheelDiam / 2, -wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);

    // Chassis
    cmdOverlayCtx.fillStyle = colChassis;
    cmdOverlayCtx.fillRect(-chassisL / 2, -chassisW / 2, chassisL, chassisW);

    // Bumpers
    const bumperDepth = 0.05;
    const bumperWidth = chassisW + 0.02;
    cmdOverlayCtx.fillStyle = colBumper;
    cmdOverlayCtx.fillRect(chassisL / 2, -bumperWidth / 2, bumperDepth, bumperWidth);
    cmdOverlayCtx.fillRect(-chassisL / 2 - bumperDepth, -bumperWidth / 2, bumperDepth, bumperWidth);

    // Indicator
    cmdOverlayCtx.fillStyle = "rgba(0,0,0,0.2)";
    cmdOverlayCtx.fillRect(chassisL * 0.1, -chassisW * 0.3, chassisL * 0.3, chassisW * 0.6);
    cmdOverlayCtx.fillStyle = "#ffffff";
    cmdOverlayCtx.beginPath();
    cmdOverlayCtx.moveTo(chassisL * 0.3, 0);
    cmdOverlayCtx.lineTo(chassisL * 0.45, 0);
    cmdOverlayCtx.strokeStyle = "#ffffff";
    cmdOverlayCtx.lineWidth = 0.02;
    cmdOverlayCtx.stroke();
    cmdOverlayCtx.beginPath();
    cmdOverlayCtx.moveTo(chassisL * 0.45, 0);
    cmdOverlayCtx.lineTo(chassisL * 0.4, chassisW * 0.05);
    cmdOverlayCtx.lineTo(chassisL * 0.4, -chassisW * 0.05);
    cmdOverlayCtx.fill();

  } else {
    // Box/Cylinder fallback
    const wPx = dims.width;
    const lPx = dims.length;
    cmdOverlayCtx.fillStyle = "#f59e0b";
    cmdOverlayCtx.beginPath();
    if (dims.type === "cylinder") {
      cmdOverlayCtx.arc(0, 0, wPx / 2, 0, Math.PI * 2);
    } else {
      cmdOverlayCtx.rect(-lPx / 2, -wPx / 2, lPx, wPx);
    }
    cmdOverlayCtx.fill();
    // Indicator
    cmdOverlayCtx.fillStyle = "#ffffff";
    cmdOverlayCtx.beginPath();
    cmdOverlayCtx.moveTo(lPx / 4, 0);
    cmdOverlayCtx.lineTo(lPx / 2, 0);
    cmdOverlayCtx.lineWidth = 0.02;
    cmdOverlayCtx.stroke();
  }
  cmdOverlayCtx.restore();

  if (!cmdStats || !cmdStats.instant) {
    cmdOverlayCtx.fillStyle = "#6b7280";
    cmdOverlayCtx.font = "11px system-ui, sans-serif";
    cmdOverlayCtx.textAlign = "center";
    cmdOverlayCtx.textBaseline = "middle";
    cmdOverlayCtx.fillText("No cmd_vel at this time", cx, ch * 0.9);
    return;
  }

  const i = cmdStats.instant;
  const p = cmdStats.prefix;

  // Draw linear velocity arrow in robot frame (rotated -90 for screen up)
  const vx = i.vx;
  const vy = i.vy;
  const speed = Math.sqrt(vx * vx + vy * vy);

  // Use prefix max for scaling if available
  const maxSpeedRef = p && p.sample_count > 0
    ? Math.max(Math.abs(p.max_vx_forward), Math.abs(p.max_vx_reverse), 0.5)
    : Math.max(Math.abs(vx), 0.5);

  const maxArrowLen = cw * 0.4;
  const arrowLen = Math.min(maxArrowLen, (speed / maxSpeedRef) * maxArrowLen);

  if (speed > 1e-3) {
    cmdOverlayCtx.save();
    cmdOverlayCtx.translate(cx, cy);
    cmdOverlayCtx.rotate(-Math.PI / 2); // Robot frame

    const angle = Math.atan2(vy, vx);
    const ax = Math.cos(angle) * arrowLen;
    const ay = Math.sin(angle) * arrowLen;

    cmdOverlayCtx.strokeStyle = "#f97316";
    cmdOverlayCtx.lineWidth = 3;
    cmdOverlayCtx.beginPath();
    cmdOverlayCtx.moveTo(0, 0);
    cmdOverlayCtx.lineTo(ax, ay);
    cmdOverlayCtx.stroke();

    // Arrowhead
    const headSize = 8;
    cmdOverlayCtx.translate(ax, ay);
    cmdOverlayCtx.rotate(angle);
    cmdOverlayCtx.fillStyle = "#f97316";
    cmdOverlayCtx.beginPath();
    cmdOverlayCtx.moveTo(0, 0);
    cmdOverlayCtx.lineTo(-headSize, headSize * 0.5);
    cmdOverlayCtx.lineTo(-headSize, -headSize * 0.5);
    cmdOverlayCtx.fill();

    cmdOverlayCtx.restore();
  }

  // Stats Text
  if (p) {
    cmdOverlayCtx.font = "10px monospace";
    cmdOverlayCtx.fillStyle = "#111827";
    cmdOverlayCtx.textAlign = "left";
    cmdOverlayCtx.textBaseline = "top";

    const lineH = 12;
    let ty = 5;
    const tx = 5;

    function drawStat(label: string, val: number, unit = "m/s") {
      cmdOverlayCtx.fillText(`${label}: ${val.toFixed(2)} ${unit}`, tx, ty);
      ty += lineH;
    }

    drawStat("Max Fwd", p.max_vx_forward);
    drawStat("Max Rev", p.max_vx_reverse);
    drawStat("Avg Vel", p.avg_vx);

    if (p.has_lateral) {
      ty += 2;
      drawStat("Max Lat", p.max_vy_abs);
      drawStat("Avg Lat", p.avg_vy_abs);
    }
  }
  cmdOverlayCtx.restore();
}

// ==============================
// Map tiles
// ==============================
type TileCell = {
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  dirty: boolean;
  tw: number;
  th: number;
  flashUntil: number; // ms timestamp (performance.now)
};

type PaletteFn = (v: number) => [number, number, number, number];

const mapPalette: PaletteFn = (v) => {
  if (v === -1) return [160, 160, 160, 255];
  if (v >= 65) return [0, 0, 0, 255];
  return [255, 255, 255, 255];
};

const globalCostmapPalette: PaletteFn = (v) => {
  // Global costmap: 25% opacity (alpha ~ 64)
  const alpha = 64;

  if (v === 0) return [0, 0, 0, 0]; // Transparent
  if (v === -1) return [112, 137, 134, alpha]; // Grey-Green
  if (v === 99) return [0, 255, 255, alpha];   // Cyan (Inscribed)
  if (v === 100) return [255, 0, 255, alpha];  // Magenta (Lethal)

  // 1-98: Blue -> Red
  // v=1 => Blue (0,0,255), v=98 => Red (255,0,0)
  if (v >= 1 && v <= 98) {
    const t = (v - 1) / 97.0;
    const r = Math.round(255 * t);
    const b = Math.round(255 * (1 - t));
    return [r, 0, b, alpha];
  }

  // Fallback for unexpected values
  return [0, 0, 0, 0];
};

const localCostmapPalette: PaletteFn = (v) => {
  // Local costmap: Full opacity (or as defined)
  const alpha = 255;

  if (v === 0) return [0, 0, 0, 0]; // Transparent
  if (v === -1) return [112, 137, 134, alpha]; // Grey-Green
  if (v === 99) return [0, 255, 255, alpha];   // Cyan (Inscribed)
  if (v === 100) return [255, 0, 255, alpha];  // Magenta (Lethal)

  // 1-98: Blue -> Red
  if (v >= 1 && v <= 98) {
    const t = (v - 1) / 97.0;
    const r = Math.round(255 * t);
    const b = Math.round(255 * (1 - t));
    return [r, 0, b, alpha];
  }

  return [0, 0, 0, 0];
};

class MapTiles {
  w: number;
  h: number;
  res: number;
  origin: MapMeta["origin"];
  tileSize: number;
  cols: number;
  rows: number;
  data: Int8Array;
  tiles: TileCell[][];
  palette: PaletteFn;

  constructor(meta: MapMeta, palette: PaletteFn = mapPalette) {
    this.w = meta.width;
    this.h = meta.height;
    this.res = meta.resolution;
    this.origin = meta.origin;
    this.tileSize = meta.tile_size || 256;
    this.palette = palette;

    this.cols = Math.ceil(this.w / this.tileSize);
    this.rows = Math.ceil(this.h / this.tileSize);

    this.data = new Int8Array(this.w * this.h);
    this.tiles = [];
    for (let r = 0; r < this.rows; r++) {
      this.tiles[r] = [];
      for (let c = 0; c < this.cols; c++) {
        const tw = Math.min(this.tileSize, this.w - c * this.tileSize);
        const th = Math.min(this.tileSize, this.h - r * this.tileSize);
        const cnv = document.createElement("canvas");
        cnv.width = tw;
        cnv.height = th;
        const tctx = cnv.getContext("2d");
        if (!tctx) throw new Error("Failed to get tile 2D context");
        tctx.imageSmoothingEnabled = false;
        this.tiles[r][c] = { canvas: cnv, ctx: tctx, dirty: true, tw, th, flashUntil: 0 };
      }
    }
  }

  loadFull(int8Flat: Int8Array): void {
    if (int8Flat.length !== this.w * this.h) return;
    this.data.set(int8Flat);
    for (let r = 0; r < this.rows; r++) for (let c = 0; c < this.cols; c++) this.tiles[r][c].dirty = true;
  }

  private _rebuildTile(r: number, c: number): void {
    const t = this.tiles[r][c];
    const tw = t.tw,
      th = t.th;
    const img = t.ctx.createImageData(tw, th);
    const x0 = c * this.tileSize;
    const y0 = r * this.tileSize;
    for (let yy = 0; yy < th; yy++) {
      const srcRow = (y0 + yy) * this.w + x0;
      const dstRow = (th - 1 - yy) * tw;
      for (let xx = 0; xx < tw; xx++) {
        const v = this.data[srcRow + xx];
        const [R, G, B, A] = this.palette(v);
        const i = (dstRow + xx) * 4;
        img.data[i + 0] = R;
        img.data[i + 1] = G;
        img.data[i + 2] = B;
        img.data[i + 3] = A;
      }
    }
    t.ctx.putImageData(img, 0, 0);
    t.dirty = false;
  }

  draw(
    ctx2d: CanvasRenderingContext2D,
    world2screenFn: (x: number, y: number) => [number, number],
    viewport: { xMin: number; xMax: number; yMin: number; yMax: number }
  ): void {
    if (!this.data) return;

    // 1. Calculate visible tile range (AABB in map frame)
    // Transform viewport corners to map frame to find min/max col/row
    const corners = [
      { x: viewport.xMin, y: viewport.yMin },
      { x: viewport.xMax, y: viewport.yMin },
      { x: viewport.xMax, y: viewport.yMax },
      { x: viewport.xMin, y: viewport.yMax },
    ];

    const cosYaw = Math.cos(this.origin.yaw);
    const sinYaw = Math.sin(this.origin.yaw);

    let minLx = Infinity, maxLx = -Infinity;
    let minLy = Infinity, maxLy = -Infinity;

    for (const p of corners) {
      const dx = p.x - this.origin.x;
      const dy = p.y - this.origin.y;
      const lx = dx * cosYaw + dy * sinYaw;
      const ly = -dx * sinYaw + dy * cosYaw;
      if (lx < minLx) minLx = lx;
      if (lx > maxLx) maxLx = lx;
      if (ly < minLy) minLy = ly;
      if (ly > maxLy) maxLy = ly;
    }

    const c0 = Math.max(0, Math.floor(minLx / this.res / this.tileSize));
    const c1 = Math.min(this.cols - 1, Math.floor(maxLx / this.res / this.tileSize));
    const r0 = Math.max(0, Math.floor(minLy / this.res / this.tileSize));
    const r1 = Math.min(this.rows - 1, Math.floor(maxLy / this.res / this.tileSize));

    const now = performance.now();
    const k = zoomTransform.k;
    const scale = k * this.res;

    // 2. Draw visible tiles
    for (let r = r0; r <= r1; r++) {
      for (let c = c0; c <= c1; c++) {
        const t = this.tiles[r][c];
        if (t.dirty) this._rebuildTile(r, c);

        // Map-local coordinates of the tile's bottom-left corner (min x, min y)
        const lx = c * this.tileSize * this.res;
        const ly = r * this.tileSize * this.res;

        // Transform to World coordinates
        const wx = this.origin.x + lx * cosYaw - ly * sinYaw;
        const wy = this.origin.y + lx * sinYaw + ly * cosYaw;

        // Project to Screen coordinates
        const [sx, sy] = world2screenFn(wx, wy);

        ctx2d.save();
        ctx2d.setTransform(1, 0, 0, 1, 0, 0);
        ctx2d.translate(sx, sy);
        ctx2d.rotate(-this.origin.yaw);
        ctx2d.scale(scale, scale);

        // Draw image. 
        // Image (0,0) is Top-Left (High Map Y).
        // We are at (sx, sy) which corresponds to Low Map Y.
        // We want Image (0, h) [Low Map Y] to be at (0,0).
        // So we draw at (0, -t.th).
        ctx2d.drawImage(t.canvas, 0, -t.th);

        // Flash overlay
        if (t.flashUntil > now) {
          const alpha = Math.min(0.15, ((t.flashUntil - now) / 120) * 0.15);
          ctx2d.globalAlpha = alpha;
          ctx2d.fillStyle = "#60a5fa";
          ctx2d.fillRect(0, -t.th, t.tw, t.th);
        }

        ctx2d.restore();
      }
    }
  }
}

// ==============================
// Live map WS (ignored while in bag mode)
// ==============================
function connectMapWS(): void {
  const isDev = location.port === "5173";
  const wsBase = isDev ? `ws://${location.hostname}:8000` : `ws://${location.host}`;
  const ws = new WebSocket(`${wsBase}/ws/map`);
  ws.onopen = () => setStatus("Map WS connected");
  ws.onclose = () => {
    setStatus("Map WS closed, retrying…");
    setTimeout(connectMapWS, 1000);
  };
  ws.onerror = () => setStatus("Map WS error");
  ws.onmessage = (ev: MessageEvent<string>) => {
    if (isBagMode) return; // do not let WS override bag snapshots
    let msg: MapWSMessage | any;
    try {
      msg = JSON.parse(ev.data);
    } catch {
      return;
    }
    if (!msg || typeof msg.type !== "string") return;

    if (msg.type === "map_meta") {
      window._latestMapMeta = msg.meta as MapMeta;
    } else if (msg.type === "map_full") {
      const meta = window._latestMapMeta;
      if (!meta) return;
      mapLayer = new MapTiles(meta);
      currentMapVersion = meta.version ?? 0;
      mapLayer.loadFull(Int8Array.from((msg as any).data as number[]));
      setStatus(`Map v${meta.version} loaded`);
    } else if (msg.type === "map_updates") {
      if (!mapLayer) return;
      const updates = (msg as any).updates as Array<{ x: number; y: number; w: number; h: number; data: number[] }>;
      for (const up of updates) {
        const { x, y, w, h, data } = up;
        const c0 = Math.floor(x / mapLayer.tileSize),
          c1 = Math.floor((x + w - 1) / mapLayer.tileSize);
        const r0 = Math.floor(y / mapLayer.tileSize),
          r1 = Math.floor((y + h - 1) / mapLayer.tileSize);
        for (let yy = 0; yy < h; yy++) {
          const srcOff = yy * w;
          const dstOff = (y + yy) * mapLayer.w + x;
          mapLayer.data.set(Int8Array.from(data.slice(srcOff, srcOff + w)), dstOff);
        }
        const now = performance.now();
        for (let r = r0; r <= r1; r++)
          for (let c = c0; c <= c1; c++)
            if (r >= 0 && r < mapLayer.rows && c >= 0 && c < mapLayer.cols) {
              mapLayer.tiles[r][c].dirty = true;
              mapLayer.tiles[r][c].flashUntil = now + 120;
            }
      }
    }
  };
}
connectMapWS();

// ==============================
// Map fetch at time (bag mode)
// ==============================
async function fetchMapAtRel(rel: number): Promise<void> {
  if (!isBagMode || t0 == null) return;
  const absT = t0 + rel;
  isMapLoading = true;
  try {
    const r = await fetch(`/api/v1/map_full_at?t=${encodeURIComponent(absT.toFixed(3))}`);
    const j = (await r.json()) as MapFullAtResponse | Record<string, unknown>;
    if (!("meta" in j) || !("data" in j)) return;
    const meta = (j as MapFullAtResponse).meta;
    const version = meta.version ?? 0;

    const needsRebuild =
      !mapLayer ||
      version !== currentMapVersion ||
      mapLayer.w !== meta.width ||
      mapLayer.h !== meta.height ||
      mapLayer.res !== meta.resolution ||
      mapLayer.origin.x !== meta.origin.x ||
      mapLayer.origin.y !== meta.origin.y;

    if (needsRebuild) {
      mapLayer = new MapTiles(meta);
      currentMapVersion = version;
    }
    mapLayer.loadFull(Int8Array.from((j as MapFullAtResponse).data));
  } finally {
    isMapLoading = false;
  }
}

// Map patches between two absolute times (bag mode)
async function fetchMapDeltaBetween(absT0: number, absT1: number): Promise<void> {
  if (!isBagMode || t0 == null) return;
  if (!mapLayer) {
    // If we somehow don't have a base, load full at target then return
    const rel = absT1 - t0;
    await fetchMapAtRel(rel);
    return;
  }
  if (deltaBusy) return;
  deltaBusy = true;
  try {
    const url = `/api/v1/map_delta?t0=${encodeURIComponent(absT0.toFixed(3))}&t1=${encodeURIComponent(
      absT1.toFixed(3)
    )}`;
    const r = await fetch(url);
    const j = (await r.json()) as MapDeltaResponse;

    if (!j) return;

    if (j.reset) {
      const meta = j.reset.meta;
      mapLayer = new MapTiles(meta);
      currentMapVersion = meta.version ?? 0;
      mapLayer.loadFull(Int8Array.from(j.reset.data));
      return;
    }

    if (typeof j.version === "number" && j.version !== currentMapVersion) {
      // Safety: version drift; pull full at t1
      await fetchMapAtRel(absT1 - (t0 ?? 0));
      currentMapVersion = j.version;
      return;
    }

    const updates = j.patches || [];
    if (!updates.length) return;
    for (const up of updates) {
      const { x, y, w, h, data } = up;
      const c0 = Math.floor(x / mapLayer.tileSize),
        c1 = Math.floor((x + w - 1) / mapLayer.tileSize);
      const r0 = Math.floor(y / mapLayer.tileSize),
        r1 = Math.floor((y + h - 1) / mapLayer.tileSize);
      for (let yy = 0; yy < h; yy++) {
        const srcOff = yy * w;
        const dstOff = (y + yy) * mapLayer.w + x;
        mapLayer.data.set(Int8Array.from(data.slice(srcOff, srcOff + w)), dstOff);
      }
      const now = performance.now();
      for (let r2 = r0; r2 <= r1; r2++)
        for (let c2 = c0; c2 <= c1; c2++)
          if (r2 >= 0 && r2 < mapLayer.rows && c2 >= 0 && c2 < mapLayer.cols) {
            mapLayer.tiles[r2][c2].dirty = true;
            mapLayer.tiles[r2][c2].flashUntil = now + 120;
          }
    }
  } catch {
    // ignore network errors in the play loop
  } finally {
    deltaBusy = false;
  }
}

async function fetchCostmapsAtRel(rel: number): Promise<void> {
  console.log(`[costmap] fetchAtRel ${rel}, isBag=${isBagMode}, t0=${t0}`);
  if (!isBagMode || t0 == null) return;
  const absT = t0 + rel;
  console.log(`[costmap] fetching for absT=${absT}`);

  // Global
  try {
    const r = await fetch(`/api/v1/costmap/global/full_at?t=${encodeURIComponent(absT.toFixed(3))}`);
    if (r.ok) {
      const j = (await r.json()) as MapFullAtResponse;
      const meta = j.meta;
      const version = meta.version ?? 0;
      const needsRebuild =
        !globalCostmapLayer ||
        version !== currentGlobalCostmapVersion ||
        globalCostmapLayer.w !== meta.width ||
        globalCostmapLayer.h !== meta.height;

      if (needsRebuild) {
        globalCostmapLayer = new MapTiles(meta, globalCostmapPalette);
        currentGlobalCostmapVersion = version;
      }
      globalCostmapLayer!.loadFull(Int8Array.from(j.data));
    } else {
      // If 404 or empty, maybe clear it?
      // globalCostmapLayer = null; 
    }
  } catch {
    // ignore
  }

  // Local
  try {
    const r = await fetch(`/api/v1/costmap/local/full_at?t=${encodeURIComponent(absT.toFixed(3))}`);
    if (r.ok) {
      const j = (await r.json()) as MapFullAtResponse;
      const meta = j.meta;
      const version = meta.version ?? 0;
      const needsRebuild =
        !localCostmapLayer ||
        version !== currentLocalCostmapVersion ||
        localCostmapLayer.w !== meta.width ||
        localCostmapLayer.h !== meta.height;

      if (needsRebuild) {
        localCostmapLayer = new MapTiles(meta, localCostmapPalette);
        currentLocalCostmapVersion = version;
      }
      localCostmapLayer!.loadFull(Int8Array.from(j.data));
    } else {
      // localCostmapLayer = null;
    }
  } catch {
    // ignore
  }
}

// ==============================
// Lidar scan fetch (bag mode)
// ==============================
async function requestScanAtAbs(absT: number): Promise<void> {
  if (!isBagMode || t0 == null) return;
  if (scanReqInFlight) return;
  // if (lastScanReqAbsT != null && Math.abs(absT - lastScanReqAbsT) < 0.03) return; // throttle

  scanReqInFlight = true;
  lastScanReqAbsT = absT;

  try {
    const url = `/api/v1/scan_at?t=${encodeURIComponent(absT.toFixed(3))}`;
    const r = await fetch(url);
    if (!r.ok) {
      return;
    }
    const j = (await r.json()) as ScanAtResponse;
    if (!j || !Array.isArray(j.points) || j.count <= 0) {
      scanPoints = null;
      return;
    }
    scanPoints = new Float32Array(j.points);
  } catch {
    // ignore network errors; keep last scan
  } finally {
    scanReqInFlight = false;
  }
}

async function requestPlanAtAbs(absT: number): Promise<void> {
  if (!isBagMode || t0 == null || !showPlan) return;
  if (planReqInFlight) return;

  planReqInFlight = true;
  lastPlanReqAbsT = absT;

  try {
    const url = `/api/v1/plan_at?t=${encodeURIComponent(absT.toFixed(3))}`;
    const r = await fetch(url);
    if (!r.ok) {
      planPath = null;
      return;
    }
    const j = (await r.json()) as PlanResponse;
    planPath = j.poses;
  } catch {
    // ignore
  } finally {
    planReqInFlight = false;
  }
}

async function requestGoalAtAbs(absT: number): Promise<void> {
  if (!isBagMode || t0 == null || !showGoal) return;
  if (goalReqInFlight) return;

  goalReqInFlight = true;
  lastGoalReqAbsT = absT;

  try {
    const url = `/api/v1/goal_at?t=${encodeURIComponent(absT.toFixed(3))}`;
    const r = await fetch(url);
    if (!r.ok) {
      goalPose = null;
      return;
    }
    const j = (await r.json()) as GoalResponse;
    goalPose = { x: j.x, y: j.y, yaw: j.yaw };
  } catch {
    // ignore
  } finally {
    goalReqInFlight = false;
  }
}

// ==============================
// Bag panel
// ==============================
async function loadBagListIntoPanel(): Promise<void> {
  if (!bagPanelStatus || !bagList) return;
  bagPanelStatus.textContent = "Loading…";
  bagList.innerHTML = "";
  try {
    const j = (await (await fetch("/api/v1/bag/list")).json()) as { bags?: Array<{ name: string; size: number }> };
    const bags = j.bags || [];
    if (!bags.length) {
      bagPanelStatus.textContent = "No bag files found.";
      return;
    }
    bagPanelStatus.textContent = "";
    bags.forEach((b) => {
      const li = document.createElement("li");
      li.textContent = b.name;
      const size = document.createElement("small");
      size.textContent = `${(b.size / 1024).toFixed(1)} KB`;
      li.appendChild(size);
      li.onclick = () => startBagReplay(b.name);
      bagList.appendChild(li);
    });
  } catch {
    bagPanelStatus.textContent = "Failed to load bag list.";
  }
}
function openBagPanel(): void {
  if (!bagPanel) return;
  bagPanel.classList.remove("hidden");
  void loadBagListIntoPanel();
}
function closeBagPanel(): void {
  if (!bagPanel) return;
  bagPanel.classList.add("hidden");
}
if (bagPanelBtn) bagPanelBtn.onclick = openBagPanel;
if (bagPanelClose) bagPanelClose.onclick = closeBagPanel;
if (bagPanel)
  bagPanel.addEventListener("click", (ev: MouseEvent) => {
    if (ev.target === bagPanel) closeBagPanel();
  });

// ==============================
// Start bag (jump to END immediately)
// ==============================
async function startBagReplay(name: string): Promise<void> {
  stopPlayback(); // clean playback state
  let resp: Response;
  try {
    resp = await fetch("/api/v1/bag/play", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    });
  } catch {
    setStatus("Bag start failed (network)");
    return;
  }
  const j = (await resp.json()) as { ok?: boolean; error?: string };
  if (!j.ok) {
    setStatus(j.error || "Replay failed");
    return;
  }

  isBagMode = true;
  liveMode = false;

  // Hide the small "Live" button and show Play/Pause (if present)
  if (liveBtn) liveBtn.style.display = "none";
  if (playBtn) playBtn.style.display = "";

  path = [];
  scanPoints = null;
  lastScanReqAbsT = null;
  scanReqInFlight = false;

  // reset cmd overlay stats for new bag
  cmdStats = null;
  lastCmdStatsReqAbsT = null;
  cmdStatsReqInFlight = false;
  updateCmdOverlayNumbers();

  setStatus(`Replaying bag: ${name}`);
  closeBagPanel();

  // Wait for pose_history to stabilize, then jump to end
  await waitForStablePoseHistory({ timeoutMs: 5000, settleMs: 600, pollMs: 150 });
  setTimelineFromPoseHist();
  setTimelineToEnd();

  // Fetch exact map at end and prime delta state
  const dur = getDuration();
  await fetchMapAtRel(dur);
  await fetchCostmapsAtRel(dur);
  lastAbsTForDelta = (t0 ?? 0) + dur;
  if (t0 != null) {
    const absEnd = t0 + dur;
    requestScanAtAbs(absEnd);
    requestPlanAtAbs(absEnd);
    requestGoalAtAbs(absEnd);
    if (cmdOverlayEnabled) requestCmdStatsAtAbs(absEnd);
  }

  // Fetch robot description (URDF)
  void fetchRobotDescription();
}

// ==============================

function world2screen(x: number, y: number): [number, number] {
  const X = zoomTransform.applyX(x);
  const Y = zoomTransform.applyY(-y); // +y up
  return [X, Y];
}

// 50% path marker (bag) — progressive refine at ~25Hz
// ==============================
function startPlayback(): void {
  if (!isBagMode || isPlaying) return;
  isPlaying = true;
  lastTickTs = null;
  if (playBtn) playBtn.textContent = "Pause";

  // Prime lastAbsT for delta
  const relNow = Number(slider!.value) || 0;
  lastAbsTForDelta = (t0 ?? 0) + relNow;

  // Start the synchronized loop
  void playLoop();
}

function stopPlayback(): void {
  isPlaying = false;
  lastTickTs = null;
  if (playBtn) playBtn.textContent = "Play";
}

async function playLoop(): Promise<void> {
  if (!isPlaying || !isBagMode || t0 == null) return;

  const now = performance.now();
  if (lastTickTs == null) {
    lastTickTs = now;
  }

  // Calculate dt, but clamp it to avoid huge jumps if fetching takes long
  // We want to advance time by actual elapsed time, but if the loop is slow,
  // we effectively slow down playback.
  // Let's say we target real-time.
  let dt = (now - lastTickTs) / 1000;
  lastTickTs = now;

  // Cap dt to e.g. 0.1s to prevent skipping too much if we lagged
  if (dt > 0.1) dt = 0.1;

  const dur = getDuration();
  let rel = Number(slider!.value) || 0;
  rel += dt * playRate;

  if (rel >= dur) {
    rel = dur;
    updateTimeUI(rel);
    scrubTime = (t0 ?? 0) + rel;
    scrubPose = getBagPoseAtRel(rel);

    // Final fetch at end
    const absT = (t0 ?? 0) + rel;
    await Promise.all([
      fetchMapDeltaBetween(lastAbsTForDelta!, absT),
      requestScanAtAbs(absT),
      requestPlanAtAbs(absT),
      requestGoalAtAbs(absT),
      cmdOverlayEnabled ? requestCmdStatsAtAbs(absT) : Promise.resolve(),
      fetchCostmapsAtRel(rel),
      requestPublishAtAbs(absT)
    ]);

    stopPlayback();
    return;
  }

  updateTimeUI(rel);
  scrubTime = (t0 ?? 0) + rel;
  scrubPose = getBagPoseAtRel(rel);

  const absT = (t0 ?? 0) + rel;

  // Synchronized fetch: wait for ALL data before proceeding to next frame
  await Promise.all([
    fetchMapDeltaBetween(lastAbsTForDelta!, absT),
    requestScanAtAbs(absT),
    requestPlanAtAbs(absT),
    requestGoalAtAbs(absT),
    cmdOverlayEnabled ? requestCmdStatsAtAbs(absT) : Promise.resolve(),
    fetchCostmapsAtRel(rel),
    requestPublishAtAbs(absT)
  ]);

  lastAbsTForDelta = absT;

  // Schedule next iteration immediately
  if (isPlaying) {
    // We use setTimeout(..., 0) or just call recursively?
    // Recursive call might stack overflow if not careful, but async function returns Promise, so it's fine.
    // Better to use requestAnimationFrame to yield to UI thread, 
    // but we want to drive this as fast as possible (or as slow as needed).
    // requestAnimationFrame might be too fast if we just did a heavy fetch?
    // No, requestAnimationFrame is good for rendering.
    // Actually, we just finished a heavy fetch. We should let the browser render.
    requestAnimationFrame(() => void playLoop());
  }
}

if (playBtn) {
  playBtn.onclick = () => {
    if (!isBagMode) return;
    if (isPlaying) stopPlayback();
    else startPlayback();
  };
}

// ==============================
// "Go back to Live"
// ==============================
liveModeBtn.onclick = async () => {
  stopPlayback();
  try {
    await fetch("/api/v1/reset", { method: "POST" });
  } catch {
    /* ignore */
  }
  isBagMode = false;
  liveMode = true;

  if (liveBtn) liveBtn.style.display = "";
  if (playBtn) playBtn.style.display = "none";

  scanPoints = null;
  lastScanReqAbsT = null;
  scanReqInFlight = false;

  // cmd overlay is still visible but there is no bag-based data anymore
  cmdStats = null;
  lastCmdStatsReqAbsT = null;
  cmdStatsReqInFlight = false;
  updateCmdOverlayNumbers();

  await loadPoseHistoryOnce();
  setTimelineFromPoseHist();
  jumpToLive();
  setStatus("Back to live mode.");
};

// ==============================
// WS: path/pose (both modes)
// ==============================
function connectPathWS(): void {
  const isDev = location.port === "5173";
  const wsBase = isDev ? `ws://${location.hostname}:8000` : `ws://${location.host}`;
  const ws = new WebSocket(`${wsBase}/ws/stream`);
  ws.onopen = () => setStatus("Connected (path)");
  ws.onclose = () => {
    setStatus("Disconnected (path). Reconnecting…");
    setTimeout(connectPathWS, 1000);
  };
  ws.onerror = () => setStatus("WS error (path)");
  ws.onmessage = (ev: MessageEvent<string>) => {
    let msg: PathWSMessage | any;
    try {
      msg = JSON.parse(ev.data);
    } catch {
      return;
    }
    if (!msg || typeof msg.type !== "string") return;

    if (msg.type === "snapshot") {
      path = (msg.path as Array<[number, number]>) || [];
      pose = (msg.pose as typeof pose) || pose;
    } else if (msg.type === "append") {
      if (Array.isArray(msg.append) && msg.append.length > 0) {
        path.push(...(msg.append as Array<[number, number]>));
        const max = 10000;
        if (path.length > max) path = path.slice(path.length - max);
      }
      if (msg.pose) pose = msg.pose as typeof pose;
    }
  };
}
connectPathWS();

// ==============================
// Live poller (disabled in bag mode)
// ==============================
setInterval(async () => {
  if (isBagMode) return;

  let j: { count?: number; t0?: number; t1?: number } = {};
  try {
    j = (await (await fetch("/api/v1/pose_history_meta")).json()) as any;
  } catch {
    // ignore
  }
  if ((j.count || 0) > lastPoseCount) {
    await loadPoseHistoryOnce();
    setTimelineFromPoseHist();
  }
  if (liveMode && poseHist.length && t0 != null) {
    const tMax = poseHist[poseHist.length - 1].t;
    const rel = tMax - t0;
    updateTimeUI(rel);
  }
}, 1000);

// ==============================
// Slider interactions
//  - input: scrub pose immediately (map waits)
//  - change: lazy-load exact map at that time in bag mode and reset delta base
// ==============================
slider!.addEventListener("input", () => {
  if (!poseHist.length || t0 == null) return;
  stopPlayback(); // scrubbing pauses playback
  liveMode = false; // disable live auto-follow
  const rel = Number(slider!.value);
  updateTimeUI(rel);
  scrubTime = t0 + rel;
  scrubPose = poseAt(scrubTime);
  if (isBagMode && scrubTime != null) {
    requestScanAtAbs(scrubTime);
    requestPlanAtAbs(scrubTime);
    requestGoalAtAbs(scrubTime);
    if (cmdOverlayEnabled) requestCmdStatsAtAbs(scrubTime);
    requestPublishAtAbs(scrubTime);
  }
});

slider!.addEventListener("change", async () => {
  if (!poseHist.length || t0 == null) return;
  if (!isBagMode) return; // live mode uses WS/REST live map
  const rel = Number(slider!.value);
  await fetchMapAtRel(rel);
  await fetchCostmapsAtRel(rel);
  const absT = t0 + rel;
  lastAbsTForDelta = absT; // resync delta base to playhead
  requestScanAtAbs(absT);
  requestPlanAtAbs(absT);
  requestGoalAtAbs(absT);
  if (cmdOverlayEnabled) requestCmdStatsAtAbs(absT);
  requestPublishAtAbs(absT);
});

// Jump-to-live (ignored in bag mode)
if (liveBtn) {
  liveBtn.onclick = () => {
    if (isBagMode) return;
    liveMode = true;
    jumpToLive();
  };
}
function jumpToLive(): void {
  if (!poseHist.length || t0 == null) return;
  const tMax = poseHist[poseHist.length - 1].t;
  const rel = tMax - t0;
  updateTimeUI(rel);
  scrubTime = null;
  scrubPose = null;
}

// ==============================
// cmd_vel overlay toggle
// ==============================
// ==============================
// View Menu Interactions
// ==============================
if (toggleGlobalCostmap) {
  toggleGlobalCostmap.addEventListener("change", () => {
    showGlobalCostmap = toggleGlobalCostmap.checked;
  });
}
if (toggleLocalCostmap) {
  toggleLocalCostmap.addEventListener("change", () => {
    showLocalCostmap = toggleLocalCostmap.checked;
  });
}
if (toggleLidar) {
  toggleLidar.addEventListener("change", () => {
    showLidar = toggleLidar.checked;
  });
}
if (togglePlan) {
  togglePlan.addEventListener("change", () => {
    showPlan = togglePlan.checked;
    if (showPlan && isBagMode && t0 != null) {
      const rel = Number(slider!.value) || 0;
      requestPlanAtAbs(t0 + rel);
    } else if (!showPlan) {
      planPath = null;
    }
  });
}
if (toggleGoal) {
  toggleGoal.addEventListener("change", () => {
    showGoal = toggleGoal.checked;
    if (showGoal && isBagMode && t0 != null) {
      const rel = Number(slider!.value) || 0;
      requestGoalAtAbs(t0 + rel);
    } else if (!showGoal) {
      goalPose = null;
    }
  });
}
if (toggleCmdOverlay) {
  toggleCmdOverlay.addEventListener("change", () => {
    const next = toggleCmdOverlay.checked;
    cmdOverlayEnabled = next;
    updateCmdOverlayVisibility();
    if (next) {
      if (isBagMode && t0 != null) {
        const rel = Number(slider!.value) || 0;
        const absT = t0 + rel;
        requestCmdStatsAtAbs(absT);
      } else {
        cmdStats = null;
        updateCmdOverlayNumbers();
      }
    }
  });
}

// ==============================
// Optional: initial REST map fallback on app load
// ==============================
setTimeout(async () => {
  try {
    const meta = (await (await fetch("/api/v1/map_meta")).json()) as MapMeta | Record<string, unknown>;
    if (!meta || !(meta as any).width) return;
    const full = (await (await fetch("/api/v1/map_full")).json()) as MapFullResponse | Record<string, unknown>;
    if (!("data" in full) || !(full as MapFullResponse).data || !(full as MapFullResponse).data.length) return;
    mapLayer = new MapTiles(meta as MapMeta);
    currentMapVersion = (meta as MapMeta).version ?? 0;
    mapLayer.loadFull(Int8Array.from((full as MapFullResponse).data));
    setStatus(`(Fallback) Map v${(meta as MapMeta).version} loaded`);
  } catch {
    /* ignore */
  }
  // Try to fetch robot description on startup (for live mode or if bag already has it)
  void fetchRobotDescription();
}, 1500);

// ==============================

// ... (RobotShape and robotDims remain same)

// ... (fetchRobotDescription remains same)

// ... (loadPoseHistoryOnce remains same)

function updateHalfwayPoint(currentPose: { x: number; y: number }): void {
  if (!poseHist.length || t0 == null || !poseHistDist) return;

  const rel = Number(slider!.value) || 0;
  const absT = t0 + rel;

  // 1. Dynamic Calculation (Always run this)
  // Prevent premature triggering:
  // Require a valid plan and min distance
  if (!planPath || planPath.length === 0) {
    halfwayPoint = null;
    halfwayT = null;
    return;
  }

  // 1. Calculate History Distance
  let idx = 0;
  while (idx < poseHist.length && poseHist[idx].t <= absT) {
    idx++;
  }
  const lastIdx = Math.max(0, idx - 1);

  let distHistory = poseHistDist[lastIdx];
  const dx = currentPose.x - poseHist[lastIdx].x;
  const dy = currentPose.y - poseHist[lastIdx].y;
  distHistory += Math.sqrt(dx * dx + dy * dy);

  // 2. Calculate Plan Distance
  let distPlan = 0;
  if (planPath && planPath.length > 0) {
    const dx0 = planPath[0][0] - currentPose.x;
    const dy0 = planPath[0][1] - currentPose.y;
    distPlan += Math.sqrt(dx0 * dx0 + dy0 * dy0);

    for (let i = 1; i < planPath.length; i++) {
      const dx = planPath[i][0] - planPath[i - 1][0];
      const dy = planPath[i][1] - planPath[i - 1][1];
      distPlan += Math.sqrt(dx * dx + dy * dy);
    }
  }

  // 3. Total Estimated Distance
  const totalDist = distHistory + distPlan;

  if (totalDist < 1.0) {
    halfwayPoint = null;
    halfwayT = null;
    return;
  }

  const targetDist = totalDist * 0.5;

  // 4. Determine Reached Status & Calculate Candidate Point
  let candidatePoint: { x: number, y: number, reached: boolean } | null = null;
  let candidateT: { t: number, reached: boolean } | null = null;
  const isReached = targetDist <= distHistory;

  if (isReached) {
    // It's in history
    let k = 0;
    while (k < lastIdx && poseHistDist[k + 1] <= targetDist) {
      k++;
    }

    if (k < lastIdx) {
      const dStart = poseHistDist[k];
      const dEnd = poseHistDist[k + 1];
      const ratio = (targetDist - dStart) / (dEnd - dStart);
      candidatePoint = {
        x: poseHist[k].x + (poseHist[k + 1].x - poseHist[k].x) * ratio,
        y: poseHist[k].y + (poseHist[k + 1].y - poseHist[k].y) * ratio,
        reached: true
      };
    } else {
      const dStart = poseHistDist[lastIdx];
      const dEnd = distHistory;
      const ratio = (targetDist - dStart) / Math.max(1e-6, dEnd - dStart);
      candidatePoint = {
        x: poseHist[lastIdx].x + (currentPose.x - poseHist[lastIdx].x) * ratio,
        y: poseHist[lastIdx].y + (currentPose.y - poseHist[lastIdx].y) * ratio,
        reached: true
      };
    }

    // Calculate T for reached point
    // Search targetDist in FULL poseHistDist
    let m = 0;
    if (poseHistDist[poseHistDist.length - 1] < targetDist) {
      candidateT = null; // Should not happen if reached is true based on current history
    } else {
      while (m < poseHistDist.length - 1 && poseHistDist[m + 1] <= targetDist) {
        m++;
      }
      const dStart = poseHistDist[m];
      const dEnd = poseHistDist[m + 1];
      const ratio = (targetDist - dStart) / Math.max(1e-6, dEnd - dStart);
      const tInterp = poseHist[m].t + (poseHist[m + 1].t - poseHist[m].t) * ratio;
      candidateT = { t: tInterp, reached: true };
    }

  } else {
    // It's in plan (Future)
    let d = distHistory;
    if (planPath && planPath.length > 0) {
      const dx0 = planPath[0][0] - currentPose.x;
      const dy0 = planPath[0][1] - currentPose.y;
      const segLen0 = Math.sqrt(dx0 * dx0 + dy0 * dy0);

      if (d + segLen0 >= targetDist) {
        const remain = targetDist - d;
        const ratio = remain / segLen0;
        candidatePoint = {
          x: currentPose.x + dx0 * ratio,
          y: currentPose.y + dy0 * ratio,
          reached: false
        };
      } else {
        d += segLen0;
        let found = false;
        for (let i = 1; i < planPath.length; i++) {
          const dx = planPath[i][0] - planPath[i - 1][0];
          const dy = planPath[i][1] - planPath[i - 1][1];
          const segLen = Math.sqrt(dx * dx + dy * dy);
          if (d + segLen >= targetDist) {
            const remain = targetDist - d;
            const ratio = remain / segLen;
            candidatePoint = {
              x: planPath[i - 1][0] + dx * ratio,
              y: planPath[i - 1][1] + dy * ratio,
              reached: false
            };
            found = true;
            break;
          }
          d += segLen;
        }
        if (!found) candidatePoint = null;
      }
    } else {
      candidatePoint = null;
    }

    // Future T is unknown/not in bag
    candidateT = null;
  }

  // 5. Latch Logic
  // If we found a reached point and haven't latched yet, latch it.
  if (!fixedHalfway && candidatePoint && candidatePoint.reached && candidateT && candidateT.reached) {
    fixedHalfway = {
      t: candidateT.t,
      x: candidatePoint.x,
      y: candidatePoint.y
    };
  }

  // 6. Hybrid Display Logic
  // Map Marker:
  // - If Fixed AND Past Fixed Time: Show Fixed (Blue)
  // - Else: Show Dynamic (Gray)
  if (fixedHalfway && absT >= fixedHalfway.t) {
    halfwayPoint = { x: fixedHalfway.x, y: fixedHalfway.y, reached: true };
  } else {
    halfwayPoint = candidatePoint;
    // Force reached=false if we are showing dynamic (even if candidate says reached,
    // if we are here it means we are before the fixed time or not fixed yet)
    if (halfwayPoint) halfwayPoint.reached = false;
  }

  // Timeline Marker:
  // - If Fixed: Show Fixed Time (Gray/Blue based on current time)
  // - Else: Show Dynamic Time
  if (fixedHalfway) {
    const isPast = absT >= fixedHalfway.t;
    halfwayT = { t: fixedHalfway.t, reached: isPast };
  } else {
    halfwayT = candidateT;
  }

  // 7. Visibility Logic
  if (halfwayPoint) {
    updateTimelineMarker();
  } else {
    const marker = document.getElementById("halfwayMarker");
    if (marker) marker.style.display = "none";
  }
}

function updateTimelineMarker(): void {
  let marker = document.getElementById("halfwayMarker");
  if (!marker) {
    marker = document.createElement("div");
    marker.id = "halfwayMarker";
    marker.style.position = "absolute";
    marker.style.width = "4px";
    marker.style.height = "16px"; // Taller than slider track
    marker.style.top = "22px"; // Center vertically over slider roughly
    marker.style.pointerEvents = "none";
    marker.style.zIndex = "10";
    marker.style.borderRadius = "2px";

    const container = document.getElementById("timeControls");
    if (container) {
      container.style.position = "relative";
      container.appendChild(marker);
    }
  }

  const dur = getDuration();
  // Show only if we have a valid time in the past (t > 0) and it's within range
  if (halfwayT && halfwayT.t > 0 && t0 != null && dur > 0) {
    const relT = halfwayT.t - t0;
    // Clamp to 0..dur just in case
    if (relT >= 0 && relT <= dur) {
      // Color based on reached status
      marker.style.backgroundColor = halfwayT.reached ? "#3b82f6" : "#9ca3af"; // Blue-500 vs Gray-400

      // Try to get slider's bounding rect relative to container
      const sliderEl = document.getElementById("timeSlider");
      const container = document.getElementById("timeControls");
      if (sliderEl && container) {
        const sRect = sliderEl.getBoundingClientRect();
        const cRect = container.getBoundingClientRect();
        const leftOffset = sRect.left - cRect.left;
        const width = sRect.width;

        // thumb width correction (approx 16px)
        const thumbW = 16;
        const trackW = width - thumbW;
        const px = leftOffset + (thumbW / 2) + (relT / dur) * trackW;

        marker.style.left = `${px}px`;
        marker.style.top = `${(sRect.top - cRect.top) + (sRect.height / 2) - 8}px`; // Center vertically
        marker.style.display = "block";
        return;
      }
    }
  }
  if (marker) marker.style.display = "none";
}

function drawRobot(xm: number, ym: number, yaw: number, color = "#f59e0b"): void {
  const [x, y] = world2screen(xm, ym);

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);

  if (robotDims) {
    const k = zoomTransform.k;

    if (robotDims.type === "a300") {
      // Dimensions
      const chassisL = robotDims.length * k;
      const chassisW = robotDims.width * k;
      const wheelBase = (robotDims.wheel_base || 0.512) * k;
      const wheelTrack = (robotDims.wheel_track || 0.562) * k;
      const wheelRad = (robotDims.wheel_radius || 0.1651) * k;
      const wheelW = (robotDims.wheel_width || 0.1143) * k;
      const wheelDiam = wheelRad * 2;

      // Colors
      const colChassis = "#f59e0b"; // Husky Yellow
      const colDark = "#333333";    // Dark Grey/Black
      const colBumper = "#1f2937";  // Darker Grey

      // 1. Suspension Beams (connects wheels on each side)
      // Approx size: length = wheelBase, width = small
      const beamW = 0.05 * k;
      ctx.fillStyle = colDark;
      // Left Beam
      ctx.fillRect(-wheelBase / 2, wheelTrack / 2 - beamW / 2, wheelBase, beamW);
      // Right Beam
      ctx.fillRect(-wheelBase / 2, -wheelTrack / 2 - beamW / 2, wheelBase, beamW);

      // 2. Wheels
      ctx.fillStyle = colDark;
      // Front Left
      ctx.fillRect(wheelBase / 2 - wheelDiam / 2, wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);
      // Front Right
      ctx.fillRect(wheelBase / 2 - wheelDiam / 2, -wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);
      // Rear Left
      ctx.fillRect(-wheelBase / 2 - wheelDiam / 2, wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);
      // Rear Right
      ctx.fillRect(-wheelBase / 2 - wheelDiam / 2, -wheelTrack / 2 - wheelW / 2, wheelDiam, wheelW);

      // 3. Chassis Body
      ctx.fillStyle = colChassis;
      ctx.fillRect(-chassisL / 2, -chassisW / 2, chassisL, chassisW);

      // 4. Bumpers (Front and Rear)
      // Approx size: slightly wider than chassis, thin
      const bumperDepth = 0.05 * k;
      const bumperWidth = chassisW + 0.02 * k;
      ctx.fillStyle = colBumper;

      // Front Bumper
      ctx.fillRect(chassisL / 2, -bumperWidth / 2, bumperDepth, bumperWidth);
      // Rear Bumper
      ctx.fillRect(-chassisL / 2 - bumperDepth, -bumperWidth / 2, bumperDepth, bumperWidth);

      // 5. Direction Indicator / Top Plate details
      // Draw a "front" indicator on the chassis
      ctx.fillStyle = "rgba(0,0,0,0.2)";
      ctx.fillRect(chassisL * 0.1, -chassisW * 0.3, chassisL * 0.3, chassisW * 0.6); // "Top plate" area

      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.moveTo(chassisL * 0.3, 0);
      ctx.lineTo(chassisL * 0.45, 0);
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.stroke();

      // Arrow head
      ctx.beginPath();
      ctx.moveTo(chassisL * 0.45, 0);
      ctx.lineTo(chassisL * 0.4, chassisW * 0.05);
      ctx.lineTo(chassisL * 0.4, -chassisW * 0.05);
      ctx.fill();

    } else {
      // Render based on URDF dimensions (Box/Cylinder)
      const wPx = robotDims.width * k;
      const lPx = robotDims.length * k;

      ctx.fillStyle = color;
      ctx.beginPath();
      if (robotDims.type === "cylinder") {
        ctx.arc(0, 0, wPx / 2, 0, Math.PI * 2);
      } else {
        ctx.rect(-lPx / 2, -wPx / 2, lPx, wPx);
      }
      ctx.fill();

      // Direction indicator
      ctx.fillStyle = "#ffffff";
      ctx.beginPath();
      ctx.moveTo(lPx / 4, 0);
      ctx.lineTo(lPx / 2, 0);
      ctx.stroke();
    }

  } else {
    // Fallback: Triangle
    const size = Math.max(8, 12 * (zoomTransform.k / 40) * 1.6);
    ctx.beginPath();
    ctx.moveTo(size, 0);
    ctx.lineTo(-size * 0.6, size * 0.5);
    ctx.lineTo(-size * 0.6, -size * 0.5);
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
  }
  ctx.restore();
}


function drawGrid(stepMeters: number, color: string): void {
  const k = zoomTransform.k;
  const stepPx = stepMeters * k;
  if (stepPx < 15) return;

  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;

  const invX0 = zoomTransform.invertX(0);
  const invX1 = zoomTransform.invertX(W);
  const invY0 = zoomTransform.invertY(0);
  const invY1 = zoomTransform.invertY(H);
  const yMin = -Math.max(invY0, invY1);
  const yMax = -Math.min(invY0, invY1);
  const xMin = Math.min(invX0, invX1);
  const xMax = Math.max(invX0, invX1);

  const x0 = Math.floor(xMin / stepMeters) * stepMeters;
  const y0 = Math.floor(yMin / stepMeters) * stepMeters;

  for (let x = x0; x <= xMax; x += stepMeters) {
    const [X1, Y1] = world2screen(x, yMin);
    const [X2, Y2] = world2screen(x, yMax);
    ctx.beginPath();
    ctx.moveTo(X1, Y1);
    ctx.lineTo(X2, Y2);
    ctx.stroke();
  }
  for (let y = y0; y <= yMax; y += stepMeters) {
    const [X1, Y1] = world2screen(xMin, y);
    const [X2, Y2] = world2screen(xMax, y);
    ctx.beginPath();
    ctx.moveTo(X1, Y1);
    ctx.lineTo(X2, Y2);
    ctx.stroke();
  }
  ctx.restore();
}

function drawLidar(): void {
  if (!scanPoints || scanPoints.length < 2) return;

  const baseRadius = 1.5 * (zoomTransform.k / 40);
  const radius = Math.max(1, baseRadius);

  ctx.save();
  ctx.fillStyle = "#ef4444"; // uniform red, similar to RViz

  for (let i = 0; i < scanPoints.length; i += 2) {
    const xm = scanPoints[i];
    const ym = scanPoints[i + 1];
    const [X, Y] = world2screen(xm, ym);
    ctx.beginPath();
    ctx.arc(X, Y, radius, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.restore();
}

function drawPlan(): void {
  if (!planPath || planPath.length < 2) return;

  ctx.save();
  ctx.strokeStyle = "#00ff00"; // Green for plan
  ctx.lineWidth = 1;
  ctx.setLineDash([5, 5]); // Dashed line

  ctx.beginPath();
  for (let i = 0; i < planPath.length; i++) {
    const [x, y] = world2screen(planPath[i][0], planPath[i][1]);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();

  // Draw a small marker at the end of the plan
  const last = planPath[planPath.length - 1];
  const [lx, ly] = world2screen(last[0], last[1]);
  const size = Math.max(4, 6 * (zoomTransform.k / 40));

  ctx.setLineDash([]); // Solid line for marker
  ctx.fillStyle = "#00ff00";
  ctx.beginPath();
  ctx.arc(lx, ly, size, 0, Math.PI * 2);
  ctx.fill();

  ctx.restore();
}

function drawGoal(): void {
  if (!goalPose) return;

  const [x, y] = world2screen(goalPose.x, goalPose.y);
  const size = Math.max(10, 15 * (zoomTransform.k / 40));

  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-goalPose.yaw);

  // Draw a target marker (circle with crosshair) or arrow
  // Let's draw a distinct arrow (Cyan)
  ctx.fillStyle = "#00ffff";
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.5, size * 0.5);
  ctx.lineTo(-size * 0.5, -size * 0.5);
  ctx.closePath();
  ctx.fill();

  // Add a ring
  ctx.strokeStyle = "#00ffff";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(0, 0, size * 1.2, 0, Math.PI * 2);
  ctx.stroke();

  ctx.restore();
}

function draw(): void {
  try {
    ctx.save();
    ctx.clearRect(0, 0, W, H);

    const invX0 = zoomTransform.invertX(0);
    const invX1 = zoomTransform.invertX(W);
    const invY0 = zoomTransform.invertY(0);
    const invY1 = zoomTransform.invertY(H);
    const viewport = {
      xMin: Math.min(invX0, invX1),
      xMax: Math.max(invX0, invX1),
      yMin: -Math.max(invY0, invY1),
      yMax: -Math.min(invY0, invY1),
    };

    if (mapLayer) mapLayer.draw(ctx, world2screen, viewport);
    if (showGlobalCostmap && globalCostmapLayer) globalCostmapLayer.draw(ctx, world2screen, viewport);
    if (showLocalCostmap && localCostmapLayer) localCostmapLayer.draw(ctx, world2screen, viewport);

    drawGrid(1.0, "rgba(0, 0, 0, 0.08)");
    drawGrid(0.25, "rgba(0, 0, 0, 0.035)");

    // Lidar scan (on top of map/grid)
    if (showLidar) drawLidar();
    if (showPlan) drawPlan();
    if (showGoal) drawGoal();

    // Which pose to render
    let renderPose: { x: number; y: number; yaw: number };
    if (isBagMode) {
      const rel = Number(slider!.value) || 0;
      renderPose = (scrubPose || getBagPoseAtRel(rel)) as { x: number; y: number; yaw: number };
    } else {
      renderPose = liveMode ? pose : (scrubPose || pose)!;
    }

    // Draw Path
    // Bag mode: reconstruct from poseHist up to current time
    // Live mode: use accumulated 'path' array
    if (isBagMode && poseHist.length && t0 != null) {
      const rel = Number(slider!.value) || 0;
      const absT = t0 + rel;

      ctx.beginPath();
      let started = false;
      let lastX = -99999, lastY = -99999;
      const minSq = 0.05 * 0.05; // 5cm threshold

      for (const p of poseHist) {
        if (p.t > absT) break;

        const dx = p.x - lastX;
        const dy = p.y - lastY;
        if (dx * dx + dy * dy < minSq) continue;

        const [sx, sy] = world2screen(p.x, p.y);
        if (!started) {
          ctx.moveTo(sx, sy);
          started = true;
        } else {
          ctx.lineTo(sx, sy);
        }
        lastX = p.x;
        lastY = p.y;
      }

      // Connect to current interpolated robot pose
      if (started) {
        const [sx, sy] = world2screen(renderPose.x, renderPose.y);
        ctx.lineTo(sx, sy);
      } else if (poseHist.length > 0 && poseHist[0].t <= absT) {
        // If we didn't start (maybe only 1 point or all filtered), just draw to robot
        const [sx, sy] = world2screen(poseHist[0].x, poseHist[0].y);
        ctx.moveTo(sx, sy);
        const [ex, ey] = world2screen(renderPose.x, renderPose.y);
        ctx.lineTo(ex, ey);
      }

      ctx.lineWidth = 1;
      ctx.strokeStyle = "#60a5fa"; // Blue-ish
      ctx.stroke();

    } else if (path.length > 1) {
      ctx.beginPath();
      for (let i = 0; i < path.length; i++) {
        const [x, y] = world2screen(path[i][0], path[i][1]);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.lineWidth = 1;
      ctx.strokeStyle = "#60a5fa";
      ctx.stroke();
    }
    drawRobot(renderPose.x, renderPose.y, renderPose.yaw, "#f59e0b");

    // Calculate dynamic halfway point (History + Plan)
    updateHalfwayPoint(renderPose);

    // Draw 50% marker if available
    if (isBagMode && halfwayPoint) {
      const [hx, hy] = world2screen(halfwayPoint.x, halfwayPoint.y);
      const size = Math.max(6, 8 * (zoomTransform.k / 40));

      ctx.save();
      // Color based on reached status
      const color = halfwayPoint.reached ? "#3b82f6" : "#9ca3af"; // Blue-500 vs Gray-400
      ctx.fillStyle = color;
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;

      // Draw pin/circle
      ctx.beginPath();
      ctx.arc(hx, hy, size, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Label
      ctx.fillStyle = "#ffffff";
      ctx.font = "bold 12px sans-serif";
      ctx.textAlign = "center";
      ctx.textBaseline = "bottom";
      ctx.fillText("Est. 50%", hx, hy - size - 2);

      ctx.restore();
    }

    // Loading shimmer while fetching map snapshot
    if (isMapLoading) {
      ctx.fillStyle = "rgba(96,165,250,0.08)";
      ctx.fillRect(0, 0, W, H);
    }

    ctx.restore();

    // Draw cmd_vel mini overlay canvas (separate canvas)
    drawCmdOverlayMini();

    requestAnimationFrame(draw);
  } catch (e) {
    console.error("Draw loop error:", e);
    setStatus("Draw error: " + String(e));
  }
}

requestAnimationFrame(draw);
