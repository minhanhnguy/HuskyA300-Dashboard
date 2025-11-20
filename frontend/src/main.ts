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
} from "./types";

/**
 * Husky Dashboard — main.ts
 * - Live mode: timeline grows; slider can follow live or scrub.
 * - Bag mode: timeline frozen to bag duration; play/pause + scrub.
 * - Map “time-travel”: in bag mode fetch exact map at slider time.
 * - During play, apply map *patches* between frames for progressive refining.
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

// ==============================
// Helpers
// ==============================
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
  } else {
    t0 = null;
    lastPoseCount = 0;
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

  constructor(meta: MapMeta) {
    this.w = meta.width;
    this.h = meta.height;
    this.res = meta.resolution;
    this.origin = meta.origin;
    this.tileSize = meta.tile_size || 256;

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

  private _colorOf(v: number): number {
    if (v === -1) return 160; // unknown
    if (v >= 65) return 0; // occupied
    return 255; // free
    // tweak later for prettier palette if needed
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
        const g = this._colorOf(v);
        const i = (dstRow + xx) * 4;
        img.data[i + 0] = g;
        img.data[i + 1] = g;
        img.data[i + 2] = g;
        img.data[i + 3] = 255;
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
    const xMin = Math.floor((viewport.xMin - this.origin.x) / this.res);
    const xMax = Math.ceil((viewport.xMax - this.origin.x) / this.res);
    const yMin = Math.floor((viewport.yMin - this.origin.y) / this.res);
    const yMax = Math.ceil((viewport.yMax - this.origin.y) / this.res);
    const c0 = Math.max(0, Math.floor(xMin / this.tileSize));
    const c1 = Math.min(this.cols - 1, Math.floor((xMax - 1) / this.tileSize));
    const r0 = Math.max(0, Math.floor(yMin / this.tileSize));
    const r1 = Math.min(this.rows - 1, Math.floor((yMax - 1) / this.tileSize));
    const now = performance.now();

    for (let r = r0; r <= r1; r++)
      for (let c = c0; c <= c1; c++) {
        const t = this.tiles[r][c];
        if (t.dirty) this._rebuildTile(r, c);
        const cellX = c * this.tileSize,
          cellY = r * this.tileSize;
        const wx0 = this.origin.x + cellX * this.res;
        const wy0 = this.origin.y + cellY * this.res;
        const wx1 = wx0 + t.tw * this.res;
        const wy1 = wy0 + t.th * this.res;
        const [X0, Y0] = world2screenFn(wx0, wy0);
        const [X1] = world2screenFn(wx1, wy0);
        const [, Y1] = world2screenFn(wx0, wy1);
        const sx = (X1 - X0) / t.tw,
          sy = (Y1 - Y0) / t.th;

        ctx2d.save();
        ctx2d.setTransform(1, 0, 0, 1, 0, 0);
        ctx2d.translate(X0, Y0);
        ctx2d.scale(Math.abs(sx), Math.abs(sy));
        const dx = sx < 0 ? -t.tw : 0,
          dy = sy < 0 ? -t.th : 0;
        ctx2d.drawImage(t.canvas, dx, dy);

        // flash overlay (120ms) for freshly updated tiles
        if (t.flashUntil > now) {
          const alpha = Math.min(0.15, ((t.flashUntil - now) / 120) * 0.15);
          ctx2d.globalAlpha = alpha;
          ctx2d.fillStyle = "#60a5fa";
          ctx2d.fillRect(dx, dy, t.tw, t.th);
          ctx2d.globalAlpha = 1;
        }

        ctx2d.restore();
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

// ==============================
// Lidar scan fetch (bag mode)
// ==============================
function requestScanAtAbs(absT: number): void {
  if (!isBagMode || t0 == null) return;
  if (scanReqInFlight) return;
  if (lastScanReqAbsT != null && Math.abs(absT - lastScanReqAbsT) < 0.03) return; // throttle

  scanReqInFlight = true;
  lastScanReqAbsT = absT;

  (async () => {
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
  })();
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

  setStatus(`Replaying bag: ${name}`);
  closeBagPanel();

  // Wait for pose_history to stabilize, then jump to end
  await waitForStablePoseHistory({ timeoutMs: 5000, settleMs: 600, pollMs: 150 });
  setTimelineFromPoseHist();
  setTimelineToEnd();

  // Fetch exact map at end and prime delta state
  const dur = getDuration();
  await fetchMapAtRel(dur);
  lastAbsTForDelta = (t0 ?? 0) + dur;
  if (t0 != null) {
    requestScanAtAbs(t0 + dur);
  }
}

// ==============================
// Play/Pause (bag) — progressive refine at ~25Hz
// ==============================
function startPlayback(): void {
  if (!isBagMode || isPlaying) return;
  isPlaying = true;
  lastTickTs = null;
  if (playBtn) playBtn.textContent = "Pause";

  // Prime lastAbsT for delta
  const relNow = Number(slider!.value) || 0;
  lastAbsTForDelta = (t0 ?? 0) + relNow;

  // Drive delta requests at ~UI rate (25Hz)
  if (!mapPlayTimer) {
    mapPlayTimer = window.setInterval(async () => {
      if (!isPlaying || !isBagMode || t0 == null) return;
      const rel = Number(slider!.value) || 0;
      const targetAbs = t0 + rel;
      if (targetAbs > (lastAbsTForDelta ?? targetAbs) + 1e-4) {
        await fetchMapDeltaBetween(lastAbsTForDelta!, targetAbs);
        lastAbsTForDelta = targetAbs;
      }
      requestScanAtAbs(targetAbs);
    }, 1000 / 25);
  }

  requestAnimationFrame(playTick);
}

function stopPlayback(): void {
  isPlaying = false;
  lastTickTs = null;
  if (playBtn) playBtn.textContent = "Play";
  if (mapPlayTimer) {
    clearInterval(mapPlayTimer);
    mapPlayTimer = null;
  }
}

function playTick(ts: number): void {
  if (!isPlaying) return;
  if (lastTickTs == null) {
    lastTickTs = ts;
    requestAnimationFrame(playTick);
    return;
  }
  const dt = (ts - lastTickTs) / 1000;
  lastTickTs = ts;

  const dur = getDuration();
  let rel = Number(slider!.value) || 0;
  rel += dt * playRate;

  if (rel >= dur) {
    rel = dur;
    updateTimeUI(rel);
    scrubTime = (t0 ?? 0) + rel;
    scrubPose = getBagPoseAtRel(rel);
    stopPlayback();
  } else {
    updateTimeUI(rel);
    scrubTime = (t0 ?? 0) + rel;
    scrubPose = getBagPoseAtRel(rel);
    requestAnimationFrame(playTick);
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
  }
});

slider!.addEventListener("change", async () => {
  if (!poseHist.length || t0 == null) return;
  if (!isBagMode) return; // live mode uses WS/REST live map
  const rel = Number(slider!.value);
  await fetchMapAtRel(rel);
  lastAbsTForDelta = t0 + rel; // resync delta base to playhead
  requestScanAtAbs(t0 + rel);
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
}, 1500);

// ==============================
// Rendering
// ==============================
function world2screen(x: number, y: number): [number, number] {
  const X = zoomTransform.applyX(x);
  const Y = zoomTransform.applyY(-y); // +y up
  return [X, Y];
}

function drawRobot(xm: number, ym: number, yaw: number, color = "#f59e0b"): void {
  const [x, y] = world2screen(xm, ym);
  const size = Math.max(8, 12 * (zoomTransform.k / 40) * 1.6);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(-yaw);
  ctx.beginPath();
  ctx.moveTo(size, 0);
  ctx.lineTo(-size * 0.6, size * 0.5);
  ctx.lineTo(-size * 0.6, -size * 0.5);
  ctx.closePath();
  ctx.fillStyle = color;
  ctx.fill();
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

function draw(): void {
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

  drawGrid(1.0, "rgba(0, 0, 0, 0.08)");
  drawGrid(0.25, "rgba(0, 0, 0, 0.035)");

  // Lidar scan (on top of map/grid)
  drawLidar();

  if (path.length > 1) {
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

  // Which pose to render
  let renderPose: { x: number; y: number; yaw: number };
  if (isBagMode) {
    const rel = Number(slider!.value) || 0;
    renderPose = (scrubPose || getBagPoseAtRel(rel)) as { x: number; y: number; yaw: number };
  } else {
    renderPose = liveMode ? pose : (scrubPose || pose)!;
  }
  drawRobot(renderPose.x, renderPose.y, renderPose.yaw, "#f59e0b");

  // Loading shimmer while fetching map snapshot
  if (isMapLoading) {
    ctx.fillStyle = "rgba(96,165,250,0.08)";
    ctx.fillRect(0, 0, W, H);
  }

  ctx.restore();
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);
