import * as d3 from 'd3';

// =====================================================
// Canvas setup
// =====================================================
const canvas = document.getElementById('view');
const ctx = canvas.getContext('2d');
ctx.imageSmoothingEnabled = false; // crisper tiles

let W = 0, H = 0;
function resize() {
  const dpr = window.devicePixelRatio || 1;
  W = canvas.clientWidth;
  H = canvas.clientHeight;
  canvas.width = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resize);
resize();

// =====================================================
// Zoom (world transform)
// =====================================================
let zoomTransform = d3.zoomIdentity.translate(W / 2, H / 2).scale(40);
const zoom = d3.zoom()
  .scaleExtent([2, 400])
  .on('zoom', (ev) => { zoomTransform = ev.transform; });

d3.select(canvas)
  .call(zoom)
  .call(zoom.transform, zoomTransform);

// =====================================================
// UI elements & helpers
// =====================================================
const statusDiv = document.getElementById('status');
function setStatus(txt) { statusDiv.textContent = txt; }

const slider = document.getElementById('timeSlider');
const timeLabel = document.getElementById('timeLabel');
const liveBtn = document.getElementById('liveBtn');

document.getElementById('resetBtn').onclick = async () => {
  await fetch('/api/v1/reset', { method: 'POST' });
  path = [];
  setStatus('Reset path.');
};
document.getElementById('reverseBtn').onclick = async () => {
  try {
    const r = await fetch('/api/v1/reverse_replay', { method: 'POST' });
    const j = await r.json();
    setStatus(j.ok ? 'Reverse replay started' : 'Reverse replay failed');
  } catch {
    setStatus('Reverse replay error');
  }
};

// =====================================================
// Live path/pose (from backend /ws/stream)
// =====================================================
let path = [];                         // [[x,y], ...]
let pose = { x: 0, y: 0, yaw: 0 };    // live pose

function connectPathWS() {
  const isDev = location.port === '5173';
  const wsBase = isDev
    ? `ws://${location.hostname}:8000`
    : `ws://${location.host}`;

  const ws = new WebSocket(`${wsBase}/ws/stream`);
  ws.onopen = () => setStatus('Connected (path)');
  ws.onclose = () => { setStatus('Disconnected (path). Reconnecting…'); setTimeout(connectPathWS, 1000); };
  ws.onerror = () => setStatus('WS error (path)');

  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'snapshot') {
      path = msg.path || [];
      pose = msg.pose || pose;
      setStatus(`Snapshot: ${path.length} pts | v=${(msg.v||0).toFixed(2)} m/s, w=${(msg.w||0).toFixed(2)} rad/s`);
    } else if (msg.type === 'append') {
      if (Array.isArray(msg.append) && msg.append.length > 0) {
        path.push(...msg.append);
        const max = 10000;
        if (path.length > max) path = path.slice(path.length - max);
      }
      if (msg.pose) pose = msg.pose;
    }
  };
}
connectPathWS();

// =====================================================
// Pose history for time slider
// =====================================================
let poseHist = [];    // [{t,x,y,yaw}, ...] sorted by t
let t0 = null;
let lastPoseCount = 0;

let scrubTime = null;   // absolute time (sec)
let scrubPose = null;   // {x,y,yaw} at scrubTime
let liveMode  = true;

async function loadPoseHistory() {
  const r = await fetch('/api/v1/pose_history');
  const j = await r.json();
  poseHist = j.pose_history || [];
  if (poseHist.length) {
    t0 = poseHist[0].t;
    lastPoseCount = poseHist.length;
    updateSliderBounds();
    if (liveMode) jumpToLive();
  }
}
loadPoseHistory();

setInterval(async () => {
  const r = await fetch('/api/v1/pose_history_meta');
  const j = await r.json();
  if ((j.count || 0) > lastPoseCount) {
    await loadPoseHistory();
  }
  if (liveMode && poseHist.length) {
    const tMax = poseHist[poseHist.length - 1].t;
    slider.value = (tMax - t0).toFixed(2);
    timeLabel.textContent = `t = ${parseFloat(slider.value).toFixed(2)} s`;
  }
}, 1000);

slider.addEventListener('input', () => {
  if (!poseHist.length) return;
  liveMode = false;
  const rel = parseFloat(slider.value);
  const targetT = t0 + rel;
  scrubTime = targetT;
  scrubPose = poseAt(targetT);
  timeLabel.textContent = `t = ${rel.toFixed(2)} s`;
});
liveBtn.onclick = () => { liveMode = true; jumpToLive(); };

function updateSliderBounds() {
  if (!poseHist.length) {
    slider.min = "0"; slider.max = "0"; slider.value = "0";
    timeLabel.textContent = "t = 0.00 s";
    return;
  }
  const tMin = poseHist[0].t, tMax = poseHist[poseHist.length - 1].t;
  slider.min = "0";
  slider.max = (tMax - tMin).toFixed(2);
  if (liveMode) slider.value = slider.max;
  timeLabel.textContent = `t = ${parseFloat(slider.value).toFixed(2)} s`;
}

function jumpToLive() {
  if (!poseHist.length) return;
  const tMax = poseHist[poseHist.length - 1].t;
  slider.value = (tMax - t0).toFixed(2);
  timeLabel.textContent = `t = ${parseFloat(slider.value).toFixed(2)} s`;
  scrubTime = null;
  scrubPose = null;
}

function poseAt(t) {
  if (!poseHist.length) return null;
  if (t <= poseHist[0].t) return poseHist[0];
  if (t >= poseHist[poseHist.length - 1].t) return poseHist[poseHist.length - 1];

  let lo = 0, hi = poseHist.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (poseHist[mid].t < t) lo = mid + 1; else hi = mid;
  }
  const i = lo;
  const a = poseHist[i - 1], b = poseHist[i];
  const u = Math.max(0, Math.min(1, (t - a.t) / Math.max(1e-6, (b.t - a.t))));
  const x = a.x + (b.x - a.x) * u;
  const y = a.y + (b.y - a.y) * u;
  let dyaw = b.yaw - a.yaw;
  if (dyaw >  Math.PI) dyaw -= 2*Math.PI;
  if (dyaw < -Math.PI) dyaw += 2*Math.PI;
  const yaw = a.yaw + dyaw * u;
  return { x, y, yaw };
}

// =====================================================
// Map tiled layer (from /ws/map + REST fallback)
// =====================================================
class MapTiles {
  constructor(meta) {
    this.w = meta.width;           // cells
    this.h = meta.height;
    this.res = meta.resolution;    // meters per cell
    this.origin = meta.origin;     // {x,y,yaw}
    this.tileSize = meta.tile_size || 256;

    this.cols = Math.ceil(this.w / this.tileSize);
    this.rows = Math.ceil(this.h / this.tileSize);

    // Keep buffer in ROS bottom-up order (row-major, y increases upward).
    this.data = new Int8Array(this.w * this.h); // -1..100
    this.tiles = [];

    for (let r = 0; r < this.rows; r++) {
      this.tiles[r] = [];
      for (let c = 0; c < this.cols; c++) {
        const tw = Math.min(this.tileSize, this.w - c * this.tileSize);
        const th = Math.min(this.tileSize, this.h - r * this.tileSize);
        const cnv = document.createElement('canvas');
        cnv.width = tw; cnv.height = th;
        const tctx = cnv.getContext('2d', { willReadFrequently: false });
        tctx.imageSmoothingEnabled = false; // crisp tiles
        this.tiles[r][c] = { canvas: cnv, ctx: tctx, dirty: true, tw, th };
      }
    }
  }

  loadFull(int8Flat) {
    if (int8Flat.length !== this.w * this.h) return;
    this.data.set(int8Flat); // exact ROS row-major order
    for (let r = 0; r < this.rows; r++)
      for (let c = 0; c < this.cols; c++)
        this.tiles[r][c].dirty = true;
  }

  applyPatch({ x, y, w, h, data }) {
    // x,y are ROS indices from the bottom-left; our buffer uses the same convention.
    for (let yy = 0; yy < h; yy++) {
      const srcOff = yy * w;
      const dstOff = (y + yy) * this.w + x;
      this.data.set(data.slice(srcOff, srcOff + w), dstOff);
    }
    const c0 = Math.floor(x / this.tileSize), c1 = Math.floor((x + w - 1) / this.tileSize);
    const r0 = Math.floor(y / this.tileSize), r1 = Math.floor((y + h - 1) / this.tileSize);
    for (let r = r0; r <= r1; r++)
      for (let c = c0; c <= c1; c++)
        if (r >= 0 && r < this.rows && c >= 0 && c < this.cols)
          this.tiles[r][c].dirty = true;
  }

  _colorOf(v) {
    if (v === -1) return 160; // unknown -> gray
    if (v >= 65) return 0;    // occupied -> black
    return 255;               // free -> white
  }

  _rebuildTile(r, c) {
    const t = this.tiles[r][c];
    const tw = t.tw, th = t.th;
    const img = t.ctx.createImageData(tw, th);

    const x0 = c * this.tileSize;
    const y0 = r * this.tileSize;

    // Convert ROS bottom-up rows to Canvas top-down rows (single vertical flip here).
    for (let yy = 0; yy < th; yy++) {
      const srcRow = (y0 + yy) * this.w + x0;        // ROS row
      const dstRow = (th - 1 - yy) * tw;             // Canvas row
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

  draw(ctx, world2screen, viewport) {
    if (!this.data) return;

    // Visible cell bounds (origin is the corner of cell (0,0))
    const xMin = Math.floor((viewport.xMin - this.origin.x) / this.res);
    const xMax = Math.ceil ((viewport.xMax - this.origin.x) / this.res);
    const yMin = Math.floor((viewport.yMin - this.origin.y) / this.res);
    const yMax = Math.ceil ((viewport.yMax - this.origin.y) / this.res);

    const c0 = Math.max(0, Math.floor(xMin / this.tileSize));
    const c1 = Math.min(this.cols - 1, Math.floor((xMax - 1) / this.tileSize));
    const r0 = Math.max(0, Math.floor(yMin / this.tileSize));
    const r1 = Math.min(this.rows - 1, Math.floor((yMax - 1) / this.tileSize));

    for (let r = r0; r <= r1; r++) {
      for (let c = c0; c <= c1; c++) {
        const t = this.tiles[r][c];
        if (t.dirty) this._rebuildTile(r, c);

        const cellX = c * this.tileSize;
        const cellY = r * this.tileSize;

        // World rectangle of this tile.
        const wx0 = this.origin.x + (cellX * this.res);
        const wy0 = this.origin.y + (cellY * this.res);
        const wx1 = wx0 + t.tw * this.res;
        const wy1 = wy0 + t.th * this.res;

        const [X0, Y0] = world2screen(wx0, wy0);
        const [X1, _]  = world2screen(wx1, wy0);
        const [__, Y1] = world2screen(wx0, wy1);

        // Use positive scales; if sign is negative, offset by tile size in pixels.
        const sx = (X1 - X0) / t.tw;
        const sy = (Y1 - Y0) / t.th;

        ctx.save();
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.translate(X0, Y0);
        ctx.scale(Math.abs(sx), Math.abs(sy));
        const dx = sx < 0 ? -t.tw : 0;
        const dy = sy < 0 ? -t.th : 0;
        ctx.drawImage(t.canvas, dx, dy);
        ctx.restore();
      }
    }
  }
}

let mapLayer = null;

// Robust map WS (dev/prod) + initial REST fallback
function connectMapWS() {
  const isDev = location.port === '5173';
  const wsBase = isDev
    ? `ws://${location.hostname}:8000`
    : `ws://${location.host}`;

  const ws = new WebSocket(`${wsBase}/ws/map`);
  ws.onopen = () => setStatus('Map WS connected');
  ws.onclose = () => { setStatus('Map WS closed, retrying…'); setTimeout(connectMapWS, 1000); };
  ws.onerror = () => setStatus('Map WS error');

  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'map_meta') {
      window._latestMapMeta = msg.meta;
    } else if (msg.type === 'map_full') {
      const meta = window._latestMapMeta;
      if (!meta) return;
      mapLayer = new MapTiles(meta);
      mapLayer.loadFull(Int8Array.from(msg.data));
      setStatus(`Map v${meta.version} loaded: ${meta.width}x${meta.height} @ ${meta.resolution}m`);
    } else if (msg.type === 'map_updates') {
      if (!mapLayer) return;
      for (const up of msg.updates) mapLayer.applyPatch(up);
    }
  };
}
connectMapWS();

// One-time REST fallback in case WS connects before first /map arrives
async function ensureInitialMapOnce() {
  try {
    const meta = await fetch('/api/v1/map_meta').then(r => r.json());
    if (!meta || !meta.width) return;
    const full = await fetch('/api/v1/map_full').then(r => r.json());
    if (!full || !full.data || !full.data.length) return;

    mapLayer = new MapTiles(meta);
    mapLayer.loadFull(Int8Array.from(full.data));
    setStatus(`(Fallback) Map v${meta.version} loaded: ${meta.width}x${meta.height} @ ${meta.resolution}m`);
  } catch { /* ignore */ }
}
setTimeout(ensureInitialMapOnce, 1500);

// =====================================================
// Rendering helpers
// =====================================================
function world2screen(x, y) {
  const X = zoomTransform.applyX(x);
  const Y = zoomTransform.applyY(-y); // invert Y so +y is up
  return [X, Y];
}

function drawRobot(xm, ym, yaw, color = '#f59e0b') {
  const [x, y] = world2screen(xm, ym);
  const size = Math.max(8, 12 * (zoomTransform.k / 40));
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

function drawGrid(stepMeters, color) {
  const k = zoomTransform.k;
  const stepPx = stepMeters * k;
  if (stepPx < 15) return;

  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = 1;

  const invX0 = zoomTransform.invertX(0);
  const invX1 = zoomTransform.invertX(W);
  const xMin = Math.min(invX0, invX1);
  const xMax = Math.max(invX0, invX1);
  const invY0 = zoomTransform.invertY(0);
  const invY1 = zoomTransform.invertY(H);
  const yMin = -Math.max(invY0, invY1);
  const yMax = -Math.min(invY0, invY1);

  const x0 = Math.floor(xMin / stepMeters) * stepMeters;
  const y0 = Math.floor(yMin / stepMeters) * stepMeters;

  for (let x = x0; x <= xMax; x += stepMeters) {
    const [X1, Y1] = world2screen(x, yMin);
    const [X2, Y2] = world2screen(x, yMax);
    ctx.beginPath(); ctx.moveTo(X1, Y1); ctx.lineTo(X2, Y2); ctx.stroke();
  }
  for (let y = y0; y <= yMax; y += stepMeters) {
    const [X1, Y1] = world2screen(xMin, y);
    const [X2, Y2] = world2screen(xMax, y);
    ctx.beginPath(); ctx.moveTo(X1, Y1); ctx.lineTo(X2, Y2); ctx.stroke();
  }
  ctx.restore();
}

// =====================================================
// Draw loop: MAP -> grid -> path -> robots
// =====================================================
function draw() {
  ctx.save();
  ctx.clearRect(0, 0, W, H);

  // world viewport (for tile culling)
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

  // 1) Map
  if (mapLayer) mapLayer.draw(ctx, world2screen, viewport);

  // 2) Grid overlays
  drawGrid(1.0,  '#142235');
  drawGrid(0.25, '#0e1826');

  // 3) Path
  if (path.length > 1) {
    ctx.beginPath();
    for (let i = 0; i < path.length; i++) {
      const [x, y] = world2screen(path[i][0], path[i][1]);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.lineWidth = 2;
    ctx.strokeStyle = '#60a5fa';
    ctx.stroke();
  }

  // 4) Live robot
  drawRobot(pose.x, pose.y, pose.yaw, '#f59e0b');

  // 5) Scrub marker
  if (!liveMode && scrubPose) drawRobot(scrubPose.x, scrubPose.y, scrubPose.yaw, '#34d399');

  ctx.restore();
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);
