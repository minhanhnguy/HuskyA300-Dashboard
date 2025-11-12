import * as d3 from 'd3';

/**
 * Husky Dashboard — main.js
 * - Live mode: timeline grows; slider can follow live or scrub.
 * - Bag mode: timeline frozen to bag duration; play/pause + scrub.
 * - Map “time-travel”: in bag mode fetch exact map at slider time.
 */

// ==============================
// DOM references
// ==============================
const canvas       = document.getElementById('view');
const ctx          = canvas.getContext('2d'); ctx.imageSmoothingEnabled = false;

const statusDiv    = document.getElementById('status');
const slider       = document.getElementById('timeSlider');
const timeLabel    = document.getElementById('timeLabel');
const liveBtn      = document.getElementById('liveBtn');         // hidden in bag mode
const liveModeBtn  = document.getElementById('liveModeBtn');     // "Go back to Live"
const playBtn      = document.getElementById('playBtn');         // Play/Pause for bag mode (optional; code tolerates missing)

// Bag selection panel
const bagPanelBtn    = document.getElementById('bagPanelBtn');
const bagPanel       = document.getElementById('bagPanel');
const bagPanelClose  = document.getElementById('bagPanelClose');
const bagPanelStatus = document.getElementById('bagPanelStatus');
const bagList        = document.getElementById('bagList');

// ==============================
// Canvas + zoom
// ==============================
let W = 0, H = 0;
function resize() {
  const dpr = window.devicePixelRatio || 1;
  W = canvas.clientWidth; H = canvas.clientHeight;
  canvas.width = Math.round(W * dpr);
  canvas.height = Math.round(H * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
window.addEventListener('resize', resize);
resize();

let zoomTransform = d3.zoomIdentity.translate(W / 2, H / 2).scale(40);
const zoom = d3.zoom().scaleExtent([2, 400]).on('zoom', ev => { zoomTransform = ev.transform; });
d3.select(canvas).call(zoom).call(zoom.transform, zoomTransform);

// ==============================
// Global state
// ==============================
let isBagMode = false;     // true => freeze live growth; use map_full_at
let liveMode  = true;      // auto-follow end in live mode (ignored in bag mode)

let isPlaying  = false;    // bag playback
let playRate   = 1.0;      // 1×
let lastTickTs = null;
let mapPlayTimer = null;   // coarse map refresh timer (bag playback)

let path = [];                       // [[x,y], ...]
let pose = { x: 0, y: 0, yaw: 0 };   // live pose

let poseHist = [];   // [{t,x,y,yaw}]
let t0 = null;       // first timestamp
let lastPoseCount = 0;

let scrubTime = null;  // absolute seconds
let scrubPose = null;  // interpolated pose at scrubTime

// Map tile layer
let mapLayer = null;
let currentMapVersion = -999;
let isMapLoading = false;  // draw shimmer overlay while fetching

// ==============================
// Helpers
// ==============================
const sleep = (ms) => new Promise(r => setTimeout(r, ms));
function setStatus(txt) { statusDiv.textContent = txt; }

function getDuration() {
  if (!poseHist.length || t0 == null) return 0;
  const tMax = poseHist[poseHist.length - 1].t;
  return Math.max(0, tMax - t0);
}

function updateTimeUI(rel) {
  const v = Number.isFinite(rel) ? rel : 0;
  slider.value = v.toFixed(2);
  timeLabel.textContent = `t = ${v.toFixed(2)} s`;
}

async function loadPoseHistoryOnce() {
  const j = await fetch('/api/v1/pose_history').then(r => r.json()).catch(() => ({}));
  poseHist = j.pose_history || [];
  if (poseHist.length) {
    t0 = poseHist[0].t;
    lastPoseCount = poseHist.length;
  } else {
    t0 = null;
    lastPoseCount = 0;
  }
}

function setTimelineFromPoseHist() {
  const dur = getDuration();
  slider.min = "0";
  slider.max = dur.toFixed(2);
  if (dur === 0) updateTimeUI(0);
}

function setTimelineToEnd() {
  const dur = getDuration();
  updateTimeUI(dur);
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
  const i = lo, a = poseHist[i - 1], b = poseHist[i];
  const u = Math.max(0, Math.min(1, (t - a.t) / Math.max(1e-6, (b.t - a.t))));
  const x = a.x + (b.x - a.x) * u;
  const y = a.y + (b.y - a.y) * u;
  let dyaw = b.yaw - a.yaw;
  if (dyaw >  Math.PI) dyaw -= 2*Math.PI;
  if (dyaw < -Math.PI) dyaw += 2*Math.PI;
  const yaw = a.yaw + dyaw * u;
  return { x, y, yaw };
}

function getBagPoseAtRel(rel) {
  if (!poseHist.length || t0 == null) return pose;
  return poseAt(t0 + rel);
}

// Wait until pose history “stabilizes” (no growth for ~settleMs)
async function waitForStablePoseHistory({ timeoutMs = 5000, settleMs = 600, pollMs = 150 } = {}) {
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
    } else if (count > 0 && (performance.now() - lastChange) >= settleMs) {
      break;
    }
    await sleep(pollMs);
  }
}

// ==============================
// Map tiles
// ==============================
class MapTiles {
  constructor(meta) {
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
        const cnv = document.createElement('canvas');
        cnv.width = tw; cnv.height = th;
        const tctx = cnv.getContext('2d');
        tctx.imageSmoothingEnabled = false;
        this.tiles[r][c] = { canvas: cnv, ctx: tctx, dirty: true, tw, th };
      }
    }
  }
  loadFull(int8Flat) {
    if (int8Flat.length !== this.w * this.h) return;
    this.data.set(int8Flat);
    for (let r = 0; r < this.rows; r++)
      for (let c = 0; c < this.cols; c++)
        this.tiles[r][c].dirty = true;
  }
  _colorOf(v) { if (v === -1) return 160; if (v >= 65) return 0; return 255; }
  _rebuildTile(r, c) {
    const t = this.tiles[r][c];
    const tw = t.tw, th = t.th;
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
  draw(ctx, world2screen, viewport) {
    if (!this.data) return;
    const xMin = Math.floor((viewport.xMin - this.origin.x) / this.res);
    const xMax = Math.ceil ((viewport.xMax - this.origin.x) / this.res);
    const yMin = Math.floor((viewport.yMin - this.origin.y) / this.res);
    const yMax = Math.ceil ((viewport.yMax - this.origin.y) / this.res);
    const c0 = Math.max(0, Math.floor(xMin / this.tileSize));
    const c1 = Math.min(this.cols - 1, Math.floor((xMax - 1) / this.tileSize));
    const r0 = Math.max(0, Math.floor(yMin / this.tileSize));
    const r1 = Math.min(this.rows - 1, Math.floor((yMax - 1) / this.tileSize));
    for (let r = r0; r <= r1; r++) for (let c = c0; c <= c1; c++) {
      const t = this.tiles[r][c];
      if (t.dirty) this._rebuildTile(r, c);
      const cellX = c * this.tileSize, cellY = r * this.tileSize;
      const wx0 = this.origin.x + (cellX * this.res);
      const wy0 = this.origin.y + (cellY * this.res);
      const wx1 = wx0 + t.tw * this.res;
      const wy1 = wy0 + t.th * this.res;
      const [X0, Y0] = world2screen(wx0, wy0);
      const [X1, _ ] = world2screen(wx1, wy0);
      const [__, Y1] = world2screen(wx0, wy1);
      const sx = (X1 - X0) / t.tw, sy = (Y1 - Y0) / t.th;
      ctx.save();
      ctx.setTransform(1, 0, 0, 1, 0, 0);
      ctx.translate(X0, Y0);
      ctx.scale(Math.abs(sx), Math.abs(sy));
      const dx = sx < 0 ? -t.tw : 0, dy = sy < 0 ? -t.th : 0;
      ctx.drawImage(t.canvas, dx, dy);
      ctx.restore();
    }
  }
}

// Live map WS (ignored while in bag mode)
function connectMapWS() {
  const isDev = location.port === '5173';
  const wsBase = isDev ? `ws://${location.hostname}:8000` : `ws://${location.host}`;
  const ws = new WebSocket(`${wsBase}/ws/map`);
  ws.onopen = () => setStatus('Map WS connected');
  ws.onclose = () => { setStatus('Map WS closed, retrying…'); setTimeout(connectMapWS, 1000); };
  ws.onerror = () => setStatus('Map WS error');
  ws.onmessage = (ev) => {
    if (isBagMode) return; // do not let WS override bag snapshots
    const msg = JSON.parse(ev.data);
    if (msg.type === 'map_meta') {
      window._latestMapMeta = msg.meta;
    } else if (msg.type === 'map_full') {
      const meta = window._latestMapMeta;
      if (!meta) return;
      mapLayer = new MapTiles(meta);
      currentMapVersion = meta.version ?? 0;
      mapLayer.loadFull(Int8Array.from(msg.data));
      setStatus(`Map v${meta.version} loaded`);
    } else if (msg.type === 'map_updates') {
      if (!mapLayer) return;
      // We keep bag-mode exactness separate; live mode just applies patches here
      for (const up of msg.updates) {
        const { x, y, w, h, data } = up;
        // A fast in-place patch: mark dirty tiles; rebuild on draw()
        const c0 = Math.floor(x / mapLayer.tileSize), c1 = Math.floor((x + w - 1) / mapLayer.tileSize);
        const r0 = Math.floor(y / mapLayer.tileSize), r1 = Math.floor((y + h - 1) / mapLayer.tileSize);
        for (let yy = 0; yy < h; yy++) {
          const srcOff = yy * w;
          const dstOff = (y + yy) * mapLayer.w + x;
          mapLayer.data.set(Int8Array.from(data.slice(srcOff, srcOff + w)), dstOff);
        }
        for (let r = r0; r <= r1; r++)
          for (let c = c0; c <= c1; c++)
            if (r >= 0 && r < mapLayer.rows && c >= 0 && c < mapLayer.cols)
              mapLayer.tiles[r][c].dirty = true;
      }
    }
  };
}
connectMapWS();

// ==============================
// Map fetch at time (bag mode)
// ==============================
async function fetchMapAtRel(rel) {
  if (!isBagMode || t0 == null) return;
  const absT = t0 + rel;
  isMapLoading = true;
  try {
    const r = await fetch(`/api/v1/map_full_at?t=${encodeURIComponent(absT.toFixed(3))}`);
    const j = await r.json();
    if (!j || !j.meta || !j.data) return;
    const meta = j.meta;
    const version = meta.version ?? 0;

    // meta/version change => rebuild tiles
    const needsRebuild = (
      !mapLayer ||
      version !== currentMapVersion ||
      mapLayer.w !== meta.width ||
      mapLayer.h !== meta.height ||
      mapLayer.res !== meta.resolution ||
      mapLayer.origin.x !== meta.origin.x ||
      mapLayer.origin.y !== meta.origin.y
    );
    if (needsRebuild) {
      mapLayer = new MapTiles(meta);
      currentMapVersion = version;
    }
    mapLayer.loadFull(Int8Array.from(j.data));
  } finally {
    isMapLoading = false;
  }
}

// ==============================
// Bag panel
// ==============================
async function loadBagListIntoPanel() {
  bagPanelStatus.textContent = 'Loading…';
  bagList.innerHTML = '';
  try {
    const j = await fetch('/api/v1/bag/list').then(r => r.json());
    const bags = j.bags || [];
    if (!bags.length) {
      bagPanelStatus.textContent = 'No bag files found.';
      return;
    }
    bagPanelStatus.textContent = '';
    bags.forEach(b => {
      const li = document.createElement('li');
      li.textContent = b.name;
      const size = document.createElement('small');
      size.textContent = `${(b.size / 1024).toFixed(1)} KB`;
      li.appendChild(size);
      li.onclick = () => startBagReplay(b.name);
      bagList.appendChild(li);
    });
  } catch {
    bagPanelStatus.textContent = 'Failed to load bag list.';
  }
}
function openBagPanel()  { bagPanel.classList.remove('hidden'); loadBagListIntoPanel(); }
function closeBagPanel() { bagPanel.classList.add('hidden'); }
bagPanelBtn.onclick = openBagPanel;
bagPanelClose.onclick = closeBagPanel;
bagPanel.addEventListener('click', (ev) => { if (ev.target === bagPanel) closeBagPanel(); });

// ==============================
// Start bag (jump to END immediately)
// ==============================
async function startBagReplay(name) {
  stopPlayback(); // clean playback state
  let resp;
  try {
    resp = await fetch('/api/v1/bag/play', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name })
    });
  } catch {
    setStatus('Bag start failed (network)');
    return;
  }
  const j = await resp.json();
  if (!j.ok) {
    setStatus(j.error || 'Replay failed');
    return;
  }

  isBagMode = true;
  liveMode  = false;

  // Hide the small "Live" button and show Play/Pause (if present)
  if (liveBtn) liveBtn.style.display = 'none';
  if (playBtn) playBtn.style.display = '';

  path = [];
  setStatus(`Replaying bag: ${name}`);
  closeBagPanel();

  // Wait for pose_history to stabilize, then jump to end
  await waitForStablePoseHistory({ timeoutMs: 5000, settleMs: 600, pollMs: 150 });
  setTimelineFromPoseHist();
  setTimelineToEnd();

  // Fetch exact map at end
  const dur = getDuration();
  await fetchMapAtRel(dur);
}

// ==============================
// Play/Pause (bag) — coarse map refresh (~5 Hz)
// ==============================
function startPlayback() {
  if (!isBagMode || isPlaying) return;
  isPlaying = true;
  lastTickTs = null;
  if (playBtn) playBtn.textContent = 'Pause';
  if (!mapPlayTimer) {
    mapPlayTimer = setInterval(async () => {
      const rel = Number(slider.value) || 0;
      await fetchMapAtRel(rel);
    }, 200);
  }
  requestAnimationFrame(playTick);
}

function stopPlayback() {
  isPlaying = false;
  lastTickTs = null;
  if (playBtn) playBtn.textContent = 'Play';
  if (mapPlayTimer) { clearInterval(mapPlayTimer); mapPlayTimer = null; }
}

function playTick(ts) {
  if (!isPlaying) return;
  if (lastTickTs == null) { lastTickTs = ts; requestAnimationFrame(playTick); return; }
  const dt = (ts - lastTickTs) / 1000;
  lastTickTs = ts;

  const dur = getDuration();
  let rel = Number(slider.value) || 0;
  rel += dt * playRate;

  if (rel >= dur) {
    rel = dur;
    updateTimeUI(rel);
    scrubTime = t0 + rel;
    scrubPose = getBagPoseAtRel(rel);
    stopPlayback();
  } else {
    updateTimeUI(rel);
    scrubTime = t0 + rel;
    scrubPose = getBagPoseAtRel(rel);
    requestAnimationFrame(playTick);
  }
}

if (playBtn) {
  playBtn.onclick = () => {
    if (!isBagMode) return;
    if (isPlaying) stopPlayback(); else startPlayback();
  };
}

// ==============================
// "Go back to Live"
// ==============================
liveModeBtn.onclick = async () => {
  stopPlayback();
  try { await fetch('/api/v1/reset', { method: 'POST' }); } catch {}
  isBagMode = false;
  liveMode  = true;

  if (liveBtn)  liveBtn.style.display  = '';
  if (playBtn)  playBtn.style.display  = 'none';

  await loadPoseHistoryOnce();
  setTimelineFromPoseHist();
  jumpToLive();
  setStatus('Back to live mode.');
};

// ==============================
// WS: path/pose (both modes)
// ==============================
function connectPathWS() {
  const isDev = location.port === '5173';
  const wsBase = isDev ? `ws://${location.hostname}:8000` : `ws://${location.host}`;
  const ws = new WebSocket(`${wsBase}/ws/stream`);
  ws.onopen  = () => setStatus('Connected (path)');
  ws.onclose = () => { setStatus('Disconnected (path). Reconnecting…'); setTimeout(connectPathWS, 1000); };
  ws.onerror = () => setStatus('WS error (path)');
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'snapshot') {
      path = msg.path || [];
      pose = msg.pose || pose;
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

// ==============================
// Live poller (disabled in bag mode)
// ==============================
setInterval(async () => {
  if (isBagMode) return;

  const j = await fetch('/api/v1/pose_history_meta').then(r => r.json()).catch(() => ({}));
  if ((j.count || 0) > lastPoseCount) {
    await loadPoseHistoryOnce();
    setTimelineFromPoseHist();
  }
  if (liveMode && poseHist.length) {
    const tMax = poseHist[poseHist.length - 1].t;
    const rel = (tMax - t0);
    updateTimeUI(rel);
  }
}, 1000);

// ==============================
// Slider interactions
//  - input: scrub pose immediately (map waits)
//  - change: lazy-load exact map at that time in bag mode
// ==============================
slider.addEventListener('input', () => {
  if (!poseHist.length || t0 == null) return;
  stopPlayback();            // scrubbing pauses playback
  liveMode = false;          // disable live auto-follow
  const rel = Number(slider.value);
  updateTimeUI(rel);
  scrubTime = t0 + rel;
  scrubPose = poseAt(scrubTime);
});

slider.addEventListener('change', async () => {
  if (!poseHist.length || t0 == null) return;
  if (!isBagMode) return; // live mode uses WS/REST live map
  const rel = Number(slider.value);
  await fetchMapAtRel(rel);
});

// Jump-to-live (ignored in bag mode)
if (liveBtn) {
  liveBtn.onclick = () => {
    if (isBagMode) return;
    liveMode = true;
    jumpToLive();
  };
}
function jumpToLive() {
  if (!poseHist.length || t0 == null) return;
  const tMax = poseHist[poseHist.length - 1].t;
  const rel = (tMax - t0);
  updateTimeUI(rel);
  scrubTime = null;
  scrubPose = null;
}

// ==============================
// Optional: initial REST map fallback on app load
// ==============================
setTimeout(async () => {
  try {
    const meta = await fetch('/api/v1/map_meta').then(r => r.json());
    if (!meta || !meta.width) return;
    const full = await fetch('/api/v1/map_full').then(r => r.json());
    if (!full || !full.data || !full.data.length) return;
    mapLayer = new MapTiles(meta);
    currentMapVersion = meta.version ?? 0;
    mapLayer.loadFull(Int8Array.from(full.data));
    setStatus(`(Fallback) Map v${meta.version} loaded`);
  } catch { /* ignore */ }
}, 1500);

// ==============================
// Rendering
// ==============================
function world2screen(x, y) {
  const X = zoomTransform.applyX(x);
  const Y = zoomTransform.applyY(-y); // +y up
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
  ctx.lineTo(-size * 0.6,  size * 0.5);
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
    ctx.beginPath(); ctx.moveTo(X1, Y1); ctx.lineTo(X2, Y2); ctx.stroke();
  }
  for (let y = y0; y <= yMax; y += stepMeters) {
    const [X1, Y1] = world2screen(xMin, y);
    const [X2, Y2] = world2screen(xMax, y);
    ctx.beginPath(); ctx.moveTo(X1, Y1); ctx.lineTo(X2, Y2); ctx.stroke();
  }
  ctx.restore();
}

function draw() {
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

  drawGrid(1.0,  '#142235');
  drawGrid(0.25, '#0e1826');

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

  // Which pose to render
  let renderPose;
  if (isBagMode) {
    const rel = Number(slider.value) || 0;
    renderPose = scrubPose || getBagPoseAtRel(rel);
  } else {
    renderPose = (liveMode ? pose : (scrubPose || pose));
  }
  drawRobot(renderPose.x, renderPose.y, renderPose.yaw, '#f59e0b');

  // Loading shimmer while fetching map snapshot
  if (isMapLoading) {
    ctx.fillStyle = 'rgba(96,165,250,0.08)';
    ctx.fillRect(0, 0, W, H);
  }

  ctx.restore();
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);
