// frontend/src/canvas/MapTiles.ts
/**
 * Tile-based map rendering system.
 * Handles occupancy grids with tile caching for efficient rendering.
 */

import type { MapMeta } from '../types';

type TileCell = {
    canvas: HTMLCanvasElement;
    ctx: CanvasRenderingContext2D;
    dirty: boolean;
    tw: number;
    th: number;
    flashUntil: number; // ms timestamp (performance.now)
};

export type PaletteFn = (v: number) => [number, number, number, number];

export const mapPalette: PaletteFn = (v) => {
    if (v === -1) return [160, 160, 160, 255];
    if (v >= 65) return [0, 0, 0, 255];
    return [255, 255, 255, 255];
};

export const globalCostmapPalette: PaletteFn = (v) => {
    const alpha = 64; // 25% opacity
    if (v === 0) return [0, 0, 0, 0];
    if (v === -1) return [112, 137, 134, alpha];
    if (v === 99) return [0, 255, 255, alpha];   // Cyan (Inscribed)
    if (v === 100) return [255, 0, 255, alpha];  // Magenta (Lethal)
    if (v >= 1 && v <= 98) {
        const t = (v - 1) / 97.0;
        const r = Math.round(255 * t);
        const b = Math.round(255 * (1 - t));
        return [r, 0, b, alpha];
    }
    return [0, 0, 0, 0];
};

export const localCostmapPalette: PaletteFn = (v) => {
    const alpha = 255;
    if (v === 0) return [0, 0, 0, 0];
    if (v === -1) return [112, 137, 134, alpha];
    if (v === 99) return [0, 255, 255, alpha];
    if (v === 100) return [255, 0, 255, alpha];
    if (v >= 1 && v <= 98) {
        const t = (v - 1) / 97.0;
        const r = Math.round(255 * t);
        const b = Math.round(255 * (1 - t));
        return [r, 0, b, alpha];
    }
    return [0, 0, 0, 0];
};

export class MapTiles {
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
        for (let r = 0; r < this.rows; r++) {
            for (let c = 0; c < this.cols; c++) {
                this.tiles[r][c].dirty = true;
            }
        }
    }

    private _rebuildTile(r: number, c: number): void {
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
        transform: { k: number },
        world2screenFn: (x: number, y: number) => [number, number],
        viewport: { xMin: number; xMax: number; yMin: number; yMax: number }
    ): void {
        if (!this.data) return;

        // Calculate visible tile range
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
        const k = transform.k;
        const scale = k * this.res;

        // Draw visible tiles
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) {
                const t = this.tiles[r][c];
                if (t.dirty) this._rebuildTile(r, c);

                const lx = c * this.tileSize * this.res;
                const ly = r * this.tileSize * this.res;

                const wx = this.origin.x + lx * cosYaw - ly * sinYaw;
                const wy = this.origin.y + lx * sinYaw + ly * cosYaw;

                const [sx, sy] = world2screenFn(wx, wy);

                ctx2d.save();
                ctx2d.setTransform(1, 0, 0, 1, 0, 0);
                ctx2d.translate(sx, sy);
                ctx2d.rotate(-this.origin.yaw);
                ctx2d.scale(scale, scale);
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

    applyPatches(patches: Array<{ x: number; y: number; w: number; h: number; data: number[] }>): void {
        const now = performance.now();
        for (const up of patches) {
            const { x, y, w, h, data } = up;
            const c0 = Math.floor(x / this.tileSize);
            const c1 = Math.floor((x + w - 1) / this.tileSize);
            const r0 = Math.floor(y / this.tileSize);
            const r1 = Math.floor((y + h - 1) / this.tileSize);
            for (let yy = 0; yy < h; yy++) {
                const srcOff = yy * w;
                const dstOff = (y + yy) * this.w + x;
                this.data.set(Int8Array.from(data.slice(srcOff, srcOff + w)), dstOff);
            }
            for (let r = r0; r <= r1; r++) {
                for (let c = c0; c <= c1; c++) {
                    if (r >= 0 && r < this.rows && c >= 0 && c < this.cols) {
                        this.tiles[r][c].dirty = true;
                        this.tiles[r][c].flashUntil = now + 120;
                    }
                }
            }
        }
    }
}
