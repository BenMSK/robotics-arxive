// Rasterize the app icon into apple-touch-icon.png (180), icon-192.png, icon-512.png. No dependencies.
// Edit ICON so it matches the <svg class="logo"> in index.html (64-unit space). The background fills the whole
// square: iOS and Android round the corners themselves.
// usage: node tools/makeicon.js <outdir>
const fs = require("fs"), path = require("path"), zlib = require("zlib");
const outDir = process.argv[2] || ".";

const ICON = {
  bg: ["#2456D6", "#2456D6"],            // flat brand blue, same as <svg> in index.html
  shapes: [                              // a white page with a planned path from an orange start to a blue goal
    { rect: [16, 11, 32, 42, 4], fill: "#FFFFFF" },
    { circle: [22.00, 44.00, 1.6], fill: "#2456D6" }, { circle: [33.00, 28.00, 1.6], fill: "#2456D6" }, { circle: [22.71, 43.47, 1.6], fill: "#2456D6" }, { circle: [33.84, 27.74, 1.6], fill: "#2456D6" }, { circle: [23.34, 42.88, 1.6], fill: "#2456D6" }, { circle: [34.61, 27.46, 1.6], fill: "#2456D6" }, { circle: [23.91, 42.25, 1.6], fill: "#2456D6" }, { circle: [35.31, 27.16, 1.6], fill: "#2456D6" }, { circle: [24.41, 41.56, 1.6], fill: "#2456D6" }, { circle: [35.96, 26.84, 1.6], fill: "#2456D6" }, { circle: [24.86, 40.85, 1.6], fill: "#2456D6" }, { circle: [36.54, 26.51, 1.6], fill: "#2456D6" }, { circle: [25.27, 40.09, 1.6], fill: "#2456D6" }, { circle: [37.08, 26.16, 1.6], fill: "#2456D6" }, { circle: [25.63, 39.32, 1.6], fill: "#2456D6" }, { circle: [37.56, 25.79, 1.6], fill: "#2456D6" }, { circle: [25.96, 38.52, 1.6], fill: "#2456D6" }, { circle: [38.00, 25.41, 1.6], fill: "#2456D6" }, { circle: [26.27, 37.71, 1.6], fill: "#2456D6" }, { circle: [38.40, 25.01, 1.6], fill: "#2456D6" }, { circle: [26.56, 36.89, 1.6], fill: "#2456D6" }, { circle: [38.76, 24.60, 1.6], fill: "#2456D6" }, { circle: [26.85, 36.07, 1.6], fill: "#2456D6" }, { circle: [39.08, 24.18, 1.6], fill: "#2456D6" }, { circle: [27.13, 35.25, 1.6], fill: "#2456D6" }, { circle: [39.38, 23.75, 1.6], fill: "#2456D6" }, { circle: [27.41, 34.44, 1.6], fill: "#2456D6" }, { circle: [39.64, 23.31, 1.6], fill: "#2456D6" }, { circle: [27.71, 33.65, 1.6], fill: "#2456D6" }, { circle: [39.89, 22.86, 1.6], fill: "#2456D6" }, { circle: [28.03, 32.89, 1.6], fill: "#2456D6" }, { circle: [40.12, 22.39, 1.6], fill: "#2456D6" }, { circle: [28.37, 32.15, 1.6], fill: "#2456D6" }, { circle: [40.33, 21.93, 1.6], fill: "#2456D6" }, { circle: [28.75, 31.44, 1.6], fill: "#2456D6" }, { circle: [40.54, 21.45, 1.6], fill: "#2456D6" }, { circle: [29.17, 30.78, 1.6], fill: "#2456D6" }, { circle: [40.73, 20.97, 1.6], fill: "#2456D6" }, { circle: [29.64, 30.17, 1.6], fill: "#2456D6" }, { circle: [40.93, 20.48, 1.6], fill: "#2456D6" }, { circle: [30.17, 29.60, 1.6], fill: "#2456D6" }, { circle: [41.13, 19.99, 1.6], fill: "#2456D6" }, { circle: [30.76, 29.10, 1.6], fill: "#2456D6" }, { circle: [41.33, 19.50, 1.6], fill: "#2456D6" }, { circle: [31.43, 28.66, 1.6], fill: "#2456D6" }, { circle: [41.54, 19.00, 1.6], fill: "#2456D6" }, { circle: [32.17, 28.29, 1.6], fill: "#2456D6" }, { circle: [41.76, 18.50, 1.6], fill: "#2456D6" }, { circle: [33.00, 28.00, 1.6], fill: "#2456D6" }, { circle: [42.00, 18.00, 1.6], fill: "#2456D6" },
    { circle: [22, 44, 3.4], fill: "#F2A65A" }, { circle: [42, 18, 3.4], fill: "#2456D6" },
  ],
};

const hex = h => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];
const lerp = (a, b, t) => a.map((v, i) => v + (b[i] - v) * t);
function inRect(x, y, [rx, ry, w, hh, r]) {
  if (x < rx || x > rx + w || y < ry || y > ry + hh) return false;
  const cx = Math.max(rx + r, Math.min(x, rx + w - r)), cy = Math.max(ry + r, Math.min(y, ry + hh - r));
  return Math.hypot(x - cx, y - cy) <= r;
}
function inPoly(x, y, pts) { let inside = false; for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) { const [xi, yi] = pts[i], [xj, yj] = pts[j]; if ((yi > y) !== (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) inside = !inside; } return inside; }
const inside = (x, y, s) =>
  s.rect ? inRect(x, y, s.rect) :
  s.circle ? Math.hypot(x - s.circle[0], y - s.circle[1]) <= s.circle[2] :
  s.ellipse ? ((x - s.ellipse[0]) / s.ellipse[2]) ** 2 + ((y - s.ellipse[1]) / s.ellipse[3]) ** 2 <= 1 :
  s.poly ? inPoly(x, y, s.poly) : false;

const BG1 = hex(ICON.bg[0]), BG2 = hex(ICON.bg[1]), FILLS = ICON.shapes.map(s => hex(s.fill));
function shade(x, y) {
  let c = lerp(BG1, BG2, Math.max(0, Math.min(1, (x + y) / 128)));
  ICON.shapes.forEach((s, i) => { if (inside(x, y, s)) c = FILLS[i]; });
  return c;
}
function render(W) {
  const SS = 4, rows = [];
  for (let py = 0; py < W; py++) {
    const row = Buffer.alloc(1 + W * 3); row[0] = 0;
    for (let px = 0; px < W; px++) {
      let r = 0, g = 0, b = 0;
      for (let sy = 0; sy < SS; sy++) for (let sx = 0; sx < SS; sx++) { const c = shade((px + (sx + 0.5) / SS) * 64 / W, (py + (sy + 0.5) / SS) * 64 / W); r += c[0]; g += c[1]; b += c[2]; }
      const n = SS * SS; row[1 + px * 3] = Math.round(r / n); row[2 + px * 3] = Math.round(g / n); row[3 + px * 3] = Math.round(b / n);
    }
    rows.push(row);
  }
  return png(W, W, Buffer.concat(rows));
}
const CRC = (() => { const t = new Uint32Array(256); for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1; t[n] = c >>> 0; } return t; })();
const crc32 = buf => { let c = 0xFFFFFFFF; for (const b of buf) c = CRC[(c ^ b) & 0xFF] ^ (c >>> 8); return (c ^ 0xFFFFFFFF) >>> 0; };
function chunk(type, data) { const len = Buffer.alloc(4); len.writeUInt32BE(data.length); const td = Buffer.concat([Buffer.from(type, "ascii"), data]); const crc = Buffer.alloc(4); crc.writeUInt32BE(crc32(td)); return Buffer.concat([len, td, crc]); }
function png(w, hh, raw) { const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(hh, 4); ihdr[8] = 8; ihdr[9] = 2; return Buffer.concat([Buffer.from([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]), chunk("IHDR", ihdr), chunk("IDAT", zlib.deflateSync(raw, { level: 9 })), chunk("IEND", Buffer.alloc(0))]); }

for (const [name, size] of [["apple-touch-icon.png", 180], ["icon-192.png", 192], ["icon-512.png", 512]]) {
  const file = path.join(outDir, name); fs.writeFileSync(file, render(size)); console.log(name, size + "px", fs.statSync(file).size, "bytes");
}
