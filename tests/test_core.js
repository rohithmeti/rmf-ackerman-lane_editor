// node tests for tag_graph_core.js :  node tests/test_core.js
const fs = require('fs'), path = require('path'), assert = require('assert');
const TG = require('../tag_graph_core.js'), LM = require('../lane_math.js');
let n = 0; const ok = (c, m) => { assert(c, m); n++; console.log('  ok  ' + m); };

// 1. distance transform vs brute force
{ const w = 23, h = 17, blocked = new Uint8Array(w * h); let seed = 7; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < w * h; i++) blocked[i] = rnd() < 0.08 ? 1 : 0; blocked[5] = 1;
  const d = TG.edt(blocked, w, h); let worst = 0;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) { let b = Infinity;
    for (let yy = 0; yy < h; yy++) for (let xx = 0; xx < w; xx++) if (blocked[yy * w + xx]) b = Math.min(b, Math.hypot(x - xx, y - yy));
    worst = Math.max(worst, Math.abs(b - d[y * w + x])); }
  ok(worst < 1e-4, 'edt matches brute force (max diff ' + worst.toExponential(1) + ')'); }

// 2. synthetic room 12 x 8 m, res 0.05, walls, a 2 x 1 m block in the middle
const mk = (W, H, res, ox, oy, fn) => { const gray = new Uint8Array(W * H).fill(254);
  for (let r = 0; r < H; r++) for (let c = 0; c < W; c++) { const x = ox + (c + .5) * res, y = oy + (H - 1 - r + .5) * res; if (fn(x, y)) gray[r * W + c] = 0; }
  return { w: W, h: H, res, ox, oy, gray }; };
const room = mk(240, 160, 0.05, 0, 0, (x, y) => x < 0.2 || x > 11.8 || y < 0.2 || y > 7.8 || (x > 5 && x < 7 && y > 3.5 && y < 4.5));
const rp = TG.robotParams({ length: 0.6, width: 0.515, margin: 0.15 });
const field = TG.buildField(room, 240);
{ const lines = { v: [], h: [] }; for (let x = 1; x <= 11; x += 2) lines.v.push({ x, y0: 0, y1: 8 }); for (let y = 1; y <= 7; y += 2) lines.h.push({ y, x0: 0, x1: 12 });
  const g = TG.derive(lines, field, rp, { maxEdge: 2 }); TG.assignIds(g, {}); const c = TG.check(g, 0.9);
  ok(g.nodes.length > 20 && g.edges.length > 30, 'derive: ' + g.nodes.length + ' nodes, ' + g.edges.length + ' lanes');
  ok(c.diagonal === 0, 'no diagonal lanes'); ok(c.crossings === 0, 'no crossing lanes'); ok(c.nodesOnLane === 0, 'no node lying on another lane');
  ok(!g.nodes.some(p => p.x > 4.4 && p.x < 7.6 && p.y > 3.0 && p.y < 5.0), 'no node inside / next to the block (spot-turn room)');
  const ids = g.nodes.map(p => p.id); ok(new Set(ids).size === ids.length, 'ids unique');
  // overlapping collinear lines merge
  const g2 = TG.derive({ v: [{ x: 3, y0: 1, y1: 5 }, { x: 3.005, y0: 4, y1: 7 }], h: [] }, field, rp, { maxEdge: 2 });
  ok(g2.edges.length === 3 && g2.nodes.length === 4, 'overlapping collinear lines merged into one lane chain (' + g2.nodes.length + ' nodes)');
  // frame
  const fr = { dx: 13.55, dy: -10, yawDeg: 0 }, w = TG.toWorld(fr, 1, 2), b = TG.toMap(fr, w.x, w.y); ok(Math.hypot(b.x - 1, b.y - 2) < 1e-9, 'toWorld/toMap round trip');
  const fr2 = { dx: 3, dy: -2, yawDeg: 37 }, q = TG.toWorld(fr2, 4, 1), r2 = TG.toMap(fr2, q.x, q.y); ok(Math.hypot(r2.x - 4, r2.y - 1) < 1e-9, 'rotated frame round trip');
  const sol = TG.solveFrame([{ map: { x: 0, y: 0 }, world: TG.toWorld(fr2, 0, 0) }, { map: { x: 5, y: 1 }, world: TG.toWorld(fr2, 5, 1) }]);
  ok(Math.abs(sol.dx - 3) < 1e-6 && Math.abs(sol.dy + 2) < 1e-6 && Math.abs(sol.yawDeg - 37) < 1e-6 && Math.abs(sol.scale - 1) < 1e-9, '2-point frame calibration recovers dx,dy,yaw');
  const y = TG.tagMapYaml(g, { frame: fr2, date: 'test' }); ok(/nodes:\n  \d+: \{x:/.test(y) && /edges:\n- \{from:/.test(y), 'yaml has the stack format');
  ok(TG.sdfBlock(g, fr2).includes('model://agv_tags/materials/textures/tag36_11_00000.png'), 'sdf block references the tag textures'); }

// 3. the warehouse map (world frame copy from the ros2_agv repo) - compare with the python generator (282 tags / 444 lanes)
const mp = path.join(__dirname, '..', 'testmap', 'warehouse_slam');
if (fs.existsSync(mp + '.pgm')) {
  const meta = LM.parseMapYaml(fs.readFileSync(mp + '.yaml', 'utf8')), pgm = LM.parsePGM(new Uint8Array(fs.readFileSync(mp + '.pgm')));
  const map = { w: pgm.width, h: pgm.height, res: meta.resolution, ox: meta.origin[0], oy: meta.origin[1], gray: pgm.data, meta };
  const f = TG.buildField(map, 240), lines = TG.autoLines(f, { x0: -14.3 - 13.55, x1: 14.3 - 13.55, y0: -24.3 + 10, y1: 24.3 + 10 }, rp, {});
  const g = TG.derive(lines, f, rp, { maxEdge: 2.0 }); TG.assignIds(g, {}); const c = TG.check(g, 0.9);
  console.log('  warehouse: ' + lines.v.length + ' vertical + ' + lines.h.length + ' horizontal lines (' + lines.info.aisleX + '+' + lines.info.aisleY + ' aisle lines) -> ' + g.nodes.length + ' tags, ' + g.edges.length + ' lanes; dropped nodes ' + g.dropped.nodes.length + ', isolated ' + g.isolated);
  ok(g.nodes.length > 150 && g.nodes.length < 500, 'warehouse tag count plausible (' + g.nodes.length + '; the python generator, which also knows the world file, gives 282)');
  ok(c.diagonal === 0 && c.crossings === 0 && c.nodesOnLane === 0, 'warehouse graph: 0 diagonal, 0 crossings, 0 nodes on foreign lanes');
}
console.log(n + ' checks passed');
