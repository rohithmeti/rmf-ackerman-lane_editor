#!/usr/bin/env node
/* Headless AprilTag Graph Maker: same logic as the GUI (tag_graph_core.js), no browser.
 *   node auto_layout_cli.js --map testmap/warehouse_slam.yaml --registration registration.json --name warehouse --out warehouse.taggraph.json
 * options: --robot L W margin (0.60 0.515 0.15)  --spacing 2.0  --min-sep 1.0  --min-aisle 2.5  --max-edge 3.5  --free-gray 240
 *          --frame dx dy yawDeg  (instead of --registration)   --start-near-world X Y  (this tag becomes tag 1; default: nearest the world origin)
 *          --bounds x0 x1 y0 y1  (map frame; default: the map inset by 0.6 m)   --tag-size 0.20
 */
const fs = require('fs'), path = require('path');
const LM = require('./lane_math.js'), TG = require('./tag_graph_core.js');
const argv = process.argv.slice(2), opt = {};
for (let i = 0; i < argv.length; i++) if (argv[i].startsWith('--')) { const k = argv[i].slice(2), v = []; while (argv[i + 1] && !argv[i + 1].startsWith('--')) v.push(argv[++i]); opt[k] = v; }
const num = (k, d, j = 0) => opt[k] ? parseFloat(opt[k][j]) : d;
if (!opt.map) { console.error(fs.readFileSync(__filename, 'utf8').split('*/')[0].replace(/^\/\* ?|^ \* ?/gm, '')); process.exit(1); }

const meta = LM.parseMapYaml(fs.readFileSync(opt.map[0], 'utf8'));
const pgm = LM.parsePGM(new Uint8Array(fs.readFileSync(path.join(path.dirname(opt.map[0]), String(meta.image).split('/').pop()))));
const map = { w: pgm.width, h: pgm.height, res: meta.resolution, ox: meta.origin[0], oy: meta.origin[1], gray: pgm.data, meta };
let frame = { dx: 0, dy: 0, yawDeg: 0 }, frameSet = false;
if (opt.registration) { const r = JSON.parse(fs.readFileSync(opt.registration[0], 'utf8')); frame = { dx: r.dx, dy: r.dy, yawDeg: r.yaw_deg }; frameSet = true; }
if (opt.frame) { frame = { dx: num('frame', 0, 0), dy: num('frame', 0, 1), yawDeg: num('frame', 0, 2) }; frameSet = true; }
const rp = TG.robotParams({ length: num('robot', 0.6, 0), width: num('robot', 0.515, 1), margin: num('robot', 0.15, 2) });
const field = TG.buildField(map, num('free-gray', 240));
const inset = 0.6, b = opt.bounds ? { x0: num('bounds', 0, 0), x1: num('bounds', 0, 1), y0: num('bounds', 0, 2), y1: num('bounds', 0, 3) }
  : { x0: map.ox + inset, x1: map.ox + map.w * map.res - inset, y0: map.oy + inset, y1: map.oy + map.h * map.res - inset };
const L = TG.autoLines(field, b, rp, { spacing: num('spacing', 2.0), minSep: num('min-sep', 1.0), minAisle: num('min-aisle', 2.5) });
const g = TG.derive({ v: L.v, h: L.h }, field, rp, { maxEdge: num('max-edge', 3.5), keepLargestOnly: true });
// tag 1 = the tag nearest to the chosen world point (default: the simulator's origin)
const sw = opt['start-near-world'] ? { x: num('start-near-world', 0, 0), y: num('start-near-world', 0, 1) } : { x: 0, y: 0 };
let si = 0, best = Infinity; g.nodes.forEach((n, i) => { const w = TG.toWorld(frame, n.x, n.y), d = Math.hypot(w.x - sw.x, w.y - sw.y); if (d < best) { best = d; si = i; } });
TG.assignIds(g, { startId: 0, startNodeIndex: si, startTag: 1 });
const c = TG.check(g, 0.9);
const name = (opt.name && opt.name[0]) || 'layout', out = (opt.out && opt.out[0]) || `${name}.taggraph.json`;
fs.writeFileSync(out, JSON.stringify(TG.exportJson(g, { name, tagSize: num('tag-size', 0.2), frame: frameSet ? frame : null, mapName: path.basename(opt.map[0]), mapOrigin: [map.ox, map.oy],
  robot: { length: rp.length, width: rp.width, margin: rp.margin }, startTag: 1, date: new Date().toISOString() }), null, 1));
console.log(`${g.nodes.length} tags, ${g.edges.length} lanes (${L.info.aisleX}+${L.info.aisleY} aisle lines), diagonal ${c.diagonal}, crossings ${c.crossings}, tags on foreign lanes ${c.nodesOnLane}; dropped spots ${g.dropped.nodes.length}, narrow lanes ${g.dropped.edges.length}`);
const s = g.nodes[si], w = TG.toWorld(frame, s.x, s.y);
console.log(`tag 1 at map (${s.x.toFixed(2)}, ${s.y.toFixed(2)}) = simulator (${w.x.toFixed(2)}, ${w.y.toFixed(2)});  frame: dx ${frame.dx.toFixed(3)} dy ${frame.dy.toFixed(3)} yaw ${frame.yawDeg.toFixed(2)} deg${frameSet ? '' : '  (NOT set)'}\n-> ${out}`);
