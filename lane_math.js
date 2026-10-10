/* Ackermann lane maths for the FMS lane maker.
 *
 * Model: the path is followed by the robot's REAR-AXLE CENTRE (standard bicycle / Ackermann model,
 * the point the minimum turning radius is usually quoted for). A lane from pose A (x, y, yaw) to a
 * point B is ONE circular arc that starts tangent to A's heading and passes through B (or a straight
 * line when B is dead ahead). For that arc:
 *     L     = |AB|                      chord length
 *     alpha = angle of B off A's heading (+ = left)
 *     R     = L / (2 sin|alpha|)        arc radius
 *     turn  = 2 * alpha                 heading change, so heading at B = yaw + 2 alpha
 * It is drivable when R >= R_design (= R_min * (1 + margin)). Equivalently, B must lie OUTSIDE the two
 * circles of radius R_design that touch A on its left and right side. At distance L the allowed angle
 * range is +- asin(min(1, L / (2 R_design))).
 *
 * Works in the browser (window.LaneMath) and in node (module.exports) so it can be unit-tested.
 */
(function (root) {
  'use strict';
  const DEG = 180 / Math.PI;
  const STRAIGHT_EPS = 0.5 / DEG;   // |alpha| below 0.5 deg = straight lane

  function wrap(a) { return Math.atan2(Math.sin(a), Math.cos(a)); }

  /** Robot geometry derived from the user's parameters (all metres). */
  function robotGeometry(p) {
    const rDesign = p.minTurnRadius * (1 + (p.marginPct || 0) / 100);
    const overhang = Math.max(0, (p.length - p.wheelbase) / 2);   // axles assumed centred in the body
    return {
      rMin: p.minTurnRadius,
      rDesign,
      wheelbase: p.wheelbase,
      track: p.track,
      length: p.length,
      width: p.width,
      overhang,                                                    // rear axle -> rear bumper = front axle -> front bumper
      maxSteerDeg: Math.atan(p.wheelbase / p.minTurnRadius) * DEG, // steering angle that gives R_min
    };
  }

  /** Evaluate a lane from pose -> target point. */
  function evalTarget(pose, target, geo) {
    const dx = target.x - pose.x, dy = target.y - pose.y;
    const L = Math.hypot(dx, dy);
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    const fwd = dx * c + dy * s;          // along the heading
    const left = -dx * s + dy * c;        // to the robot's left
    const alpha = Math.atan2(left, fwd);
    const maxAlpha = Math.asin(Math.min(1, L / (2 * geo.rDesign)));
    const out = { L, fwd, left, alpha, maxAlpha, straight: false, R: Infinity, turn: 0,
                  endYaw: pose.yaw, arcLen: L, steerDeg: 0, feasible: false, reason: '', center: null,
                  needAhead: null };
    if (L < 1e-6) { out.reason = 'same point'; return out; }
    if (Math.abs(alpha) >= Math.PI / 2) { out.reason = 'behind the robot (no reversing)'; return out; }
    if (Math.abs(alpha) < STRAIGHT_EPS) {
      out.straight = true;
      out.feasible = true;
      return out;
    }
    const sign = alpha > 0 ? 1 : -1;
    out.R = L / (2 * Math.sin(Math.abs(alpha)));
    out.turn = 2 * alpha;
    out.endYaw = wrap(pose.yaw + out.turn);
    out.arcLen = out.R * Math.abs(out.turn);
    out.steerDeg = Math.atan(geo.wheelbase / out.R) * DEG;
    out.center = { x: pose.x - sign * out.R * s, y: pose.y + sign * out.R * c };
    out.feasible = out.R >= geo.rDesign - 1e-9;
    if (!out.feasible) {
      out.reason = 'turn too tight';
      // Minimum distance ahead for this sideways offset to leave the turning circle.
      const d = Math.abs(left), r = geo.rDesign;
      out.needAhead = d <= 2 * r ? Math.sqrt(Math.max(0, 2 * r * d - d * d)) : 0;
    }
    return out;
  }

  /** Pose at arc length sDist along the lane described by ev (from evalTarget). */
  function poseAt(pose, ev, sDist) {
    if (ev.straight || !ev.center) {          // no arc (straight, or an invalid target): straight line
      return { x: pose.x + sDist * Math.cos(pose.yaw), y: pose.y + sDist * Math.sin(pose.yaw), yaw: pose.yaw };
    }
    const sign = ev.alpha > 0 ? 1 : -1;
    const yaw = pose.yaw + sign * sDist / ev.R;
    return { x: ev.center.x + sign * ev.R * Math.sin(yaw), y: ev.center.y - sign * ev.R * Math.cos(yaw), yaw: wrap(yaw) };
  }

  /** Dense samples along the lane (for drawing and collision checks). */
  function samplePath(pose, ev, step) {
    const n = Math.max(1, Math.ceil(ev.arcLen / step));
    const pts = [];
    for (let i = 0; i <= n; i++) pts.push(poseAt(pose, ev, ev.arcLen * i / n));
    return pts;
  }

  /** Micro-nodes from A (excluded) to B (included, last element). */
  function microNodes(pose, ev, spacing, maxStepDeg) {
    let n = Math.ceil(ev.arcLen / spacing - 1e-9);
    if (!ev.straight) n = Math.max(n, Math.ceil(Math.abs(ev.turn) * DEG / maxStepDeg - 1e-9));
    n = Math.max(1, n);
    const nodes = [];
    for (let i = 1; i <= n; i++) nodes.push(poseAt(pose, ev, ev.arcLen * i / n));
    return { nodes, spacing: ev.arcLen / n, stepDeg: Math.abs(ev.turn) * DEG / n };
  }

  /** Radii swept by the body and wheels on an arc of radius R (rear-axle reference). */
  function sweptRadii(R, geo) {
    if (!isFinite(R)) return null;
    const front = geo.wheelbase + geo.overhang;                  // rear axle -> front bumper
    return {
      bodyInner: R - geo.width / 2,                               // inner side of the body
      bodyOuter: Math.hypot(R + geo.width / 2, front),            // outer front corner
      wheelInnerRear: R - geo.track / 2,
      wheelOuterRear: R + geo.track / 2,
      wheelInnerFront: Math.hypot(R - geo.track / 2, geo.wheelbase),
      wheelOuterFront: Math.hypot(R + geo.track / 2, geo.wheelbase),
    };
  }

  /** Footprint corners (world) of the robot standing at pose (rear-axle reference). */
  function footprint(pose, geo) {
    const xs = [-geo.overhang, geo.wheelbase + geo.overhang], ys = [-geo.width / 2, geo.width / 2];
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    return [[xs[0], ys[0]], [xs[1], ys[0]], [xs[1], ys[1]], [xs[0], ys[1]]]
      .map(([lx, ly]) => ({ x: pose.x + lx * c - ly * s, y: pose.y + lx * s + ly * c }));
  }

  /** Points covering the footprint (for collision checks). */
  function footprintPoints(pose, geo, step) {
    const pts = [], c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    const x0 = -geo.overhang, x1 = geo.wheelbase + geo.overhang, y0 = -geo.width / 2, y1 = geo.width / 2;
    const nx = Math.max(1, Math.ceil((x1 - x0) / step)), ny = Math.max(1, Math.ceil((y1 - y0) / step));
    for (let i = 0; i <= nx; i++) for (let j = 0; j <= ny; j++) {
      const lx = x0 + (x1 - x0) * i / nx, ly = y0 + (y1 - y0) * j / ny;
      pts.push({ x: pose.x + lx * c - ly * s, y: pose.y + lx * s + ly * c });
    }
    return pts;
  }

  /* ---------- map file parsing (ROS map_server format) ---------- */

  /** Minimal parser for a map_server .yaml: flat key: value; origin as [x, y, yaw] or as a block list. */
  function parseMapYaml(text) {
    const out = {};
    let listKey = null;                       // key whose value is a "- item" block list
    for (const raw of text.split(/\r?\n/)) {
      const line = raw.replace(/#.*$/, '').trim();
      if (!line) continue;
      const item = line.match(/^-\s*(.*)$/);
      if (item && listKey) { out[listKey].push(Number(item[1])); continue; }
      listKey = null;
      const m = line.match(/^([A-Za-z_]+)\s*:\s*(.*)$/);
      if (!m) continue;
      let v = m[2].trim().replace(/^['"]|['"]$/g, '');
      if (v === '') { out[m[1]] = []; listKey = m[1]; continue; }
      if (m[1] === 'origin') v = v.replace(/[\[\]]/g, '').split(',').map(Number);
      else if (!isNaN(Number(v))) v = Number(v);
      out[m[1]] = v;
    }
    return out;
  }

  /** Parse a binary (P5) or ASCII (P2) PGM. Returns {width, height, maxval, data: Uint8Array (0..255)}. */
  function parsePGM(bytes) {
    let pos = 0;
    const tokens = [];
    function nextToken() {
      for (;;) {
        while (pos < bytes.length && /\s/.test(String.fromCharCode(bytes[pos]))) pos++;
        if (bytes[pos] === 35) { while (pos < bytes.length && bytes[pos] !== 10) pos++; continue; } // '#'
        break;
      }
      let t = '';
      while (pos < bytes.length && !/\s/.test(String.fromCharCode(bytes[pos]))) t += String.fromCharCode(bytes[pos++]);
      return t;
    }
    const magic = nextToken();
    if (magic !== 'P5' && magic !== 'P2') throw new Error('not a PGM file (magic ' + magic + ')');
    const width = parseInt(nextToken(), 10), height = parseInt(nextToken(), 10), maxval = parseInt(nextToken(), 10);
    const n = width * height, data = new Uint8Array(n);
    if (magic === 'P5') {
      pos++;                                   // single whitespace after maxval
      const two = maxval > 255;
      for (let i = 0; i < n; i++) {
        const v = two ? (bytes[pos + 2 * i] << 8) | bytes[pos + 2 * i + 1] : bytes[pos + i];
        data[i] = Math.round(v * 255 / maxval);
      }
    } else {
      for (let i = 0; i < n; i++) data[i] = Math.round(parseInt(nextToken(), 10) * 255 / maxval);
    }
    tokens.length = 0;
    return { width, height, maxval, data };
  }

  /** Occupancy class per pixel as map_server does it: 0 free, 1 unknown, 2 occupied. */
  function occupancyFromGray(gray, meta) {
    const occ = new Uint8Array(gray.length);
    const occT = meta.occupied_thresh ?? 0.65, freeT = meta.free_thresh ?? 0.196, negate = !!meta.negate;
    for (let i = 0; i < gray.length; i++) {
      const p = negate ? gray[i] / 255 : (255 - gray[i]) / 255;
      occ[i] = p > occT ? 2 : p < freeT ? 0 : 1;
    }
    return occ;
  }

  /* ---------- RMF nav graph (0.yaml) ---------- */

  function parseScalar(s) {
    s = s.trim();
    if (s === 'true') return true;
    if (s === 'false') return false;
    if (/^'.*'$/.test(s)) return s.slice(1, -1).replace(/''/g, "'");
    if (/^".*"$/.test(s)) return s.slice(1, -1);
    if (s !== '' && !isNaN(Number(s))) return Number(s);
    return s;
  }
  function splitTop(s) {                      // split on commas outside quotes / brackets
    const out = []; let cur = '', q = null, depth = 0;
    for (const ch of s) {
      if (q) { cur += ch; if (ch === q) q = null; continue; }
      if (ch === "'" || ch === '"') { q = ch; cur += ch; continue; }
      if (ch === '{' || ch === '[') depth++;
      if (ch === '}' || ch === ']') depth--;
      if (ch === ',' && depth === 0) { out.push(cur); cur = ''; continue; }
      cur += ch;
    }
    if (cur.trim()) out.push(cur);
    return out;
  }
  function parseFlowMap(s) {
    const inner = s.trim().replace(/^\{/, '').replace(/\}$/, '');
    const m = {};
    for (const part of splitTop(inner)) {
      const i = part.indexOf(':'); if (i < 0) continue;
      m[part.slice(0, i).trim()] = parseScalar(part.slice(i + 1));
    }
    return m;
  }
  function parseItemValue(s) { s = s.trim(); return s.startsWith('{') ? parseFlowMap(s) : parseScalar(s); }

  /** Parse the first level of an RMF nav graph 0.yaml (block "- - x" or flow "- [x, y, {..}]" items). */
  function parseNavGraph(text) {
    const V = [], L = [];
    let level = null, levelsIndent = -1, levelIndent = -1, section = null, secIndent = -1, cur = null, curIndent = -1, done = false;
    const flush = () => { if (cur && section) (section === 'vertices' ? V : L).push(cur); cur = null; };
    for (const raw of text.split(/\r?\n/)) {
      if (done) break;
      const line = raw.replace(/\s+#\s.*$/, '').replace(/^#.*$/, '');
      if (!line.trim()) continue;
      const ind = line.search(/\S/), t = line.trim();
      const km = !t.startsWith('-') && t.match(/^([^:\s][^:]*):\s*(.*)$/);
      if (km) {
        flush();
        const key = km[1].trim();
        if (key === 'levels') { levelsIndent = ind; section = null; continue; }
        if (levelsIndent >= 0 && ind > levelsIndent && (level === null || ind === levelIndent) && km[2] === '') {
          if (level !== null) { done = true; break; }       // only the first level
          level = key; levelIndent = ind; section = null; continue;
        }
        if (key === 'vertices' || key === 'lanes') { section = key; secIndent = ind; continue; }
        if (section && ind <= secIndent) section = null;
        continue;
      }
      if (!section) continue;
      let m;
      // deeper-indented "- value" continues the current item (the value may itself be negative: "- -0.6")
      if (cur && ind > curIndent && (m = t.match(/^-\s+(.*)$/))) { cur.push(parseItemValue(m[1])); continue; }
      if ((m = t.match(/^-\s*\[(.*)\]$/))) { flush(); cur = splitTop(m[1]).map(parseItemValue); flush(); continue; }
      if ((m = t.match(/^-\s+-\s+(.*)$/))) { flush(); cur = [parseItemValue(m[1])]; curIndent = ind; continue; }
    }
    flush();
    return {
      level: level || 'L1',
      verts: V.map(v => ({ x: Number(v[0]), y: Number(v[1]), props: (v[2] && typeof v[2] === 'object') ? v[2] : {} })),
      lanes: L.map(l => [Number(l[0]), Number(l[1])]),
    };
  }
  function vname(v) { const n = v.props && v.props.name; return n && n !== '<Unnamed>' ? String(n) : ''; }

  /**
   * Convert a graph drawn for diff-drive robots into an Ackermann graph by rounding every corner.
   * For each way through a node (in-lane a->b, out-lane b->c, never a U-turn back to a):
   *   - nearly straight (<= straightDeg): the lanes simply join at b;
   *   - a turn: an arc tangent to both lanes, radius R between R_design and the preferred radius (as big as
   *     fits in half the shorter lane), starting t = R tan(turn/2) before b and ending t after b;
   *   - if even R_design does not fit inside the two lanes (or the turn > maxTurnDeg), the way is dropped
   *     and reported.
   * Lanes are cut at every tangent point, so a robot that entered a lane on an arc can only leave through
   * exits further along it: drivable combinations stay connected, impossible ones are simply not there.
   */
  function convertToAckermann(graph, geo, opts) {
    opts = opts || {};
    const Rd = geo.rDesign, Rpref = Math.max(Rd, opts.prefRadius || Rd);
    const straightRad = (opts.straightDeg ?? 8) / DEG, maxTurn = (opts.maxTurnDeg ?? 150) / DEG;
    const mergeTol = opts.mergeTol ?? 0.06;
    const V = graph.verts;
    const seen = new Set(), lanes = [];
    for (const [a, b] of graph.lanes) {
      const k = a + '>' + b;
      if (a === b || seen.has(k) || !V[a] || !V[b]) continue;
      seen.add(k);
      const dx = V[b].x - V[a].x, dy = V[b].y - V[a].y;
      lanes.push({ a, b, len: Math.hypot(dx, dy), h: Math.atan2(dy, dx), ux: 0, uy: 0 });
    }
    lanes.forEach(l => { l.ux = Math.cos(l.h); l.uy = Math.sin(l.h); });
    const inL = V.map(() => []), outL = V.map(() => []);
    lanes.forEach((l, k) => { outL[l.a].push(k); inL[l.b].push(k); });

    // 1. classify every way through every node
    const straights = [], arcs = [], bad = [];
    const hasEntry = lanes.map(() => ({ straight: false, any: false }));
    const hasExit = lanes.map(() => ({ straight: false, any: false }));
    for (let b = 0; b < V.length; b++) {
      for (const k1 of inL[b]) for (const k2 of outL[b]) {
        const l1 = lanes[k1], l2 = lanes[k2];
        if (l2.b === l1.a) continue;                                   // U-turn: never offered
        const turn = wrap(l2.h - l1.h), at = Math.abs(turn);
        if (at <= straightRad) {
          straights.push({ k1, k2, b });
          hasExit[k1].straight = hasExit[k1].any = true; hasEntry[k2].straight = hasEntry[k2].any = true;
          continue;
        }
        const tanh = Math.tan(at / 2), room = Math.min(l1.len, l2.len);
        const need = Rd * tanh;
        if (at > maxTurn || need > room * 0.98) {
          bad.push({ a: l1.a, b, c: l2.b, turnDeg: at * DEG, need, room, reason: at > maxTurn ? 'turn too sharp' : 'not enough room' });
          continue;
        }
        const R = Math.max(Rd, Math.min(Rpref, (room / 2) / tanh));
        const t = R * tanh;
        arcs.push({ k1, k2, a: l1.a, b, c: l2.b, R, t, turn, d1: l1.len - t, d2: t });
        hasExit[k1].any = true; hasEntry[k2].any = true;
      }
    }

    // 2. points along each lane (start/end only where something uses them), merged when close
    const lanePts = lanes.map(() => []);
    lanes.forEach((l, k) => {
      if (hasEntry[k].straight || !hasEntry[k].any) lanePts[k].push(0);
      if (hasExit[k].straight || !hasExit[k].any) lanePts[k].push(l.len);
    });
    arcs.forEach(r => { lanePts[r.k1].push(r.d1); lanePts[r.k2].push(r.d2); });
    const clusters = lanePts.map((ds, k) => {
      const s = [...ds].sort((p, q) => p - q), cl = [];
      for (const d of s) {
        const last = cl[cl.length - 1];
        if (last && d - last.max <= mergeTol) { last.ds.push(d); last.max = d; } else cl.push({ ds: [d], max: d });
      }
      return cl.map(c => {
        const L0 = lanes[k].len;
        const d = c.ds.includes(0) ? 0 : c.ds.includes(L0) ? L0 : c.ds.reduce((p, q) => p + q, 0) / c.ds.length;
        return { d, min: c.ds[0], max: c.max };
      });
    });
    const ptKey = (k, d) => {
      const cl = clusters[k]; let best = 0, bd = Infinity;
      cl.forEach((c, i) => { const e = d < c.min ? c.min - d : d > c.max ? d - c.max : 0; if (e < bd) { bd = e; best = i; } });
      return k + '#' + best;
    };

    // 3. union-find: lane ends that meet at a node
    const parent = new Map();
    const find = x => { if (!parent.has(x)) parent.set(x, x); let r = x; while (parent.get(r) !== r) r = parent.get(r); parent.set(x, r); return r; };
    const unite = (x, y) => { const a = find(x), b = find(y); if (a !== b) parent.set(a, b); };
    lanes.forEach((l, k) => clusters[k].forEach((_, i) => find(k + '#' + i)));
    for (const s of straights) unite(ptKey(s.k1, lanes[s.k1].len), ptKey(s.k2, 0));
    const hubs = new Map();                       // node -> key of its "hub" (lane ends with no way onwards)
    for (let b = 0; b < V.length; b++) {
      const loose = [];
      for (const k of inL[b]) if (!hasExit[k].any) loose.push(ptKey(k, lanes[k].len));
      for (const k of outL[b]) if (!hasEntry[k].any) loose.push(ptKey(k, 0));
      if (loose.length) { loose.forEach(x => unite(loose[0], x)); hubs.set(b, loose[0]); }
    }

    // 4. vertices
    const verts = [], vid = new Map();
    const vOf = key => {
      const r = find(key);
      if (!vid.has(r)) {
        const [k, ci] = key.split('#').map(Number), l = lanes[k], d = clusters[k][ci].d;
        vid.set(r, verts.length);
        verts.push({ x: V[l.a].x + l.ux * d, y: V[l.a].y + l.uy * d, yaw: l.h, props: { name: '' }, kind: 'lane', src: null });
      }
      return vid.get(r);
    };
    const outLanes = new Set(), addLane = (i, j) => { if (i !== j) outLanes.add(i + '>' + j); };
    lanes.forEach((l, k) => {
      const ids = clusters[k].map((_, i) => vOf(k + '#' + i));
      for (let i = 0; i + 1 < ids.length; i++) addLane(ids[i], ids[i + 1]);
    });
    for (const r of arcs) {
      const l1 = lanes[r.k1], l2 = lanes[r.k2];
      const pose = { x: V[r.b].x - l1.ux * r.t, y: V[r.b].y - l1.uy * r.t, yaw: l1.h };
      const end = { x: V[r.b].x + l2.ux * r.t, y: V[r.b].y + l2.uy * r.t };
      r.pose = pose; r.ev = evalTarget(pose, end, Object.assign({}, geo, { rDesign: Rd * 0.999 }));
      r.samples = samplePath(pose, r.ev, 0.05);
      let mic = microNodes(pose, r.ev, geo.spacing || 0.3, geo.maxStep || 15).nodes.slice(0, -1);
      if (!mic.length) mic = [poseAt(pose, r.ev, r.ev.arcLen / 2)];
      const chain = [vOf(ptKey(r.k1, r.d1))];
      r.microIdx = mic.map(p => { verts.push({ x: p.x, y: p.y, yaw: p.yaw, props: { name: '' }, kind: 'micro', src: null }); chain.push(verts.length - 1); return verts.length - 1; });
      chain.push(vOf(ptKey(r.k2, r.d2)));
      for (let i = 0; i + 1 < chain.length; i++) addLane(chain[i], chain[i + 1]);
    }

    // 5. names and flags of the original waypoints
    const lost = [], moved = [];
    for (let b = 0; b < V.length; b++) {
      const nm = vname(V[b]), flags = Object.assign({}, V[b].props);
      if (!nm && !flags.is_charger && !flags.is_parking_spot) continue;
      let idx = null;
      if (hubs.has(b)) idx = vOf(hubs.get(b));
      else {
        const s = straights.find(s => s.b === b);
        if (s) idx = vOf(ptKey(s.k1, lanes[s.k1].len));
        else { const r = arcs.find(r => r.b === b); if (r) idx = r.microIdx[Math.floor((r.microIdx.length - 1) / 2)]; }
      }
      if (idx === null) { lost.push(nm || '#' + b); continue; }
      const v = verts[idx];
      if (v.src !== null) { lost.push(nm || '#' + b); continue; }  // two waypoints on one vertex
      delete flags.is_bidirectional;
      v.props = Object.assign(flags, { name: nm });
      if (!nm) delete v.props.name;
      v.src = b; v.kind = flags.is_charger || flags.is_parking_spot ? 'dock' : 'named';
      const dist = Math.hypot(v.x - V[b].x, v.y - V[b].y);
      if (dist > 0.01) moved.push({ name: nm || '#' + b, dist });
    }

    // 6. dead ends (a lane that arrives where the only way on is back) and forward-only reachability
    const deadEnds = [];
    for (let b = 0; b < V.length; b++) {
      const stuck = inL[b].some(k => !hasExit[k].any) && outL[b].length > 0;
      if (stuck) deadEnds.push({ b, name: vname(V[b]) || '#' + b });
    }
    const lanesOut = [...outLanes].map(s => s.split('>').map(Number));
    const adj = verts.map(() => []); lanesOut.forEach(([i, j]) => adj[i].push(j));
    const named = verts.map((v, i) => i).filter(i => verts[i].src !== null && vname(verts[i]));
    const reach = verts.map((v, i) => i).filter(i => verts[i].kind === 'dock').map(d => {
      const seenV = new Set([d]), st = [d];
      while (st.length) { const u = st.pop(); for (const w of adj[u]) if (!seenV.has(w)) { seenV.add(w); st.push(w); } }
      return { dock: vname(verts[d]) || '#' + d, reached: named.filter(i => seenV.has(i)).length, total: named.length,
               missing: named.filter(i => !seenV.has(i)).map(i => vname(verts[i])) };
    });
    return { level: graph.level, verts, lanes: lanesOut, arcs, straights, bad, deadEnds, lost, moved, reach,
             stats: { inVerts: V.length, inLanes: lanes.length, ways: straights.length + arcs.length + bad.length } };
  }

  const api = { DEG, wrap, robotGeometry, evalTarget, poseAt, samplePath, microNodes, sweptRadii,
                footprint, footprintPoints, parseMapYaml, parsePGM, occupancyFromGray,
                parseNavGraph, convertToAckermann };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.LaneMath = api;
})(typeof window !== 'undefined' ? window : globalThis);
