/* ============================================================================
 *  lidar_loc.js – Localisation (scan matching) sur grid map, carte vectorielle et scan
 *  Dépend de lidar_core.js (LidarCore). Navigateur : window.LidarLoc ; Node : require.
 *
 *  Conventions (identiques à lidar_core.js)
 *   - poses { x, y, th } : mètres, th en degrés (sens trigonométrique)
 *   - un « scan » brut : { a: [deg], d: [mm], i?: [] } ; points locaux : [{ x, y }] (m)
 *   - toutes les méthodes : f(référence, pointsLocaux, poseInitiale, options) -> { x, y, th, score, iters }
 * ========================================================================== */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory(require('./lidar_core.js'));
  else root.LidarLoc = factory(root.LidarCore);
})(typeof self !== 'undefined' ? self : this, function (LC) {
  'use strict';
  const D2R = Math.PI / 180, R2D = 180 / Math.PI;
  const normDeg = a => ((a + 180) % 360 + 360) % 360 - 180;
  const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());

  // ------------------------------------------------------------------ outils
  function mulberry32(seed) {               // générateur pseudo-aléatoire reproductible
    let a = seed >>> 0;
    return () => { a = (a + 0x6D2B79F5) >>> 0; let t = a; t = Math.imul(t ^ (t >>> 15), t | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61); return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
  }
  const gauss = rng => Math.sqrt(-2 * Math.log(rng() + 1e-12)) * Math.cos(2 * Math.PI * rng());

  // Scan brut -> points locaux (m), dans l'ordre angulaire
  function localPoints(scan, opt = {}) {
    return LC.polarToGlobal(scan, { x: 0, y: 0, th: 0 }, { dMin: opt.dMin ?? 0.05, dMax: opt.dMax ?? 12, iMin: opt.iMin ?? 0 });
  }
  // Sous-échantillonnage uniforme dans l'espace (1 point par case) et sans les points trop proches du lidar
  // (densité en 1/r : sinon les obstacles proches dominent le recalage)
  function uniformSubsample(pts, opt = {}) {
    const cell = opt.cell ?? 0.08, minR = opt.minR ?? 0.4, max = opt.max ?? 400;
    const seen = new Set(), out = [];
    for (const p of pts) {
      if (Math.hypot(p.x, p.y) < minR) continue;
      const k = Math.floor(p.x / cell) * 100003 + Math.floor(p.y / cell);
      if (!seen.has(k)) { seen.add(k); out.push(p); }
    }
    const src = out.length >= 20 ? out : pts;
    if (src.length <= max) return src.slice();
    const step = src.length / max, res = [];
    for (let k = 0; k < max; k++) res.push(src[Math.floor(k * step)]);
    return res;
  }
  function transform(pts, pose) {
    const c = Math.cos(pose.th * D2R), s = Math.sin(pose.th * D2R);
    return pts.map(p => ({ x: pose.x + c * p.x - s * p.y, y: pose.y + s * p.x + c * p.y }));
  }
  // pose de b exprimée dans le repère de a (a⁻¹ ∘ b) et composition a ∘ b
  function relPose(a, b) {
    const c = Math.cos(-a.th * D2R), s = Math.sin(-a.th * D2R), dx = b.x - a.x, dy = b.y - a.y;
    return { x: c * dx - s * dy, y: s * dx + c * dy, th: normDeg(b.th - a.th) };
  }
  function compose(a, b) {
    const c = Math.cos(a.th * D2R), s = Math.sin(a.th * D2R);
    return { x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, th: normDeg(a.th + b.th) };
  }
  function poseError(est, truth) {
    return { pos: Math.hypot(est.x - truth.x, est.y - truth.y), ang: Math.abs(normDeg(est.th - truth.th)) };
  }
  function solve3(A, b) {
    const M = A.map((r, i) => [...r, b[i]]);
    for (let i = 0; i < 3; i++) {
      let p = i; for (let r = i + 1; r < 3; r++) if (Math.abs(M[r][i]) > Math.abs(M[p][i])) p = r;
      if (Math.abs(M[p][i]) < 1e-12) return null;
      [M[i], M[p]] = [M[p], M[i]];
      for (let r = 0; r < 3; r++) if (r !== i) { const f = M[r][i] / M[i][i]; for (let c = i; c < 4; c++) M[r][c] -= f * M[i][c]; }
    }
    return [M[0][3] / M[0][0], M[1][3] / M[1][1], M[2][3] / M[2][2]];
  }

  // ==========================================================================
  //  Simulation (poses connues exactement)
  // ==========================================================================
  // Salle de classe en L inspirée de la salle réelle : 13,5 × 9,8 m, décroché, meuble central, pieds de chaises
  function classroomScene(seed = 1, opt = {}) {
    const rng = mulberry32(seed), S = LC.seg, walls = [];
    const box = (x0, y0, x1, y1) => walls.push(S(x0, y0, x1, y0), S(x1, y0, x1, y1), S(x1, y1, x0, y1), S(x0, y1, x0, y0));
    walls.push(S(0, 0, 13.5, 0), S(13.5, 0, 13.5, 7.8), S(13.5, 7.8, 5, 7.8), S(5, 7.8, 5, 9.8), S(5, 9.8, 0, 9.8), S(0, 9.8, 0, 0));
    walls.push(S(4.2, 7.8, 5, 7.8));                                    // retour de mur au décroché
    box(8.8, 2.4, 11.3, 4.0);                                           // meuble central
    box(0.1, 3.0, 0.6, 5.0);                                            // armoire
    const legs = [];
    const nLegs = opt.legs ?? 70;
    for (let k = 0; k < nLegs; k++) {                                  // pieds de chaises 4 × 4 cm
      const x = 1 + rng() * 11.5, y = 0.8 + rng() * 6.5;
      if (x > 8.5 && x < 11.6 && y > 2.1 && y < 4.3) continue;
      box(x, y, x + 0.04, y + 0.04); legs.push([x + 0.02, y + 0.02]);
    }
    const inside = (x, y) => x > 0.3 && y > 0.3 && y < 9.5 && x < 13.2 && !(x > 5 && y > 7.5);
    const free = (x, y, m = 0.35) => inside(x, y) && !(x > 8.8 - m && x < 11.3 + m && y > 2.4 - m && y < 4.0 + m) &&
      !(x < 0.6 + m && y > 3.0 - m && y < 5.0 + m) && legs.every(([lx, ly]) => Math.hypot(lx - x, ly - y) > m);
    return { walls, free, bounds: { x0: 0, y0: 0, x1: 13.5, y1: 9.8 } };
  }
  // Scan simulé du LD06 : un point tous les `step` degrés, bruit de distance gaussien, bruit angulaire, pertes
  function simulateScan(walls, pose, opt = {}, rng = Math.random) {
    const step = opt.step ?? 0.8, tours = opt.tours ?? 1, sd = opt.noise ?? 0.015, sa = opt.angNoise ?? 0.15;
    const drop = opt.dropout ?? 0.08, maxR = opt.maxRange ?? 12;
    const A = [], Dm = [], I = [];
    for (let t = 0; t < tours; t++) for (let a = rng() * step; a < 360; a += step) {
      if (rng() < drop) continue;
      const ang = (pose.th + a) * D2R, dx = Math.cos(ang), dy = Math.sin(ang);
      let best = Infinity;
      for (const w of walls) {
        const ex = w.x2 - w.x1, ey = w.y2 - w.y1, den = dx * ey - dy * ex;
        if (Math.abs(den) < 1e-12) continue;
        const qx = w.x1 - pose.x, qy = w.y1 - pose.y;
        const tt = (qx * ey - qy * ex) / den, u = (qx * dy - qy * dx) / den;
        if (tt > 0.02 && u >= 0 && u <= 1 && tt < best) best = tt;
      }
      if (best > maxR) continue;
      A.push(+((a + gauss(rng) * sa + 360) % 360).toFixed(2)); Dm.push(Math.round((best + gauss(rng) * sd) * 1000)); I.push(200);
    }
    return { a: A, d: Dm, i: I };
  }

  // ==========================================================================
  //  1. Grid map : champ de distance et méthodes
  // ==========================================================================
  // Transformée de distance euclidienne exacte (Felzenszwalb & Huttenlocher) aux cellules occupées
  function edt1d(f, n, d, v, z) {
    let k = 0; v[0] = 0; z[0] = -Infinity; z[1] = Infinity;
    for (let q = 1; q < n; q++) {
      let s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
      while (s <= z[k]) { k--; s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]); }
      k++; v[k] = q; z[k] = s; z[k + 1] = Infinity;
    }
    k = 0;
    for (let q = 0; q < n; q++) { while (z[k + 1] < q) k++; d[q] = (q - v[k]) * (q - v[k]) + f[v[k]]; }
  }
  function distanceField(grid, opt = {}) {
    const occT = opt.occT ?? LC.OCC_T, W = grid.W, H = grid.H, INF = 1e20;
    const g = new Float64Array(W * H);
    for (let k = 0; k < W * H; k++) g[k] = grid.data[k] >= occT ? 0 : INF;
    const n = Math.max(W, H), f = new Float64Array(n), d = new Float64Array(n), v = new Int32Array(n), z = new Float64Array(n + 1);
    for (let i = 0; i < W; i++) {                                     // colonnes
      for (let j = 0; j < H; j++) f[j] = g[j * W + i];
      edt1d(f, H, d, v, z); for (let j = 0; j < H; j++) g[j * W + i] = d[j];
    }
    for (let j = 0; j < H; j++) {                                     // lignes
      for (let i = 0; i < W; i++) f[i] = g[j * W + i];
      edt1d(f, W, d, v, z); for (let i = 0; i < W; i++) g[j * W + i] = d[i];
    }
    const dist = new Float32Array(W * H);
    for (let k = 0; k < W * H; k++) dist[k] = Math.sqrt(g[k]) * grid.res;
    return { res: grid.res, x0: grid.x0, y0: grid.y0, W, H, dist, outside: opt.outside ?? 1.0, tables: new Map() };
  }
  // distance interpolée (bilinéaire) et son gradient, en un point du monde
  function fieldSample(F, x, y) {
    const gx = (x - F.x0) / F.res - 0.5, gy = (y - F.y0) / F.res - 0.5;
    const i = Math.floor(gx), j = Math.floor(gy), fx = gx - i, fy = gy - j;
    if (i < 0 || j < 0 || i + 1 >= F.W || j + 1 >= F.H) return [F.outside, 0, 0];
    const k = j * F.W + i, d00 = F.dist[k], d10 = F.dist[k + 1], d01 = F.dist[k + F.W], d11 = F.dist[k + F.W + 1];
    const d = d00 * (1 - fx) * (1 - fy) + d10 * fx * (1 - fy) + d01 * (1 - fx) * fy + d11 * fx * fy;
    const ddx = ((d10 - d00) * (1 - fy) + (d11 - d01) * fy) / F.res, ddy = ((d01 - d00) * (1 - fx) + (d11 - d10) * fx) / F.res;
    return [d, ddx, ddy];
  }
  // table de vraisemblance exp(-d²/2σ²) (mise en cache par σ) pour les recherches rapides
  function likTable(F, sigma) {
    const key = Math.round(sigma * 1e4);
    if (F.tables.has(key)) return F.tables.get(key);
    const L = new Float32Array(F.dist.length), k2 = 1 / (2 * sigma * sigma);
    for (let k = 0; k < L.length; k++) L[k] = Math.exp(-F.dist[k] * F.dist[k] * k2);
    F.tables.set(key, L); return L;
  }
  // Recherche sur une fenêtre (x, y, θ) : pour chaque angle, les points tournés sont précalculés en cellules
  function windowSearch(F, L, pts, cx, cy, cth, o, keep = 1) {
    const best = [];
    const nx = Math.round(o.rxy / o.sxy), nth = Math.round(o.rth / o.sth);
    const PX = new Float64Array(pts.length), PY = new Float64Array(pts.length);
    for (let a = -nth; a <= nth; a++) {
      const th = cth + a * o.sth, c = Math.cos(th * D2R), s = Math.sin(th * D2R);
      for (let k = 0; k < pts.length; k++) { PX[k] = c * pts[k].x - s * pts[k].y; PY[k] = s * pts[k].x + c * pts[k].y; }
      for (let iy = -nx; iy <= nx; iy++) for (let ix = -nx; ix <= nx; ix++) {
        const tx = cx + ix * o.sxy, ty = cy + iy * o.sxy;
        let sum = 0;
        for (let k = 0; k < pts.length; k++) {
          const i = Math.floor((tx + PX[k] - F.x0) / F.res), j = Math.floor((ty + PY[k] - F.y0) / F.res);
          if (i >= 0 && j >= 0 && i < F.W && j < F.H) sum += L[j * F.W + i];
        }
        if (best.length < keep || sum > best[best.length - 1].score) {
          best.push({ x: tx, y: ty, th: th, score: sum });
          best.sort((p, q) => q.score - p.score); if (best.length > keep) best.pop();
        }
      }
    }
    best.forEach(b => b.score /= pts.length);
    return best;
  }
  // 1a. Recherche exhaustive (brute force) : fenêtre ±rxy, ±rth autour de la pose initiale, pas fixes
  // un point sur k pour limiter le coût des recherches exhaustives (les points sont déjà répartis uniformément)
  const thin = (pts, n) => pts.length <= n ? pts : pts.filter((_, k) => k % Math.ceil(pts.length / n) === 0);
  function gridBrute(F, pts, init, opt = {}) {
    const o = { rxy: 1.2, rth: 50, sxy: 0.05, sth: 1, sigma: 0.05, maxPts: 150, ...opt };
    pts = thin(pts, o.maxPts);
    const r = windowSearch(F, likTable(F, o.sigma), pts, init.x, init.y, init.th, o, 1)[0];
    return { x: r.x, y: r.y, th: normDeg(r.th), score: r.score, iters: 1 };
  }
  // 1b. Multi-résolution : grille grossière (20 cm / 4°, champ lissé), on garde les K meilleurs, puis on
  //     raffine chacun autour de lui avec des pas divisés par 2, jusqu'à 2,5 cm / 0,5°
  function gridMultiRes(F, pts, init, opt = {}) {
    const o = { rxy: 1.2, rth: 50, keep: 8, levels: [[0.2, 4], [0.1, 2], [0.05, 1], [0.025, 0.5]], ...opt };
    const [s0, a0] = o.levels[0];
    let cands = windowSearch(F, likTable(F, Math.max(0.05, s0)), pts, init.x, init.y, init.th,
                             { rxy: Math.ceil(o.rxy / s0) * s0, rth: Math.ceil(o.rth / a0) * a0, sxy: s0, sth: a0 }, o.keep * 3);
    cands = distinct(cands, s0 * 1.5, a0 * 1.5).slice(0, o.keep);
    for (let l = 1; l < o.levels.length; l++) {
      const [s, a] = o.levels[l], [sp, ap] = o.levels[l - 1], L = likTable(F, Math.max(0.05, s));
      let next = [];
      for (const c of cands) next.push(...windowSearch(F, L, pts, c.x, c.y, c.th, { rxy: sp, rth: ap, sxy: s, sth: a }, 2));
      cands = distinct(next.sort((p, q) => q.score - p.score), s, a).slice(0, Math.max(2, Math.ceil(o.keep / (l + 1))));
    }
    const r = cands[0];
    return { x: r.x, y: r.y, th: normDeg(r.th), score: r.score, iters: o.levels.length };
  }
  function distinct(list, dxy, dth) {
    const out = [];
    for (const c of list) if (!out.some(o => Math.hypot(o.x - c.x, o.y - c.y) < dxy && Math.abs(normDeg(o.th - c.th)) < dth)) out.push(c);
    return out;
  }
  // 1c. Descente de gradient (Gauss-Newton / Levenberg-Marquardt) sur le champ de distance :
  //     min Σ d(T(p))², avec le gradient du champ ; points trop loin (> dmax) ignorés (robustesse)
  function gridGradient(F, pts, init, opt = {}) {
    const o = { iters: 60, dmax0: 0.5, dmaxMin: 0.2, robust: 0.1, ...opt };   // robust : échelle des poids de Cauchy (m), 0 = sans
    let x = init.x, y = init.y, th = init.th * D2R, lambda = 1e-3, dmax = o.dmax0, it = 0, cost = Infinity;
    const evalCost = (X, Y, T) => {
      const c = Math.cos(T), s = Math.sin(T); let e = 0, n = 0;
      for (const p of pts) { const [d] = fieldSample(F, X + c * p.x - s * p.y, Y + s * p.x + c * p.y); const dd = Math.min(d, dmax); e += dd * dd; n++; }
      return e / n;
    };
    cost = evalCost(x, y, th);
    for (; it < o.iters; it++) {
      const c = Math.cos(th), s = Math.sin(th), A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
      for (const p of pts) {
        const rx = c * p.x - s * p.y, ry = s * p.x + c * p.y;
        const [d, gx, gy] = fieldSample(F, x + rx, y + ry);
        if (d > dmax) continue;
        const w = o.robust > 0 ? 1 / (1 + (d / o.robust) ** 2) : 1;     // les points loin de tout obstacle pèsent moins
        const J = [gx, gy, -gx * ry + gy * rx];
        for (let i = 0; i < 3; i++) { b[i] -= w * J[i] * d; for (let j = 0; j < 3; j++) A[i][j] += w * J[i] * J[j]; }
      }
      for (let i = 0; i < 3; i++) A[i][i] *= (1 + lambda), A[i][i] += 1e-9;
      const sol = solve3(A, b); if (!sol) break;
      const nc = evalCost(x + sol[0], y + sol[1], th + sol[2]);
      if (nc < cost) { x += sol[0]; y += sol[1]; th += sol[2]; cost = nc; lambda = Math.max(1e-6, lambda / 3); }
      else { lambda *= 5; if (lambda > 1e4) { if (dmax > o.dmaxMin) { dmax = Math.max(o.dmaxMin, dmax * 0.7); lambda = 1e-3; cost = evalCost(x, y, th); continue; } break; } continue; }
      if (Math.hypot(sol[0], sol[1]) < 1e-5 && Math.abs(sol[2]) < 1e-6) {
        if (dmax > o.dmaxMin) { dmax = Math.max(o.dmaxMin, dmax * 0.7); cost = evalCost(x, y, th); } else break;
      }
    }
    return { x, y, th: normDeg(th * R2D), score: -cost, iters: it };
  }

  // ==========================================================================
  //  2. Carte vectorielle
  // ==========================================================================
  // Index spatial des segments : chaque case (0,5 m) liste les segments qui passent à moins de `reach`
  function segIndex(segs, cell = 0.5, reach = 0.6) {
    const map = new Map(), key = (i, j) => i * 100003 + j;
    segs.forEach((s, k) => {
      const i0 = Math.floor((Math.min(s.x1, s.x2) - reach) / cell), i1 = Math.floor((Math.max(s.x1, s.x2) + reach) / cell);
      const j0 = Math.floor((Math.min(s.y1, s.y2) - reach) / cell), j1 = Math.floor((Math.max(s.y1, s.y2) + reach) / cell);
      for (let i = i0; i <= i1; i++) for (let j = j0; j <= j1; j++) {
        const cx = (i + 0.5) * cell, cy = (j + 0.5) * cell;
        if (LC.pointSegDist(cx, cy, s) > reach + cell) continue;
        const kk = key(i, j); (map.get(kk) || map.set(kk, []).get(kk)).push(k);
      }
    });
    // point le plus proche sur le segment le plus proche
    function query(x, y) {
      const l = map.get(key(Math.floor(x / cell), Math.floor(y / cell)));
      let bd = Infinity, bk = -1, bt = 0;
      if (l) for (const k of l) {
        const s = segs[k], dx = s.x2 - s.x1, dy = s.y2 - s.y1, L2 = dx * dx + dy * dy;
        let t = L2 > 0 ? ((x - s.x1) * dx + (y - s.y1) * dy) / L2 : 0; t = Math.max(0, Math.min(1, t));
        const d = Math.hypot(x - (s.x1 + t * dx), y - (s.y1 + t * dy));
        if (d < bd) { bd = d; bk = k; bt = t; }
      }
      return { d: bk < 0 ? reach : Math.min(bd, reach), k: bk, t: bt };
    }
    return { segs, query, reach };
  }
  function vecScore(I, pts, x, y, th, sigma) {
    const c = Math.cos(th * D2R), s = Math.sin(th * D2R), k2 = 1 / (2 * sigma * sigma); let sum = 0;
    for (const p of pts) { const q = I.query(x + c * p.x - s * p.y, y + s * p.x + c * p.y); sum += Math.exp(-q.d * q.d * k2); }
    return sum / pts.length;
  }
  // 2a. Recherche exhaustive : pas 10 cm / 2° sur la fenêtre, puis affinage exhaustif local 2,5 cm / 0,5°
  function vecBrute(I, pts, init, opt = {}) {
    const o = { rxy: 1.2, rth: 50, sxy: 0.1, sth: 2, fxy: 0.025, fth: 0.5, sigma: 0.08, maxPts: 150, ...opt };
    pts = thin(pts, o.maxPts);
    const scan = (cx, cy, cth, rxy, rth, sxy, sth, sig) => {
      let b = null; const nx = Math.round(rxy / sxy), na = Math.round(rth / sth);
      for (let a = -na; a <= na; a++) for (let iy = -nx; iy <= nx; iy++) for (let ix = -nx; ix <= nx; ix++) {
        const x = cx + ix * sxy, y = cy + iy * sxy, th = cth + a * sth, sc = vecScore(I, pts, x, y, th, sig);
        if (!b || sc > b.score) b = { x, y, th, score: sc };
      }
      return b;
    };
    let b = scan(init.x, init.y, init.th, o.rxy, o.rth, o.sxy, o.sth, Math.max(o.sigma, o.sxy));
    b = scan(b.x, b.y, b.th, o.sxy, o.sth, o.fxy, o.fth, o.sigma / 2);
    return { x: b.x, y: b.y, th: normDeg(b.th), score: b.score, iters: 2 };
  }
  // 2b. Descente de gradient : ICP point-à-segment (Gauss-Newton). Résidu = distance au point le plus proche
  //     du segment, projetée sur la normale du segment (à l'intérieur) ou vers l'extrémité.
  function vecICP(I, pts, init, opt = {}) {
    const o = { iters: 60, dmax0: 0.6, dmaxMin: 0.1, trim: 0.85, ...opt };
    let x = init.x, y = init.y, th = init.th * D2R, dmax = o.dmax0, it = 0, rms = 0;
    for (; it < o.iters; it++) {
      const c = Math.cos(th), s = Math.sin(th), R = [];
      for (const p of pts) {
        const rx = c * p.x - s * p.y, ry = s * p.x + c * p.y, wx = x + rx, wy = y + ry, q = I.query(wx, wy);
        if (q.k < 0 || q.d > dmax) continue;
        const g = I.segs[q.k], qx = g.x1 + q.t * (g.x2 - g.x1), qy = g.y1 + q.t * (g.y2 - g.y1);
        let nx, ny;
        if (q.t > 0 && q.t < 1) { const L = Math.hypot(g.x2 - g.x1, g.y2 - g.y1); nx = -(g.y2 - g.y1) / L; ny = (g.x2 - g.x1) / L; }
        else { const L = Math.hypot(wx - qx, wy - qy) || 1; nx = (wx - qx) / L; ny = (wy - qy) / L; }
        R.push([rx, ry, nx, ny, nx * (wx - qx) + ny * (wy - qy)]);
      }
      if (R.length < 10) break;
      R.sort((a, b) => Math.abs(a[4]) - Math.abs(b[4]));
      const Q = R.slice(0, Math.ceil(R.length * o.trim));
      const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
      for (const [rx, ry, nx, ny, e] of Q) {
        const J = [nx, ny, -nx * ry + ny * rx];
        for (let i = 0; i < 3; i++) { b[i] -= J[i] * e; for (let j = 0; j < 3; j++) A[i][j] += J[i] * J[j]; }
      }
      for (let i = 0; i < 3; i++) A[i][i] += 1e-6;
      const sol = solve3(A, b); if (!sol) break;
      x += sol[0]; y += sol[1]; th += sol[2];
      rms = Math.sqrt(Q.reduce((a, q) => a + q[4] * q[4], 0) / Q.length);
      if (Math.hypot(sol[0], sol[1]) < 1e-5 && Math.abs(sol[2]) < 1e-6) { if (dmax <= o.dmaxMin) break; }
      dmax = Math.max(o.dmaxMin, dmax * 0.8);
    }
    return { x, y, th: normDeg(th * R2D), score: -rms, iters: it };
  }
  // Caractéristiques « coins » d'une liste de segments : intersection de deux segments longs non parallèles,
  // proche d'une extrémité de chacun ; orientation = directions des deux segments vers leurs extrémités lointaines
  function lineFeatures(segs, opt = {}) {
    const minLen = opt.minLen ?? 0.6, minAng = opt.minAng ?? 40, near = opt.near ?? 0.6, maxN = opt.maxSegs ?? 40;
    const S = segs.filter(s => LC.segLen(s) >= minLen).sort((a, b) => LC.segLen(b) - LC.segLen(a)).slice(0, maxN), out = [];
    for (let i = 0; i < S.length; i++) for (let j = i + 1; j < S.length; j++) {
      const a = S[i], b = S[j]; if (LC.angleBetween(a, b).lines < minAng) continue;
      const r = LC.intersect(a, b); if (r.type !== 'point' && r.type !== 'lines') continue;
      const far = f => { const d1 = Math.hypot(f.x1 - r.x, f.y1 - r.y), d2 = Math.hypot(f.x2 - r.x, f.y2 - r.y); return Math.min(d1, d2) > near ? null : (d1 > d2 ? [f.x1, f.y1] : [f.x2, f.y2]); };
      const fa = far(a), fb = far(b); if (!fa || !fb) continue;
      const ua = Math.atan2(fa[1] - r.y, fa[0] - r.x), ub = Math.atan2(fb[1] - r.y, fb[0] - r.x);
      out.push({ x: r.x, y: r.y, ua, ub, w: Math.min(LC.segLen(a), LC.segLen(b)) });
    }
    return out;
  }
  // 2c. Calcul direct : coins du scan appariés aux coins de la carte -> pose en une formule
  //     (θ = écart des orientations, t = coin carte − R(θ)·coin scan), chaque hypothèse est notée, la
  //     meilleure est affinée par ICP. N'utilise PAS la pose initiale (localisation globale), sauf si
  //     opt.window est donné (hypothèses limitées à la fenêtre).
  function vecDirect(I, pts, init, opt = {}) {
    const o = { sigma: 0.08, refine: true, window: null, ...opt };
    // segments du scan, puis recollage des morceaux d'un même mur coupés par des ombres (fusion de la partie 4)
    const raw = LC.extractSegments(pts.slice().sort((p, q) => Math.atan2(p.y, p.x) - Math.atan2(q.y, q.x)), { x: 0, y: 0 }, { minLen: 0.4, minPts: 5 });
    const local = LC.mergeAll(raw, { angTol: 5, distTol: 0.10, gapTol: 2.5 }).segments;
    const fo = { near: o.featNear ?? 1.5, minLen: o.featMinLen ?? 0.5, minAng: o.featMinAng ?? 35 };
    const fs = lineFeatures(local, fo), key = JSON.stringify(fo);
    if (!I.features || I.featKey !== key){ I.features = lineFeatures(LC.mergeAll(I.segs, { angTol: 5, distTol: 0.10, gapTol: 2.5 }).segments, { ...fo, maxSegs: 60 }); I.featKey = key; }
    const fm = o.mapFeatures || I.features;
    const sub = pts.length > 150 ? pts.filter((_, k) => k % Math.ceil(pts.length / 150) === 0) : pts;
    let best = null, nh = 0;
    for (const a of fs) for (const m of fm) for (const [sa, sb, ma, mb] of [[a.ua, a.ub, m.ua, m.ub], [a.ua, a.ub, m.ub, m.ua]]) {
      const t1 = normDeg((ma - sa) * R2D), t2 = normDeg((mb - sb) * R2D);
      if (Math.abs(normDeg(t1 - t2)) > 8) continue;                    // les deux directions doivent concorder
      const th = normDeg(t1 + normDeg(t2 - t1) / 2), c = Math.cos(th * D2R), s = Math.sin(th * D2R);
      const x = m.x - (c * a.x - s * a.y), y = m.y - (s * a.x + c * a.y);
      if (o.window && (Math.hypot(x - init.x, y - init.y) > o.window.rxy || Math.abs(normDeg(th - init.th)) > o.window.rth)) continue;
      nh++;
      const sc = vecScore(I, sub, x, y, th, o.sigma);
      if (!best || sc > best.score) best = { x, y, th, score: sc };
    }
    if (!best) return { x: init.x, y: init.y, th: init.th, score: 0, iters: 0, failed: true, hypotheses: 0 };
    const r = o.refine ? vecICP(I, pts, best, { dmax0: 0.3 }) : best;
    return { x: r.x, y: r.y, th: normDeg(r.th), score: best.score, iters: nh, hypotheses: nh };
  }

  // ==========================================================================
  //  3. Entre scans (référence = un autre scan, pris près du scan à localiser)
  // ==========================================================================
  function pointIndex(P, cell = 0.3) {
    const map = new Map(), key = (i, j) => i * 100003 + j;
    P.forEach((p, k) => { const kk = key(Math.floor(p.x / cell), Math.floor(p.y / cell)); (map.get(kk) || map.set(kk, []).get(kk)).push(k); });
    return function nearest(x, y, maxD) {
      const ci = Math.floor(x / cell), cj = Math.floor(y / cell), r = Math.ceil(maxD / cell);
      let bd = maxD * maxD, bk = -1;
      for (let di = -r; di <= r; di++) for (let dj = -r; dj <= r; dj++) {
        const l = map.get(key(ci + di, cj + dj)); if (!l) continue;
        for (const k of l) { const d = (P[k].x - x) ** 2 + (P[k].y - y) ** 2; if (d < bd) { bd = d; bk = k; } }
      }
      return bk;
    };
  }
  // 3a. ICP point-à-point (solution fermée par décomposition en valeurs singulières, en 2D : atan2)
  function scanICPPoint(ref, pts, init, opt = {}) {
    const o = { iters: 80, dmax0: 1.0, dmaxMin: 0.1, trim: 0.8, ...opt };
    const nearest = ref.nearest || (ref.nearest = pointIndex(ref.pts));
    let x = init.x, y = init.y, th = init.th * D2R, dmax = o.dmax0, it = 0, rms = 0;
    for (; it < o.iters; it++) {
      const c = Math.cos(th), s = Math.sin(th), P = [];
      for (const p of pts) {
        const wx = x + c * p.x - s * p.y, wy = y + s * p.x + c * p.y, k = nearest(wx, wy, dmax);
        if (k >= 0) P.push([wx, wy, ref.pts[k].x, ref.pts[k].y, (ref.pts[k].x - wx) ** 2 + (ref.pts[k].y - wy) ** 2]);
      }
      if (P.length < 10) break;
      P.sort((a, b) => a[4] - b[4]);
      const Q = P.slice(0, Math.ceil(P.length * o.trim));
      let mpx = 0, mpy = 0, mqx = 0, mqy = 0;
      for (const q of Q) { mpx += q[0]; mpy += q[1]; mqx += q[2]; mqy += q[3]; }
      mpx /= Q.length; mpy /= Q.length; mqx /= Q.length; mqy /= Q.length;
      let sxx = 0, sxy = 0, syx = 0, syy = 0;
      for (const q of Q) { const ax = q[0] - mpx, ay = q[1] - mpy, bx = q[2] - mqx, by = q[3] - mqy; sxx += ax * bx; sxy += ax * by; syx += ay * bx; syy += ay * by; }
      const dth = Math.atan2(sxy - syx, sxx + syy), cd = Math.cos(dth), sd = Math.sin(dth);
      const tx = mqx - (cd * mpx - sd * mpy), ty = mqy - (sd * mpx + cd * mpy);
      const nx = cd * x - sd * y + tx, ny = sd * x + cd * y + ty, moved = Math.hypot(nx - x, ny - y);
      x = nx; y = ny; th += dth;
      rms = Math.sqrt(Q.reduce((a, q) => a + q[4], 0) / Q.length);
      if (moved < 1e-5 && Math.abs(dth) < 1e-6) { if (dmax <= o.dmaxMin) break; }
      dmax = Math.max(o.dmaxMin, dmax * 0.85);
    }
    return { x, y, th: normDeg(th * R2D), score: -rms, iters: it };
  }
  // normales du scan de référence (ACP des voisins à moins de 20 cm), pour l'ICP point-à-ligne
  function refNormals(ref) {
    if (ref.normals) return ref.normals;
    const P = ref.pts, nearIdx = new Map(), cell = 0.2, key = (i, j) => i * 100003 + j;
    P.forEach((p, k) => { const kk = key(Math.floor(p.x / cell), Math.floor(p.y / cell)); (nearIdx.get(kk) || nearIdx.set(kk, []).get(kk)).push(k); });
    ref.normals = P.map(p => {
      const ci = Math.floor(p.x / cell), cj = Math.floor(p.y / cell), nb = [];
      for (let di = -1; di <= 1; di++) for (let dj = -1; dj <= 1; dj++) for (const k of nearIdx.get(key(ci + di, cj + dj)) || [])
        if ((P[k].x - p.x) ** 2 + (P[k].y - p.y) ** 2 < cell * cell) nb.push(P[k]);
      if (nb.length < 4) return null;
      const f = LC.fitLine(nb); if (f.rmse > 0.03) return null;
      return [-f.uy, f.ux];
    });
    return ref.normals;
  }
  // 3b. ICP point-à-ligne (Gauss-Newton sur la distance aux droites locales du scan de référence)
  function scanICPLine(ref, pts, init, opt = {}) {
    const o = { iters: 80, dmax0: 1.0, dmaxMin: 0.1, trim: 0.85, ...opt };
    const nearest = ref.nearest || (ref.nearest = pointIndex(ref.pts)), N = refNormals(ref);
    let x = init.x, y = init.y, th = init.th * D2R, dmax = o.dmax0, it = 0, rms = 0;
    for (; it < o.iters; it++) {
      const c = Math.cos(th), s = Math.sin(th), R = [];
      for (const p of pts) {
        const rx = c * p.x - s * p.y, ry = s * p.x + c * p.y, k = nearest(x + rx, y + ry, dmax);
        if (k < 0 || !N[k]) continue;
        const [nx, ny] = N[k]; R.push([rx, ry, nx, ny, nx * (x + rx - ref.pts[k].x) + ny * (y + ry - ref.pts[k].y)]);
      }
      if (R.length < 10) break;
      R.sort((a, b) => Math.abs(a[4]) - Math.abs(b[4]));
      const Q = R.slice(0, Math.ceil(R.length * o.trim)), A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
      for (const [rx, ry, nx, ny, e] of Q) { const J = [nx, ny, -nx * ry + ny * rx]; for (let i = 0; i < 3; i++) { b[i] -= J[i] * e; for (let j = 0; j < 3; j++) A[i][j] += J[i] * J[j]; } }
      for (let i = 0; i < 3; i++) A[i][i] += 1e-6;
      const sol = solve3(A, b); if (!sol) break;
      x += sol[0]; y += sol[1]; th += sol[2];
      rms = Math.sqrt(Q.reduce((a, q) => a + q[4] * q[4], 0) / Q.length);
      if (Math.hypot(sol[0], sol[1]) < 1e-5 && Math.abs(sol[2]) < 1e-6) { if (dmax <= o.dmaxMin) break; }
      dmax = Math.max(o.dmaxMin, dmax * 0.85);
    }
    return { x, y, th: normDeg(th * R2D), score: -rms, iters: it };
  }
  // 3c. Corrélation multi-résolution : le scan de référence devient une mini grid map, puis multi-résolution
  //     (méthode 1b) et affinage par gradient (méthode 1c)
  function scanCorrelative(ref, pts, init, opt = {}) {
    if (!ref.field) {
      const { grid } = LC.scanToGrid({ x: 0, y: 0 }, ref.ptsAll || ref.pts, { res: 0.05, margin: 1.5, fillBetween: true });
      ref.field = distanceField(grid);
    }
    const r = gridMultiRes(ref.field, pts, init, opt);
    const g = gridGradient(ref.field, pts, r, { dmax0: 0.2, dmaxMin: 0.08 });
    return { ...g, score: r.score, iters: r.iters + g.iters };
  }

  // ==========================================================================
  //  Banc d'évaluation (mêmes scans, mêmes poses initiales pour toutes les méthodes)
  // ==========================================================================
  const METHODS = {
    grid: { brute: ['Recherche exhaustive', (R, P, i, o) => gridBrute(R.field, P, i, o)],
            multires: ['Multi-résolution', (R, P, i, o) => gridMultiRes(R.field, P, i, o)],
            gradient: ['Descente de gradient (Gauss-Newton)', (R, P, i, o) => gridGradient(R.field, P, i, o)] },
    vector: { brute: ['Recherche exhaustive', (R, P, i, o) => vecBrute(R.index, P, i, o)],
              icp: ['Descente de gradient (ICP point-segment)', (R, P, i, o) => vecICP(R.index, P, i, o)],
              direct: ['Calcul direct (coins) + ICP', (R, P, i, o) => vecDirect(R.index, P, i, o)] },
    scan: { icpPoint: ['ICP point-à-point', (R, P, i, o) => scanICPPoint(R, P, i, o)],
            icpLine: ['ICP point-à-ligne', (R, P, i, o) => scanICPLine(R, P, i, o)],
            correl: ['Corrélation multi-résolution', (R, P, i, o) => scanCorrelative(R, P, i, o)] }
  };
  const REF_LABEL = { grid: 'Grid map', vector: 'Carte vectorielle', scan: 'Entre scans' };

  // Perturbation reproductible : distance exacte `dp` dans une direction aléatoire, angle ±`da`
  function perturb(truth, dp, da, rng) {
    const dir = rng() * 2 * Math.PI, sg = rng() < 0.5 ? -1 : 1;
    return { x: truth.x + dp * Math.cos(dir), y: truth.y + dp * Math.sin(dir), th: normDeg(truth.th + sg * da) };
  }

  // Références pour un jeu de scans placés (poses connues) ; `except` = indice du scan exclu (leave-one-out)
  function buildReferences(scans, except, opt = {}) {
    const placed = scans.filter((_, k) => k !== except);
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity; const data = [];
    for (const s of placed) {
      const P = LC.polarToGlobal(s, s.pose, { dMin: 0.05, dMax: 12 });
      for (const q of P) { if (q.x < x0) x0 = q.x; if (q.x > x1) x1 = q.x; if (q.y < y0) y0 = q.y; if (q.y > y1) y1 = q.y; }
      data.push([{ x: s.pose.x, y: s.pose.y }, P]);
    }
    const g = LC.createGrid({ xMin: x0 - 1, yMin: y0 - 1, xMax: x1 + 1, yMax: y1 + 1, res: opt.res ?? 0.05 });
    for (const [r, P] of data) LC.updateGrid(g, r, P, { fillBetween: true });
    const M = LC.createVectorMap();
    for (const [r, P] of data) LC.updateVectorMap(M, r, P, { extract: { minLen: 0.3 } });
    return { grid: g, field: distanceField(g), vmap: M, index: segIndex(M.features) };
  }

  /**
   * cfg : {
   *   scenario: 'sim' | 'real', seed, levels: [[dp (m), da (°)], ...], draws, refs: ['grid','vector','scan'],
   *   methods: { grid: [...], vector: [...], scan: [...] }, window: { rxy, rth },
   *   success: { pos, ang }, coarse: { pos, ang },
   *   sim: { nMap, nTest, legs }, real: { scans: [{ name, a, d, i, pose }], pairMaxDist }
   * }
   * onProgress(fraction, texte) peut être async (le navigateur peut ainsi rafraîchir l'affichage).
   */
  async function runLocBench(cfg, onProgress) {
    const C = Object.assign({
      scenario: 'sim', seed: 7, levels: [[0, 0], [0.1, 5], [0.25, 10], [0.5, 20], [1.0, 45]], draws: 5,
      refs: ['grid', 'vector', 'scan'], methods: null, window: { rxy: 1.2, rth: 50 },
      success: { pos: 0.05, ang: 2 }, coarse: { pos: 0.20, ang: 5 }, sim: {}, real: {}
    }, cfg);
    const methods = C.methods || Object.fromEntries(C.refs.map(r => [r, Object.keys(METHODS[r])]));
    const rng = mulberry32(C.seed), rows = [], cases = [];
    const prep = s => uniformSubsample(localPoints(s), { max: 350 });

    // ---- construction des cas de test : { ref, refObj, pts, truth, name }
    if (C.scenario === 'sim') {
      const sc = classroomScene(C.seed, { legs: C.sim.legs }), simR = mulberry32(C.seed + 1);
      const randFree = () => { for (;;) { const p = { x: 0.6 + simR() * 12.4, y: 0.6 + simR() * 8.8, th: normDeg(simR() * 360) }; if (sc.free(p.x, p.y)) return p; } };
      const nMap = C.sim.nMap ?? 9, nTest = C.sim.nTest ?? 10;
      const mapPoses = [];
      for (let iy = 0; iy < 3; iy++) for (let ix = 0; ix < 3 && mapPoses.length < nMap; ix++) {   // positions de cartographie réparties
        let p = { x: 1.8 + ix * 4.6, y: 1.4 + iy * 3.1, th: normDeg(simR() * 360) }, tries = 0;
        while (!sc.free(p.x, p.y) && tries++ < 50) p = { x: p.x + (simR() - 0.5), y: p.y + (simR() - 0.5), th: p.th };
        mapPoses.push(p);
      }
      const mapScans = mapPoses.map(p => Object.assign(simulateScan(sc.walls, p, { tours: 3 }, simR), { pose: p }));
      const R = buildReferences(mapScans, -1);
      for (let t = 0; t < nTest; t++) {
        const truth = randFree(), scan = simulateScan(sc.walls, truth, { tours: 1 }, simR), pts = prep(scan);
        if (C.refs.includes('grid')) cases.push({ ref: 'grid', R, pts, truth, name: `sim${t + 1}` });
        if (C.refs.includes('vector')) cases.push({ ref: 'vector', R, pts, truth, name: `sim${t + 1}` });
        if (C.refs.includes('scan')) {
          // scan de référence pris près (0,3 à 0,6 m, ±20°) : la vérité est la pose relative
          let rp; do { const d = 0.3 + simR() * 0.3, a = simR() * 2 * Math.PI; rp = { x: truth.x + d * Math.cos(a), y: truth.y + d * Math.sin(a), th: normDeg(truth.th + (simR() - 0.5) * 40) }; } while (!sc.free(rp.x, rp.y));
          const rs = simulateScan(sc.walls, rp, { tours: 3 }, simR), all = localPoints(rs);
          cases.push({ ref: 'scan', R: { pts: uniformSubsample(all, { cell: 0.03, minR: 0.3, max: 3000 }), ptsAll: all }, pts, truth: relPose(rp, truth), name: `sim${t + 1}` });
        }
      }
    } else {
      const S = C.real.scans, maxD = C.real.pairMaxDist ?? 1.5;
      for (let k = 0; k < S.length; k++) {
        await onProgress?.(0, `Construction des références (${k + 1}/${S.length})`);
        const pts = prep(S[k]);
        if (C.refs.includes('grid') || C.refs.includes('vector')) {
          const R = buildReferences(S, k);                            // carte construite SANS le scan testé
          if (C.refs.includes('grid')) cases.push({ ref: 'grid', R, pts, truth: S[k].pose, name: S[k].name });
          if (C.refs.includes('vector')) cases.push({ ref: 'vector', R, pts, truth: S[k].pose, name: S[k].name });
        }
        if (C.refs.includes('scan')) {
          // scan de référence = le scan le plus proche (pris à moins de maxD)
          let bj = -1, bd = Infinity;
          S.forEach((o, j) => { if (j === k) return; const d = Math.hypot(o.pose.x - S[k].pose.x, o.pose.y - S[k].pose.y); if (d < bd) { bd = d; bj = j; } });
          if (bj >= 0 && bd <= maxD) {
            const all = localPoints(S[bj]);
            cases.push({ ref: 'scan', R: { pts: uniformSubsample(all, { cell: 0.03, minR: 0.3, max: 3000 }), ptsAll: all }, pts,
                         truth: relPose(S[bj].pose, S[k].pose), name: `${S[k].name}→${S[bj].name}` });
          }
        }
      }
    }

    // ---- poses initiales : tirées UNE fois par cas et par niveau, partagées par toutes les méthodes
    for (const cs of cases) cs.inits = C.levels.map(([dp, da]) => Array.from({ length: dp === 0 && da === 0 ? 1 : C.draws }, () => perturb(cs.truth, dp, da, rng)));
    const total = cases.reduce((a, cs) => a + methods[cs.ref].length * cs.inits.reduce((b, l) => b + l.length, 0), 0);
    let done = 0;
    for (const cs of cases) for (const m of methods[cs.ref]) {
      const [label, fn] = METHODS[cs.ref][m];
      for (let l = 0; l < C.levels.length; l++) for (const init of cs.inits[l]) {
        const t0 = now();
        let r; try { r = fn(cs.R, cs.pts, init, { ...C.window }); } catch (e) { r = { x: NaN, y: NaN, th: NaN, failed: true }; }
        const ms = now() - t0, e = r.failed ? { pos: NaN, ang: NaN } : poseError(r, cs.truth);
        // une méthode qui échoue (ex. aucun coin trouvé) compte comme un échec, même si la pose initiale était juste
        const ok = !r.failed && e.pos <= C.success.pos && e.ang <= C.success.ang, okC = !r.failed && e.pos <= C.coarse.pos && e.ang <= C.coarse.ang;
        rows.push({ scenario: C.scenario, ref: cs.ref, method: m, label, scan: cs.name, level: l, dp: C.levels[l][0], da: C.levels[l][1],
                    epos: e.pos, eang: e.ang, ms, ok, okCoarse: okC, failed: !!r.failed, iters: r.iters ?? 0 });
        if (++done % 10 === 0) await onProgress?.(done / total, `${REF_LABEL[cs.ref]} · ${label} · ${cs.name}`);
      }
    }
    return { cfg: C, rows, summary: summarize(rows, C) };
  }

  const median = a => { if (!a.length) return NaN; const s = a.slice().sort((p, q) => p - q), m = s.length >> 1; return s.length % 2 ? s[m] : (s[m - 1] + s[m]) / 2; };
  function summarize(rows, C) {
    const groups = new Map();
    for (const r of rows) {
      const k = `${r.ref}|${r.method}|${r.level}`;
      if (!groups.has(k)) groups.set(k, { ref: r.ref, method: r.method, label: r.label, level: r.level, dp: r.dp, da: r.da, R: [] });
      groups.get(k).R.push(r);
    }
    return [...groups.values()].map(g => {
      const S = g.R.filter(r => r.ok);
      return { ref: g.ref, method: g.method, label: g.label, level: g.level, dp: g.dp, da: g.da, n: g.R.length,
               success: S.length / g.R.length, coarse: g.R.filter(r => r.okCoarse).length / g.R.length,
               failed: g.R.filter(r => r.failed).length / g.R.length,
               medPos: median(g.R.filter(r => !r.failed).map(r => r.epos)), medAng: median(g.R.filter(r => !r.failed).map(r => r.eang)),
               medPosOk: median(S.map(r => r.epos)), medAngOk: median(S.map(r => r.eang)),
               meanMs: g.R.reduce((a, r) => a + r.ms, 0) / g.R.length };
    });
  }
  function benchToCSV(res) {
    const h = 'scenario,reference,methode,scan,niveau,perturbation_m,perturbation_deg,erreur_position_m,erreur_angle_deg,temps_ms,reussite,reussite_grossiere,iterations';
    return h + ',echec_methode\n' + res.rows.map(r => [r.scenario, r.ref, r.method, r.scan, r.level, r.dp, r.da, isFinite(r.epos) ? r.epos.toFixed(4) : '', isFinite(r.eang) ? r.eang.toFixed(3) : '', r.ms.toFixed(1), r.ok ? 1 : 0, r.okCoarse ? 1 : 0, r.iters, r.failed ? 1 : 0].join(',')).join('\n') + '\n';
  }
  function benchToMarkdown(res, title) {
    const C = res.cfg, fmt = (v, d) => isFinite(v) ? v.toFixed(d) : '—';
    let s = `## ${title}\n\nRéussite : erreur ≤ ${C.success.pos * 100} cm **et** ≤ ${C.success.ang}° · réussite grossière : ≤ ${C.coarse.pos * 100} cm et ≤ ${C.coarse.ang}°. ` +
            `Fenêtre de recherche des méthodes globales : ±${C.window.rxy} m, ±${C.window.rth}°. ${C.draws} tirages par cas et par niveau (1 seul au niveau 0).\n\n`;
    for (const ref of C.refs) {
      s += `### ${REF_LABEL[ref]}\n\n| Méthode | Perturbation initiale | Réussite | Réussite grossière | Erreur pos. médiane | Erreur ang. médiane | Temps moyen |\n|---|---|---|---|---|---|---|\n`;
      // (erreurs médianes calculées sur les essais où la méthode a rendu une pose ; « échec méthode » = aucune pose)
      for (const g of res.summary.filter(g => g.ref === ref).sort((a, b) => a.method.localeCompare(b.method) || a.level - b.level))
        s += `| ${g.label} | ${fmt(g.dp, 2)} m / ${g.da}° | ${Math.round(g.success * 100)} %${g.failed ? ` (dont ${Math.round(g.failed * 100)} % sans résultat)` : ''} | ${Math.round(g.coarse * 100)} % | ${fmt(g.medPos * 100, 1)} cm | ${fmt(g.medAng, 2)}° | ${fmt(g.meanMs, 0)} ms |\n`;
      s += '\n';
    }
    return s;
  }

  return {
    mulberry32, localPoints, uniformSubsample, transform, relPose, compose, poseError, perturb,
    classroomScene, simulateScan,
    distanceField, fieldSample, gridBrute, gridMultiRes, gridGradient,
    segIndex, vecScore, vecBrute, vecICP, vecDirect, lineFeatures,
    pointIndex, scanICPPoint, scanICPLine, scanCorrelative,
    buildReferences, runLocBench, summarize, benchToCSV, benchToMarkdown, METHODS, REF_LABEL
  };
});
