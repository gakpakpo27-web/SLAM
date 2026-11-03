/* ============================================================================
 *  lidar_maptools.js – Traitement des cartes et des obstacles
 *  Dépend de lidar_core.js (LidarCore) et lidar_loc.js (LidarLoc).
 *  Navigateur : window.LidarMapTools ; Node : require('./lidar_maptools.js').
 *
 *  Reprend les étapes des documents du cours qui n'étaient pas encore outillées :
 *   - « grid map » §1 et §5 : cellules enrichies (compteur d'impacts, nombre d'observations,
 *     horodatage), décroissance temporelle, effacement des obstacles obsolètes, lissage spatial,
 *     cellules d'intérêt (petits obstacles isolés mais sûrs) ;
 *   - « carte vectorielle » §2 et §3 : prétraitement (points aberrants), segmentation en clusters
 *     (distance, angle, intensité), reconnaissance géométrique (régression, RANSAC,
 *     Douglas-Peucker, cercles), §4–§6 : features circulaires fusionnées et suivies
 *     (confiance, absence), nettoyage (collisions géométriques, redondances) ;
 *   - « cartographie » §2 et §6–§9 : durée de vie / probabilité de persistance des obstacles
 *     (objets mobiles), alignement global des scans (correction géométrique des poses).
 *  Et pour exploiter la carte : carte de coût (obstacles gonflés du rayon du robot) et
 *  planification de trajet A*, métriques de qualité pour comparer avant / après traitement.
 *
 *  Conventions : poses { x, y, th (deg) } ; points { x, y } en mètres ; grilles de lidar_core.js.
 * ========================================================================== */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory(require('./lidar_core.js'), require('./lidar_loc.js'));
  else root.LidarMapTools = factory(root.LidarCore, root.LidarLoc);
})(typeof self !== 'undefined' ? self : this, function (LC, LL) {
  'use strict';
  const D2R = Math.PI / 180, R2D = 180 / Math.PI;
  const { UNKNOWN, FREE_T, OCC_T } = LC;
  const now = () => (typeof performance !== 'undefined' ? performance.now() : Date.now());
  const normDeg = a => ((a + 180) % 360 + 360) % 360 - 180;
  const mod = (a, m) => ((a % m) + m) % m;

  // ==========================================================================
  //  1. Prétraitement d'un scan (points aberrants)
  // ==========================================================================
  const PRE_DEFAULTS = {
    dMin: 0.10, dMax: 12,          // portée utile (m) ; LD06 : 0,02 à 12 m. Sous dMin : le robot lui-même (châssis, câbles)
    iMin: 0,                       // intensité minimale (qualité du point, 0..255)
    isolated: true, isoRadius: 0.08, isoWin: 2,   // point sans voisin à moins de max(isoRadius, 2,5 × écart angulaire)
    mixed: true, mixedJump: 0.20,  // « points mixtes » : entre un premier plan et un fond (faisceau à cheval)
    scale: 1
  };
  /**
   * scan { a: [deg], d: [mm], i?: [] }, pose { x, y, th } -> { pts: [{x,y,r}] globaux (ordre angulaire),
   * local: [{x,y}], removed: { range, intensity, isolated, mixed }, kept, total }
   */
  function preprocessScan(scan, pose, opt = {}) {
    const o = Object.assign({}, PRE_DEFAULTS, opt);
    const removed = { range: 0, intensity: 0, isolated: 0, mixed: 0 };
    const idx = [];
    for (let k = 0; k < scan.a.length; k++) {
      const r = scan.d[k] / 1000;
      if (!(r >= o.dMin && r <= o.dMax)) { removed.range++; continue; }
      if (scan.i && o.iMin > 0 && scan.i[k] < o.iMin) { removed.intensity++; continue; }
      idx.push(k);
    }
    idx.sort((p, q) => scan.a[p] - scan.a[q]);
    const n = idx.length;
    const P = idx.map(k => { const r = scan.d[k] / 1000 * o.scale, a = scan.a[k] * D2R; return { x: r * Math.cos(a), y: r * Math.sin(a), r, a: scan.a[k] }; });
    const keep = new Uint8Array(n).fill(1);
    const adj = (k, m) => Math.abs(normDeg(P[m].a - P[k].a)) < 3;      // voisins angulaires réels (pas de trou)
    if (o.mixed && n >= 3) {
      for (let k = 0; k < n; k++) {
        const p = (k - 1 + n) % n, q = (k + 1) % n;
        if (!adj(k, p) || !adj(k, q)) continue;
        const r = P[k].r, rp = P[p].r, rq = P[q].r;
        if (Math.abs(r - rp) > o.mixedJump && Math.abs(r - rq) > o.mixedJump && (r - rp) * (rq - r) > 0) { keep[k] = 0; removed.mixed++; }
      }
    }
    if (o.isolated && n >= 3) {
      const iso = [];
      for (let k = 0; k < n; k++) {
        if (!keep[k]) continue;
        const rad = Math.max(o.isoRadius, 2.5 * P[k].r * 0.8 * D2R);
        let ok = false;
        for (let w = 1; w <= o.isoWin && !ok; w++) for (const m of [(k - w + n) % n, (k + w) % n]) {
          if (m !== k && Math.hypot(P[m].x - P[k].x, P[m].y - P[k].y) <= rad) { ok = true; break; }
        }
        if (!ok) iso.push(k);
      }
      for (const k of iso) { keep[k] = 0; removed.isolated++; }
    }
    const c = Math.cos(pose.th * D2R), s = Math.sin(pose.th * D2R), pts = [], local = [];
    for (let k = 0; k < n; k++) if (keep[k]) {
      const p = P[k];
      local.push({ x: p.x, y: p.y });
      pts.push({ x: pose.x + c * p.x - s * p.y, y: pose.y + s * p.x + c * p.y, r: p.r });
    }
    return { pts, local, removed, kept: pts.length, total: scan.a.length };
  }

  // ==========================================================================
  //  2. Segmentation en clusters
  // ==========================================================================
  /**
   * Nouveau cluster quand deux points successifs (ordre angulaire) sont séparés :
   *  - en distance de plus de gapBase + gapK·r (les points s'espacent avec la distance),
   *  - ou en angle de plus de angGap (des retours manquent : vitre, surface noire, hors portée),
   *  - ou (si iJump > 0 et intensités fournies) en intensité de plus de iJump.
   * Renvoie [[{x,y,...}], ...] ; les clusters de moins de minPts points sont écartés.
   */
  function clusterScan(pts, origin, opt = {}) {
    const o = Object.assign({ gapBase: 0.10, gapK: 0.03, angGap: 4, iJump: 0, minPts: 1 }, opt);
    const n = pts.length; if (!n) return [];
    const rng = p => Math.hypot(p.x - origin.x, p.y - origin.y);
    const brg = p => Math.atan2(p.y - origin.y, p.x - origin.x) * R2D;
    const brk = (p, q) => {
      const d = Math.hypot(p.x - q.x, p.y - q.y) - (o.gapBase + o.gapK * Math.max(rng(p), rng(q)));
      if (d > 0) return d + 1;
      if (Math.abs(normDeg(brg(q) - brg(p))) > o.angGap) return 1;
      if (o.iJump > 0 && p.i !== undefined && q.i !== undefined && Math.abs(p.i - q.i) > o.iJump) return 0.5;
      return 0;
    };
    // commencer après la plus forte rupture (évite de couper un objet à l'angle 0°/360°)
    let start = 0, worst = -1;
    for (let k = 0; k < n; k++) { const b = brk(pts[k], pts[(k + 1) % n]); if (b > worst) { worst = b; start = (k + 1) % n; } }
    const out = []; let cur = [pts[start]];
    for (let m = 1; m < n; m++) {
      const p = pts[(start + m - 1) % n], q = pts[(start + m) % n];
      if (brk(p, q) > 0) { out.push(cur); cur = []; }
      cur.push(q);
    }
    out.push(cur);
    return out.filter(C => C.length >= o.minPts);
  }

  // ==========================================================================
  //  3. Reconnaissance géométrique
  // ==========================================================================
  function solve3(A, b) {                       // élimination de Gauss avec pivot partiel
    const M = A.map((r, i) => [...r, b[i]]);
    for (let c = 0; c < 3; c++) {
      let p = c; for (let r = c + 1; r < 3; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
      if (Math.abs(M[p][c]) < 1e-14) return null;
      [M[c], M[p]] = [M[p], M[c]];
      for (let r = 0; r < 3; r++) if (r !== c) { const f = M[r][c] / M[c][c]; for (let k = c; k < 4; k++) M[r][k] -= f * M[c][k]; }
    }
    return [M[0][3] / M[0][0], M[1][3] / M[1][1], M[2][3] / M[2][2]];
  }
  /**
   * Cercle par moindres carrés : solution algébrique (Kåsa) puis affinage géométrique
   * (Gauss-Newton sur Σ (|p − c| − r)²). Renvoie { x, y, r, rmse } ou null.
   */
  function fitCircle(P) {
    if (P.length < 3) return null;
    let mx = 0, my = 0; for (const p of P) { mx += p.x; my += p.y; } mx /= P.length; my /= P.length;
    // Kåsa sur coordonnées centrées : x² + y² + D x + E y + F = 0
    const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
    for (const p of P) {
      const x = p.x - mx, y = p.y - my, z = -(x * x + y * y), v = [x, y, 1];
      for (let i = 0; i < 3; i++) { b[i] += v[i] * z; for (let j = 0; j < 3; j++) A[i][j] += v[i] * v[j]; }
    }
    const s = solve3(A, b); if (!s) return null;
    let cx = -s[0] / 2, cy = -s[1] / 2, r2 = cx * cx + cy * cy - s[2];
    if (!(r2 > 0)) return null;
    let r = Math.sqrt(r2);
    for (let it = 0; it < 20; it++) {
      const JtJ = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], Jtr = [0, 0, 0];
      for (const p of P) {
        const dx = p.x - mx - cx, dy = p.y - my - cy, d = Math.hypot(dx, dy) || 1e-12, res = d - r;
        const J = [-dx / d, -dy / d, -1];
        for (let i = 0; i < 3; i++) { Jtr[i] += J[i] * res; for (let j = 0; j < 3; j++) JtJ[i][j] += J[i] * J[j]; }
      }
      const st = solve3(JtJ, Jtr.map(v => -v)); if (!st) break;
      cx += st[0]; cy += st[1]; r += st[2];
      if (Math.hypot(st[0], st[1], st[2]) < 1e-7) break;
    }
    let e2 = 0; for (const p of P) { const d = Math.hypot(p.x - mx - cx, p.y - my - cy) - r; e2 += d * d; }
    return { x: cx + mx, y: cy + my, r: Math.abs(r), rmse: Math.sqrt(e2 / P.length) };
  }

  /** Douglas-Peucker : indices des sommets de la polyligne simplifiée (tolérance tol, m) */
  function douglasPeucker(P, tol) {
    const n = P.length; if (n <= 2) return n === 2 ? [0, 1] : n ? [0] : [];
    const keep = new Uint8Array(n); keep[0] = keep[n - 1] = 1;
    const stack = [[0, n - 1]];
    while (stack.length) {
      const [a, b] = stack.pop(); if (b - a < 2) continue;
      const ch = LC.seg(P[a].x, P[a].y, P[b].x, P[b].y), deg = LC.segLen(ch) < 1e-9;
      let dm = -1, im = -1;
      for (let k = a + 1; k < b; k++) {
        const d = deg ? Math.hypot(P[k].x - P[a].x, P[k].y - P[a].y) : LC.pointLineDist(P[k].x, P[k].y, ch);
        if (d > dm) { dm = d; im = k; }
      }
      if (dm > tol) { keep[im] = 1; stack.push([a, im], [im, b]); }
    }
    const out = []; for (let k = 0; k < n; k++) if (keep[k]) out.push(k);
    return out;
  }
  /** Segments d'un cluster par Douglas-Peucker puis régression (moindres carrés totaux) sur chaque morceau */
  function dpSegments(C, opt = {}) {
    const o = Object.assign({ tol: 0.04, minPts: 5, minLen: 0.20 }, opt);
    const v = douglasPeucker(C, o.tol), out = [];
    for (let k = 0; k + 1 < v.length; k++) {
      const piece = C.slice(v[k], v[k + 1] + 1);
      if (piece.length < o.minPts) continue;
      const s = LC.fitSegment(piece);
      if (LC.segLen(s) >= o.minLen) out.push(s);
    }
    return out;
  }
  /**
   * RANSAC séquentiel : on tire 2 points, on compte les points à moins de tol de la droite,
   * on garde la meilleure droite, on la réajuste sur ses inliers, on la découpe là où les inliers
   * sont séparés de plus de gapTol (deux murs alignés = deux segments), on retire ses inliers et
   * on recommence. Robuste aux points aberrants (chaises devant un mur, reflets).
   */
  function ransacLines(P, opt = {}) {
    const o = Object.assign({ tol: 0.03, minInliers: 8, iters: 150, gapTol: 0.25, minLen: 0.25, maxLines: 40, rng: LL.mulberry32(1) }, opt);
    let rest = P.slice(); const out = [];
    for (let L = 0; L < o.maxLines && rest.length >= o.minInliers; L++) {
      let best = null, bestN = 0;
      for (let it = 0; it < o.iters; it++) {
        const a = rest[Math.floor(o.rng() * rest.length)], b = rest[Math.floor(o.rng() * rest.length)];
        const dx = b.x - a.x, dy = b.y - a.y, l = Math.hypot(dx, dy); if (l < 0.05) continue;
        const nx = -dy / l, ny = dx / l, c = nx * a.x + ny * a.y;
        let cnt = 0; for (const p of rest) if (Math.abs(nx * p.x + ny * p.y - c) <= o.tol) cnt++;
        if (cnt > bestN) { bestN = cnt; best = { nx, ny, c }; }
      }
      if (!best || bestN < o.minInliers) break;
      let inl = rest.filter(p => Math.abs(best.nx * p.x + best.ny * p.y - best.c) <= o.tol);
      const f = LC.fitLine(inl), nx = -f.uy, ny = f.ux, c = nx * f.mx + ny * f.my;
      inl = rest.filter(p => Math.abs(nx * p.x + ny * p.y - c) <= o.tol);
      const inSet = new Set(inl);
      rest = rest.filter(p => !inSet.has(p));
      const t = p => (p.x - f.mx) * f.ux + (p.y - f.my) * f.uy;
      inl.sort((p, q) => t(p) - t(q));
      let run = [inl[0]];
      const flush = () => { if (run.length >= Math.max(3, o.minInliers / 2)) { const s = LC.fitSegment(run); if (LC.segLen(s) >= o.minLen) out.push(s); } };
      for (let k = 1; k < inl.length; k++) { if (t(inl[k]) - t(inl[k - 1]) > o.gapTol) { flush(); run = []; } run.push(inl[k]); }
      flush();
    }
    return out;
  }

  /** Petit cluster -> cercle approché : on voit la moitié avant de l'objet, dont le barycentre est
   *  à 2r/π ≈ 0,64 r du centre, côté robot ; le centre est donc reculé d'autant. */
  function smallObjectCircle(C, origin) {
    let mx = 0, my = 0; for (const p of C) { mx += p.x; my += p.y; } mx /= C.length; my /= C.length;
    let ext = 0; for (const p of C) for (const q of C) ext = Math.max(ext, Math.hypot(p.x - q.x, p.y - q.y));
    const r = Math.max(0.015, ext / 2), dx = mx - origin.x, dy = my - origin.y, l = Math.hypot(dx, dy) || 1;
    return { x: mx + dx / l * 0.64 * r, y: my + dy / l * 0.64 * r, r, n: C.length, rmse: 0, kind: 'petit' };
  }

  const FEAT_DEFAULTS = {
    method: 'split',                              // 'split' (split-and-merge, partie 4), 'dp' (Douglas-Peucker), 'ransac'
    cluster: { gapBase: 0.10, gapK: 0.03, angGap: 4, minPts: 2 },
    smallMax: 0.15,                               // cluster plus petit que ça : petit objet (pied de chaise, poteau)
    circle: { minPts: 6, maxR: 0.30, minR: 0.02, maxRmse: 0.012 },
    extract: { minLen: 0.25 }, dp: {}, ransac: {}
  };
  /**
   * Extraction complète des features d'un scan (repère de la carte) :
   * clusters -> petits objets (cercles), objets circulaires (cercle ajusté, convexe face au robot),
   * sinon segments par la méthode choisie.
   */
  function extractFeatures(pts, origin, opt = {}) {
    const o = Object.assign({}, FEAT_DEFAULTS, opt, { circle: Object.assign({}, FEAT_DEFAULTS.circle, opt.circle), cluster: Object.assign({}, FEAT_DEFAULTS.cluster, opt.cluster) });
    const clusters = clusterScan(pts, origin, o.cluster);
    const segments = [], circles = [], stats = { clusters: clusters.length, small: 0, round: 0, linear: 0 };
    for (const C of clusters) {
      let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
      for (const p of C) { if (p.x < x0) x0 = p.x; if (p.x > x1) x1 = p.x; if (p.y < y0) y0 = p.y; if (p.y > y1) y1 = p.y; }
      const ext = Math.hypot(x1 - x0, y1 - y0);
      if (ext <= o.smallMax) { circles.push(smallObjectCircle(C, origin)); stats.small++; continue; }
      if (C.length >= o.circle.minPts && ext <= 2 * o.circle.maxR) {
        const f = fitCircle(C);
        if (f && f.r >= o.circle.minR && f.r <= o.circle.maxR && f.rmse <= o.circle.maxRmse) {
          let mx = 0, my = 0; for (const p of C) { mx += p.x; my += p.y; } mx /= C.length; my /= C.length;
          // l'objet doit présenter sa face convexe au robot : centre plus loin que les points
          if (Math.hypot(f.x - origin.x, f.y - origin.y) > Math.hypot(mx - origin.x, my - origin.y)) {
            circles.push({ ...f, n: C.length, kind: 'cercle' }); stats.round++; continue;
          }
        }
      }
      const segs = o.method === 'ransac' ? ransacLines(C, o.ransac) : o.method === 'dp' ? dpSegments(C, o.dp) : LC.extractSegments(C, origin, o.extract);
      if (segs.length) stats.linear++;
      segments.push(...segs);
    }
    return { clusters, segments, circles, stats };
  }

  // ==========================================================================
  //  4. Carte d'obstacles ponctuels / circulaires (features « cercle »)
  // ==========================================================================
  const OM_DEFAULTS = { assoc: 0.15, hit: 1, miss: 1, confMax: 10, maxAge: 0, absMinRays: 2, absMargin: 0.10, minR: 0.03, mobileMin: 2 };
  function createObstacleMap() { return { items: [], scans: 0, nextId: 1 }; }
  /**
   * Intègre les cercles observés dans un scan :
   *  absence : un faisceau qui passe à travers un obstacle connu (et s'arrête plus loin) le prouve
   *            absent -> confiance −, compteur de « vu absent » + (persistance) ;
   *  association : cercle observé à moins de assoc d'un obstacle -> moyenne pondérée (fusion) ;
   *  nettoyage : fusion des doublons, durée de vie maxAge, suppression à confiance nulle.
   * persistance = nObs / (nObs + absences) ; « mobile » si absent au moins mobileMin fois et persistance < 0,5.
   */
  function updateObstacleMap(M, robot, pts, circles, opt = {}) {
    const o = Object.assign({}, OM_DEFAULTS, opt), t = ++M.scans;
    const st = { scan: t, added: [], merged: [], weakened: [], removed: [], expired: [] };
    if (M.items.length) {
      const miss = new Map(), step = Math.max(1, Math.ceil(pts.length / 900));
      for (let k = 0; k < pts.length; k += step) {
        const p = pts[k], dx = p.x - robot.x, dy = p.y - robot.y, L = Math.hypot(dx, dy); if (L < 0.1) continue;
        const ux = dx / L, uy = dy / L;
        for (const it of M.items) {
          const re = Math.max(it.r, o.minR), cx = it.x - robot.x, cy = it.y - robot.y, u = cx * ux + cy * uy;
          if (u <= 0 || u + re + o.absMargin >= L) continue;          // l'impact est sur (ou avant) l'obstacle
          if (Math.abs(cx * uy - cy * ux) < 0.8 * re) miss.set(it.id, (miss.get(it.id) || 0) + 1);
        }
      }
      for (const [id, c] of miss) {
        if (c < o.absMinRays) continue;
        const it = M.items.find(q => q.id === id);
        it.conf -= o.miss; it.misses++; st.weakened.push(id);
      }
      M.items = M.items.filter(it => { if (it.conf <= 0) { st.removed.push(it.id); return false; } return true; });
    }
    for (const c of circles) {
      let best = null, bd = o.assoc;
      for (const it of M.items) { const d = Math.hypot(it.x - c.x, it.y - c.y); if (d <= bd) { bd = d; best = it; } }
      if (!best) {
        const it = { id: M.nextId++, x: c.x, y: c.y, r: c.r, conf: 1, nObs: 1, misses: 0, firstSeen: t, lastSeen: t, kind: c.kind || 'cercle' };
        M.items.push(it); st.added.push(it.id); continue;
      }
      const w = Math.min(best.nObs, 20);
      best.x = (best.x * w + c.x) / (w + 1); best.y = (best.y * w + c.y) / (w + 1); best.r = (best.r * w + c.r) / (w + 1);
      if (best.lastSeen !== t) { best.nObs++; best.conf = Math.min(o.confMax, best.conf + o.hit); }
      best.lastSeen = t;
      if (c.kind === 'cercle') best.kind = 'cercle';
      if (!st.merged.includes(best.id) && !st.added.includes(best.id)) st.merged.push(best.id);
    }
    // doublons (deux obstacles qui ont convergé au même endroit)
    for (let i = 0; i < M.items.length; i++) for (let j = M.items.length - 1; j > i; j--) {
      const a = M.items[i], b = M.items[j];
      if (Math.hypot(a.x - b.x, a.y - b.y) > o.assoc * 0.5) continue;
      const wa = a.nObs, wb = b.nObs;
      Object.assign(a, { x: (a.x * wa + b.x * wb) / (wa + wb), y: (a.y * wa + b.y * wb) / (wa + wb), r: (a.r * wa + b.r * wb) / (wa + wb),
        nObs: wa + wb, conf: Math.max(a.conf, b.conf), misses: a.misses + b.misses, firstSeen: Math.min(a.firstSeen, b.firstSeen), lastSeen: Math.max(a.lastSeen, b.lastSeen) });
      M.items.splice(j, 1); st.removed.push(b.id);
    }
    if (o.maxAge > 0) M.items = M.items.filter(it => { if (t - it.lastSeen > o.maxAge) { st.expired.push(it.id); return false; } return true; });
    for (const it of M.items) { it.persistence = it.nObs / (it.nObs + it.misses); it.mobile = it.misses >= o.mobileMin && it.persistence < 0.5; }
    return st;
  }
  /** Un « petit obstacle » posé sur un mur de la carte vectorielle est en fait un morceau de mur trop
   *  court pour devenir un segment (trou dans les points) : il est retiré de la carte d'obstacles. */
  function pruneObstaclesOnWalls(M, segs, opt = {}) {
    const { tol = 0.06 } = opt, removed = [];
    M.items = M.items.filter(it => {
      const on = segs.some(f => LC.pointSegDist(it.x, it.y, f) < it.r + tol);
      if (on) removed.push(it.id);
      return !on;
    });
    return { removed };
  }
  function obstaclesToCSV(M) {
    let s = 'id,type,x_m,y_m,rayon_m,confiance,observations,vu_absent,persistance,mobile,premier_scan,dernier_scan\n';
    for (const it of M.items) s += [it.id, it.kind, it.x.toFixed(4), it.y.toFixed(4), it.r.toFixed(4), it.conf, it.nObs, it.misses,
      (it.persistence ?? 1).toFixed(2), it.mobile ? 1 : 0, it.firstSeen, it.lastSeen].join(',') + '\n';
    return s;
  }

  // ==========================================================================
  //  5. Grid map enrichie : couches par cellule
  // ==========================================================================
  // hits : nombre de scans où la cellule a été un impact ; seen : nombre de scans où elle a été
  // observée (libre ou impact) ; last / lastOcc : numéro du dernier scan qui l'a observée / vue occupée.
  // Mémoire : 12 octets de plus par cellule (le compromis mémoire / information du cours).
  function createLayers(g) {
    const n = g.W * g.H;
    return { W: g.W, H: g.H, hits: new Uint16Array(n), seen: new Uint16Array(n), last: new Int32Array(n).fill(-1), lastOcc: new Int32Array(n).fill(-1), t: 0 };
  }
  function updateGridLayered(g, L, robot, pts, p = {}) {
    const t = ++L.t;
    return LC.updateGrid(g, robot, pts, Object.assign({}, p, {
      onCells(F, O) {
        for (const k of F) { if (L.seen[k] < 65535) L.seen[k]++; L.last[k] = t; }
        for (const k of O) { if (L.hits[k] < 65535) L.hits[k]++; if (L.seen[k] < 65535) L.seen[k]++; L.last[k] = t; L.lastOcc[k] = t; }
      }
    }));
  }
  const cloneGrid = g => Object.assign({}, g, { data: g.data.slice() });
  const cloneLayers = L => ({ W: L.W, H: L.H, hits: L.hits.slice(), seen: L.seen.slice(), last: L.last.slice(), lastOcc: L.lastOcc.slice(), t: L.t });

  /** Carte couvrant tous les scans ; data : [{ robot, pts }] (repère carte) */
  function buildLayeredGrid(data, opt = {}) {
    const o = Object.assign({ res: 0.05, margin: 0.5, fillBetween: true }, opt);
    let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
    for (const { robot, pts } of data) for (const q of [robot, ...pts]) { if (q.x < x0) x0 = q.x; if (q.x > x1) x1 = q.x; if (q.y < y0) y0 = q.y; if (q.y > y1) y1 = q.y; }
    const g = LC.createGrid({ xMin: x0 - o.margin, yMin: y0 - o.margin, xMax: x1 + o.margin, yMax: y1 + o.margin, res: o.res });
    const L = createLayers(g);
    for (const { robot, pts } of data) updateGridLayered(g, L, robot, pts, o);
    return { grid: g, layers: L };
  }

  // ==========================================================================
  //  6. Nettoyage de la grid map (document « grid map » §5)
  // ==========================================================================
  /** Décroissance temporelle : cellules non observées depuis plus de `age` scans -> rapprochées de « inconnu » de `step` */
  function decayGrid(g, L, opt = {}) {
    const { age = 10, step = 16 } = opt; let n = 0;
    for (let k = 0; k < g.data.length; k++) {
      if (L.last[k] < 0 || L.t - L.last[k] <= age) continue;
      const v = g.data[k];
      if (v > UNKNOWN) { g.data[k] = Math.max(UNKNOWN, v - step); n++; } else if (v < UNKNOWN) { g.data[k] = Math.min(UNKNOWN, v + step); n++; }
    }
    return { changed: n };
  }
  /** Obstacles obsolètes : cellule occupée qui n'a plus été un impact depuis `age` scans.
   *  Revue libre depuis -> libre ; jamais réobservée -> inconnue. */
  function clearObsolete(g, L, opt = {}) {
    const { age = 10 } = opt; let freed = 0, unknown = 0;
    for (let k = 0; k < g.data.length; k++) {
      if (g.data[k] < OCC_T || L.lastOcc[k] < 0 || L.t - L.lastOcc[k] <= age) continue;
      if (L.last[k] > L.lastOcc[k]) { g.data[k] = FREE_T - 25; freed++; } else { g.data[k] = UNKNOWN; unknown++; }
    }
    return { freed, unknown };
  }
  /** Obstacles mobiles (probabilité de persistance) : cellule observée au moins minSeen fois, mais
   *  impact dans moins de maxPersist de ces observations (une personne, une chaise déplacée, une porte).
   *  Elle est rendue libre ; le masque `mobile` les garde pour l'affichage. */
  //  Une cellule vue comme impact dans au moins keepHits scans est gardée : un objet fin (pied de chaise
  //  de 2 cm dans une cellule de 5 cm) est souvent « traversé » par les faisceaux venus d'ailleurs.
  function removeMobile(g, L, opt = {}) {
    const { minSeen = 3, maxPersist = 0.34, keepHits = 3 } = opt, mobile = new Uint8Array(g.data.length);
    let n = 0, wasOcc = 0, wasUnk = 0;
    for (let k = 0; k < g.data.length; k++) {
      const h = L.hits[k], s = L.seen[k];
      if (!h || h >= keepHits || s < minSeen || h / s > maxPersist) continue;
      mobile[k] = 1; n++;
      const v = g.data[k];
      if (v >= OCC_T) wasOcc++; else if (v > FREE_T) wasUnk++;
      if (v > FREE_T - 25) g.data[k] = FREE_T - 25;
    }
    return { removed: n, wasOcc, wasUnk, mobile };
  }
  /** Étiquetage des composantes connexes (8-connexité) des cellules occupées */
  function labelComponents(g, opt = {}) {
    const occT = opt.occT ?? OCC_T, W = g.W, H = g.H, lab = new Int32Array(W * H).fill(-1), comps = [];
    const stack = [];
    for (let k0 = 0; k0 < W * H; k0++) {
      if (lab[k0] >= 0 || g.data[k0] < occT) continue;
      const id = comps.length, cells = []; lab[k0] = id; stack.push(k0);
      while (stack.length) {
        const k = stack.pop(); cells.push(k);
        const i = k % W, j = (k - i) / W;
        for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
          if (!di && !dj) continue;
          const a = i + di, b = j + dj; if (a < 0 || b < 0 || a >= W || b >= H) continue;
          const m = b * W + a; if (lab[m] >= 0 || g.data[m] < occT) continue;
          lab[m] = id; stack.push(m);
        }
      }
      comps.push({ id, cells });
    }
    return { labels: lab, comps };
  }
  /** Petites taches isolées et peu confirmées (bruit) -> inconnues. Une tache isolée mais vue souvent
   *  (pied de chaise, poteau) est une cellule d'intérêt : elle est gardée. */
  function removeSpeckles(g, L, opt = {}) {
    const { maxCells = 2, minHits = 2 } = opt, { comps } = labelComponents(g); let n = 0, kept = 0;
    for (const c of comps) {
      if (c.cells.length > maxCells) continue;
      const support = L ? Math.max(...c.cells.map(k => L.hits[k])) : 0;
      if (L && support >= minHits) { kept++; continue; }
      for (const k of c.cells) g.data[k] = UNKNOWN; n += c.cells.length;
    }
    return { removed: n, keptInterest: kept };
  }
  /** Lissage spatial. 'majority' : une cellule inconnue prend l'état majoritaire de ses 8 voisines
   *  si au moins minVotes sont d'accord (trous et stries comblés, murs fins préservés).
   *  'mean' : moyenne des voisines dans un rayon, sauf pour les cellules occupées (sinon les murs
   *  d'une cellule d'épaisseur disparaîtraient). */
  function smoothGrid(g, opt = {}) {
    const { mode = 'majority', radius = 1, minVotes = 5 } = opt, W = g.W, H = g.H, src = g.data.slice(); let n = 0;
    for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
      const k = j * W + i, v = src[k];
      if (mode === 'majority') {
        if (v <= FREE_T || v >= OCC_T) continue;
        let f = 0, o = 0;
        for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
          if (!di && !dj) continue;
          const a = i + di, b = j + dj; if (a < 0 || b < 0 || a >= W || b >= H) continue;
          const w = src[b * W + a]; if (w <= FREE_T) f++; else if (w >= OCC_T) o++;
        }
        if (f >= minVotes) { g.data[k] = FREE_T - 5; n++; } else if (o >= minVotes) { g.data[k] = OCC_T + 5; n++; }
      } else {
        if (v >= OCC_T) continue;
        let s = 0, c = 0;
        for (let dj = -radius; dj <= radius; dj++) for (let di = -radius; di <= radius; di++) {
          const a = i + di, b = j + dj; if (a < 0 || b < 0 || a >= W || b >= H) continue;
          s += src[b * W + a]; c++;
        }
        const m = Math.round(s / c); if (m !== v) { g.data[k] = Math.min(OCC_T - 1, m); n++; }
      }
    }
    return { changed: n };
  }
  /** Fermeture morphologique des obstacles (dilatation puis érosion, carré de rayon r) :
   *  bouche les trous d'une ou deux cellules dans les murs. Les cellules franchement libres
   *  (valeur < protectFree, traversées plusieurs fois) ne sont jamais bouchées. */
  function closeGaps(g, opt = {}) {
    const { radius = 1, protectFree = 30 } = opt, W = g.W, H = g.H, occ = new Uint8Array(W * H);
    for (let k = 0; k < W * H; k++) occ[k] = g.data[k] >= OCC_T ? 1 : 0;
    const morph = (src, dil) => {
      const out = new Uint8Array(W * H);
      for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
        let v = dil ? 0 : 1;
        for (let dj = -radius; dj <= radius && (dil ? !v : v); dj++) for (let di = -radius; di <= radius; di++) {
          const a = i + di, b = j + dj, x = a < 0 || b < 0 || a >= W || b >= H ? 0 : src[b * W + a];
          if (dil && x) { v = 1; break; } if (!dil && !x) { v = 0; break; }
        }
        out[j * W + i] = v;
      }
      return out;
    };
    const closed = morph(morph(occ, true), false); let n = 0;
    for (let k = 0; k < W * H; k++) if (closed[k] && !occ[k] && g.data[k] >= protectFree) { g.data[k] = OCC_T + 30; n++; }
    return { filled: n };
  }

  // ==========================================================================
  //  7. Obstacles de la grid map : composantes, mesures, classes
  // ==========================================================================
  const OBST_DEFAULTS = { smallMax: 0.15, mediumMax: 1.0 };
  /** Chaque composante connexe d'obstacle -> { id, cells, area (m²), cx, cy, len, wid (m, ACP), angle, cls, support }.
   *  Classes par encombrement : 'petit' (pied de chaise, poteau), 'objet' (chaise, sac, poubelle),
   *  'structure' (mur, meuble). Le lidar ne voit que le contour des objets : on classe donc sur la taille. */
  function gridObstacles(g, L, opt = {}) {
    const o = Object.assign({}, OBST_DEFAULTS, opt), { labels, comps } = labelComponents(g), out = [];
    for (const c of comps) {
      let sx = 0, sy = 0; const P = c.cells.map(k => LC.cellCenter(g, k % g.W, Math.floor(k / g.W)));
      for (const [x, y] of P) { sx += x; sy += y; }
      const cx = sx / P.length, cy = sy / P.length;
      let a = 0, b = 0, d = 0; for (const [x, y] of P) { a += (x - cx) ** 2; b += (y - cy) ** 2; d += (x - cx) * (y - cy); }
      a /= P.length; b /= P.length; d /= P.length;
      const tr = (a + b) / 2, df = Math.sqrt(((a - b) / 2) ** 2 + d * d), l1 = tr + df, l2 = Math.max(0, tr - df);
      const len = Math.sqrt(12 * l1) + g.res, wid = Math.sqrt(12 * l2) + g.res, ext = Math.max(len, wid);
      const cls = ext <= o.smallMax ? 'petit' : ext <= o.mediumMax ? 'objet' : 'structure';
      const support = L ? Math.max(...c.cells.map(k => L.hits[k])) : 0;
      out.push({ id: c.id, cells: c.cells.length, area: c.cells.length * g.res * g.res, cx, cy, len, wid,
                 angle: 0.5 * Math.atan2(2 * d, a - b) * R2D, cls, support });
    }
    return { labels, obstacles: out };
  }

  // ==========================================================================
  //  8. Carte vectorielle : régularisation et nettoyage géométrique
  // ==========================================================================
  /** Direction dominante (mod 90°) pondérée par la longueur : dans une salle, les murs sont parallèles ou perpendiculaires */
  function dominantDirection(segs) {
    const h = new Float64Array(90);
    for (const s of segs) h[Math.floor(mod(LC.segAngle(s) * R2D, 90)) % 90] += LC.segLen(s);
    let best = 0, bv = -1;
    for (let b = 0; b < 90; b++) { let v = 0; for (let d = -2; d <= 2; d++) v += h[mod(b + d, 90)]; if (v > bv) { bv = v; best = b; } }
    // moyenne circulaire (période 90° -> angle × 4) autour du pic
    let cx = 0, cy = 0;
    for (const s of segs) {
      const a = mod(LC.segAngle(s) * R2D, 90);
      if (Math.abs(normDeg((a - best - 0.5) * 4) / 4) > 8) continue;
      const w = LC.segLen(s); cx += w * Math.cos(a * 4 * D2R); cy += w * Math.sin(a * 4 * D2R);
    }
    return cx || cy ? mod(Math.atan2(cy, cx) * R2D / 4, 90) : best + 0.5;
  }
  const rotateAbout = (s, mx, my, dth) => {
    const c = Math.cos(dth), sn = Math.sin(dth), r = (x, y) => [mx + c * (x - mx) - sn * (y - my), my + sn * (x - mx) + c * (y - my)];
    const [x1, y1] = r(s.x1, s.y1), [x2, y2] = r(s.x2, s.y2); return Object.assign({}, s, { x1, y1, x2, y2 });
  };
  /** Redresse les segments à moins de angTol d'une direction dominante (rotation autour de leur milieu) */
  function snapDirections(segs, opt = {}) {
    const theta0 = opt.theta0 ?? dominantDirection(segs), angTol = opt.angTol ?? 6; let n = 0;
    const out = segs.map(s => {
      const a = LC.segAngle(s) * R2D;
      let bestD = Infinity;
      for (const t of [theta0, theta0 + 90]) { const d = normDeg(2 * (t - a)) / 2; if (Math.abs(d) < Math.abs(bestD)) bestD = d; }
      if (Math.abs(bestD) > angTol || Math.abs(bestD) < 1e-9) return s;
      n++; return rotateAbout(s, (s.x1 + s.x2) / 2, (s.y1 + s.y2) / 2, bestD * D2R);
    });
    return { segments: out, snapped: n, theta0 };
  }
  /** Segments parallèles (à angTol près) dont les droites sont à moins de distTol : ramenés sur une
   *  même droite (décalage moyen pondéré par la longueur). Un mur vu par plusieurs scans devient droit. */
  function alignCollinear(segs, opt = {}) {
    const { angTol = 1.5, distTol = 0.06 } = opt, out = segs.map(s => Object.assign({}, s)), used = new Uint8Array(out.length); let n = 0;
    for (let i = 0; i < out.length; i++) {
      if (used[i]) continue;
      const a = LC.segAngle(out[i]), nx = -Math.sin(a), ny = Math.cos(a), off = s => nx * (s.x1 + s.x2) / 2 + ny * (s.y1 + s.y2) / 2;
      const grp = [i]; used[i] = 1;
      for (let j = i + 1; j < out.length; j++) {
        if (used[j]) continue;
        if (LC.angleBetween(out[i], out[j]).lines > angTol) continue;
        if (Math.abs(off(out[j]) - off(out[i])) > distTol) continue;
        grp.push(j); used[j] = 1;
      }
      if (grp.length < 2) continue;
      let sw = 0, so = 0; for (const k of grp) { const w = LC.segLen(out[k]) * (out[k].conf ?? 1); sw += w; so += w * off(out[k]); }
      const target = so / sw;
      for (const k of grp) {
        const s = out[k], d = target - off(s);
        // on garde l'orientation propre de chaque segment ; seule la position latérale change
        Object.assign(s, { x1: s.x1 + nx * d, y1: s.y1 + ny * d, x2: s.x2 + nx * d, y2: s.y2 + ny * d });
      }
      n += grp.length;
    }
    return { segments: out, aligned: n };
  }
  /** Fusion des segments proches en gardant les propriétés (confiance max, observations cumulées) */
  function mergeKeepProps(segs, tol) {
    let list = segs.map(s => Object.assign({}, s)), n = 0, changed = true;
    while (changed) {
      changed = false;
      outer: for (let i = 0; i < list.length; i++) for (let j = i + 1; j < list.length; j++) {
        const a = list[i], b = list[j];
        if (!LC.areClose(a, b, tol).close) continue;
        const g = LC.mergeSegments(a, b, LC.segLen(a) * (a.conf ?? 1), LC.segLen(b) * (b.conf ?? 1));
        list[i] = Object.assign({}, a, { x1: g.x1, y1: g.y1, x2: g.x2, y2: g.y2, conf: Math.max(a.conf ?? 1, b.conf ?? 1),
          nObs: (a.nObs ?? 1) + (b.nObs ?? 1), firstSeen: Math.min(a.firstSeen ?? 0, b.firstSeen ?? 0), lastSeen: Math.max(a.lastSeen ?? 0, b.lastSeen ?? 0) });
        list.splice(j, 1); n++; changed = true; break outer;
      }
    }
    return { segments: list, merged: n };
  }
  /** Coins propres : deux segments à plus de minAng dont l'intersection des droites est à moins de
   *  maxExt d'une extrémité -> cette extrémité est déplacée sur l'intersection (prolongement ou
   *  rognage). Si l'intersection tombe dans l'autre segment : jonction en T, seul le premier bouge. */
  function joinCorners(segs, opt = {}) {
    const { maxExt = 0.25, minAng = 60 } = opt, out = segs.map(s => Object.assign({}, s)); let n = 0;
    for (let i = 0; i < out.length; i++) for (let j = i + 1; j < out.length; j++) {
      const a = out[i], b = out[j];
      if (LC.angleBetween(a, b).lines < minAng) continue;
      const r = LC.intersect(a, b); if (r.type !== 'point' && r.type !== 'lines') continue;
      const end = s => { const d1 = Math.hypot(s.x1 - r.x, s.y1 - r.y), d2 = Math.hypot(s.x2 - r.x, s.y2 - r.y); return d1 <= d2 ? [1, d1] : [2, d2]; };
      const [ea, da] = end(a), [eb, db] = end(b);
      const tIn = (s, tt) => tt >= -1e-9 && tt <= 1 + 1e-9;
      const aNear = da <= maxExt, bNear = db <= maxExt;
      if (!aNear && !bNear) continue;
      // les deux bouts proches : vrai coin ; un seul : jonction en T si l'intersection est dans l'autre
      if (aNear && bNear) {
        if (da > 1e-9) { a['x' + ea] = r.x; a['y' + ea] = r.y; n++; }
        if (db > 1e-9) { b['x' + eb] = r.x; b['y' + eb] = r.y; n++; }
      } else if (aNear && tIn(b, r.u) && da > 1e-9) { a['x' + ea] = r.x; a['y' + ea] = r.y; n++; }
      else if (bNear && tIn(a, r.t) && db > 1e-9) { b['x' + eb] = r.x; b['y' + eb] = r.y; n++; }
    }
    return { segments: out, joined: n };
  }
  /** Collisions géométriques : deux segments qui se croisent avec un petit dépassement (< overhang)
   *  -> le dépassement est rogné (artefact d'extraction, deux murs ne se traversent pas). */
  function trimCollisions(segs, opt = {}) {
    const { overhang = 0.12, minAng = 20 } = opt, out = segs.map(s => Object.assign({}, s)); let n = 0;
    for (let i = 0; i < out.length; i++) for (let j = 0; j < out.length; j++) {
      if (i === j) continue;
      const a = out[i], b = out[j];
      if (LC.angleBetween(a, b).lines < minAng) continue;
      const r = LC.intersect(a, b); if (r.type !== 'point') continue;
      const L = LC.segLen(a), d1 = r.t * L, d2 = (1 - r.t) * L;
      if (d1 > 1e-6 && d1 < overhang && d2 > overhang) { a.x1 = r.x; a.y1 = r.y; n++; }
      else if (d2 > 1e-6 && d2 < overhang && d1 > overhang) { a.x2 = r.x; a.y2 = r.y; n++; }
    }
    return { segments: out, trimmed: n };
  }
  const REG_DEFAULTS = { minConf: 1, minLen: 0.30, snapTol: 6, alignTol: 0.06, merge: { angTol: 3, distTol: 0.06, gapTol: 0.30 },
                         maxExt: 0.25, cornerMin: 60, overhang: 0.12, orthogonal: true };
  /** Chaîne de régularisation d'une carte vectorielle -> nouvelle carte + compte rendu */
  function regularizeVectorMap(map, opt = {}) {
    const o = Object.assign({}, REG_DEFAULTS, opt), rep = { before: map.features.length };
    let S = map.features.filter(f => (f.conf ?? 1) >= o.minConf && LC.segLen(f) >= o.minLen).map(f => Object.assign({}, f));
    rep.weak = rep.before - S.length;
    if (o.orthogonal && S.length) { const r = snapDirections(S, { angTol: o.snapTol }); S = r.segments; rep.snapped = r.snapped; rep.theta0 = r.theta0; }
    { const r = alignCollinear(S, { distTol: o.alignTol }); S = r.segments; rep.aligned = r.aligned; }
    { const r = mergeKeepProps(S, o.merge); S = r.segments; rep.merged = r.merged; }
    { const r = joinCorners(S, { maxExt: o.maxExt, minAng: o.cornerMin }); S = r.segments; rep.joined = r.joined; }
    { const r = trimCollisions(S, { overhang: o.overhang }); S = r.segments; rep.trimmed = r.trimmed; }
    S = S.filter(f => LC.segLen(f) >= o.minLen);
    let id = 1; S.forEach(f => { f.id = id++; });
    const out = { features: S, corners: LC.computeCorners(S, LC.VM_DEFAULTS), scans: map.scans, nextId: id };
    rep.after = S.length; rep.corners = out.corners.length;
    rep.lenBefore = map.features.reduce((a, f) => a + LC.segLen(f), 0); rep.lenAfter = S.reduce((a, f) => a + LC.segLen(f), 0);
    return { map: out, report: rep };
  }

  // ==========================================================================
  //  9. Exploitation : carte de coût et trajet
  // ==========================================================================
  // Valeurs (convention costmap ROS) : 0 libre, 1..252 proche d'un obstacle, 253 « inscrit » (le centre
  // du robot à cette place = collision), 254 obstacle, 255 inconnu.
  const COST = { LETHAL: 254, INSCRIBED: 253, UNKNOWN: 255 };
  function costmap(g, opt = {}) {
    const o = Object.assign({ robotRadius: 0.15, inflation: 0.35, decay: 3 }, opt);
    const F = LL.distanceField(g), n = g.W * g.H, cost = new Uint8Array(n);
    let free = 0, inscribed = 0;
    for (let k = 0; k < n; k++) {
      const v = g.data[k], d = F.dist[k];
      if (v >= OCC_T) { cost[k] = COST.LETHAL; continue; }
      if (d <= o.robotRadius) { cost[k] = COST.INSCRIBED; inscribed++; continue; }
      if (v > FREE_T) { cost[k] = COST.UNKNOWN; continue; }
      cost[k] = d <= o.robotRadius + o.inflation ? Math.max(1, Math.round(252 * Math.exp(-o.decay * (d - o.robotRadius) / o.inflation))) : 0;
      free++;
    }
    return { W: g.W, H: g.H, res: g.res, x0: g.x0, y0: g.y0, cost, dist: F.dist, robotRadius: o.robotRadius,
             stats: { navigable: free * g.res * g.res, inscribed: inscribed * g.res * g.res } };
  }
  function costmapToPGM(cm, name = 'costmap') {
    const header = `P5\n# ${name} res=${cm.res} 0=libre 254=obstacle 253=inscrit 255=inconnu\n${cm.W} ${cm.H}\n255\n`;
    const out = new Uint8Array(header.length + cm.W * cm.H);
    for (let k = 0; k < header.length; k++) out[k] = header.charCodeAt(k);
    let p = header.length;
    for (let j = cm.H - 1; j >= 0; j--) for (let i = 0; i < cm.W; i++) out[p++] = cm.cost[j * cm.W + i];
    return out;
  }
  /**
   * A* 8-connexe sur la carte de coût : cellules ≥ 253 interdites (et inconnues sauf allowUnknown),
   * coût d'un pas = longueur × (1 + w·coût/252) : le trajet s'écarte des obstacles quand c'est possible.
   * Puis lissage : on saute les points intermédiaires tant que la ligne droite reste sans collision.
   * Renvoie { ok, path: [{x,y}], raw, length, expanded } (raison si échec).
   */
  function planPath(cm, start, goal, opt = {}) {
    const o = Object.assign({ w: 3, allowUnknown: false, maxExpand: 2e6 }, opt), W = cm.W, H = cm.H;
    const toCell = p => [Math.floor((p.x - cm.x0) / cm.res), Math.floor((p.y - cm.y0) / cm.res)];
    const blocked = k => cm.cost[k] === COST.LETHAL || cm.cost[k] === COST.INSCRIBED || (!o.allowUnknown && cm.cost[k] === COST.UNKNOWN);
    const [si, sj] = toCell(start), [gi, gj] = toCell(goal);
    const inside = (i, j) => i >= 0 && j >= 0 && i < W && j < H;
    if (!inside(si, sj) || !inside(gi, gj)) return { ok: false, reason: 'départ ou arrivée hors carte' };
    const S = sj * W + si, G = gj * W + gi;
    if (blocked(G)) {
      if (cm.cost[G] === COST.LETHAL) return { ok: false, reason: 'arrivée sur un obstacle' };
      if (cm.cost[G] === COST.UNKNOWN) return { ok: false, reason: 'arrivée en zone inconnue (cocher « autoriser les zones inconnues »)' };
      const d = cm.dist ? cm.dist[G] : 0;
      return { ok: false, reason: `arrivée à ${(d * 100).toFixed(0)} cm d'un obstacle, moins que le rayon du robot (${(cm.robotRadius * 100).toFixed(0)} cm)` };
    }
    const gs = new Float64Array(W * H).fill(Infinity), from = new Int32Array(W * H).fill(-1), closed = new Uint8Array(W * H);
    const h = k => { const i = k % W, j = (k - i) / W, dx = Math.abs(i - gi), dy = Math.abs(j - gj); return Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy); };
    // tas binaire
    const heap = [], push = (k, f) => { heap.push([f, k]); let c = heap.length - 1; while (c) { const p = (c - 1) >> 1; if (heap[p][0] <= heap[c][0]) break; [heap[p], heap[c]] = [heap[c], heap[p]]; c = p; } };
    const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let c = 0; for (;;) { const l = 2 * c + 1, r = l + 1; let m = c; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === c) break; [heap[m], heap[c]] = [heap[c], heap[m]]; c = m; } } return top; };
    gs[S] = 0; push(S, h(S)); let expanded = 0;
    while (heap.length) {
      const [, k] = pop(); if (closed[k]) continue; closed[k] = 1;
      if (k === G) break;
      if (++expanded > o.maxExpand) return { ok: false, reason: 'recherche trop longue' };
      const i = k % W, j = (k - i) / W;
      for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
        if (!di && !dj) continue;
        const a = i + di, b = j + dj; if (!inside(a, b)) continue;
        const m = b * W + a; if (closed[m] || (blocked(m) && m !== S)) continue;
        if (di && dj && (blocked(j * W + a) || blocked(b * W + i))) continue;      // pas de coupe en diagonale entre deux obstacles
        const c = (di && dj ? Math.SQRT2 : 1) * (1 + o.w * (cm.cost[m] === COST.UNKNOWN ? 1 : cm.cost[m] / 252));
        if (gs[k] + c < gs[m]) { gs[m] = gs[k] + c; from[m] = k; push(m, gs[m] + h(m)); }
      }
    }
    if (from[G] < 0 && G !== S) return { ok: false, reason: 'aucun passage assez large pour le robot', expanded };
    const cells = []; for (let k = G; k >= 0; k = k === S ? -1 : from[k]) cells.push(k);
    cells.reverse();
    const ctr = k => { const i = k % W, j = (k - i) / W; return { x: cm.x0 + (i + 0.5) * cm.res, y: cm.y0 + (j + 0.5) * cm.res, i, j }; };
    const raw = cells.map(ctr);
    // lissage par visibilité (ligne droite sans cellule bloquée ni trop coûteuse)
    const clear = (p, q) => { let ok = true; LC.bresenham(p.i, p.j, q.i, q.j, (a, b) => { const c = cm.cost[b * W + a]; if (c >= COST.INSCRIBED || c > 128) ok = false; }); return ok; };
    const path = [raw[0]]; let a = 0;
    while (a < raw.length - 1) { let b = raw.length - 1; while (b > a + 1 && !clear(raw[a], raw[b])) b--; path.push(raw[b]); a = b; }
    path[0] = { x: start.x, y: start.y, i: si, j: sj }; path[path.length - 1] = { x: goal.x, y: goal.y, i: gi, j: gj };
    let length = 0; for (let k = 1; k < path.length; k++) length += Math.hypot(path[k].x - path[k - 1].x, path[k].y - path[k - 1].y);
    return { ok: true, path, raw, length, expanded };
  }

  // ==========================================================================
  //  10. Qualité d'une carte
  // ==========================================================================
  /**
   *  - points sur obstacle : part des points des scans (avec leur pose) à moins de 1,5 cellule
   *    d'une cellule occupée -> cohérence de la superposition ;
   *  - distance moyenne des points à l'obstacle le plus proche (plafonnée à 50 cm) ;
   *  - netteté : part des cellules observées dont l'état est tranché (libre ou occupé) ;
   *  - conflits : cellules vues souvent, tantôt libres tantôt occupées (persistance entre 25 et 75 %).
   */
  function mapQuality(g, data, L) {
    const F = LL.distanceField(g); let n = 0, on = 0, sd = 0;
    for (const { pts } of data) for (let k = 0; k < pts.length; k += 2) {
      const d = Math.min(0.5, LL.fieldSample(F, pts[k].x, pts[k].y)[0]); n++; sd += d; if (d <= 1.5 * g.res) on++;
    }
    const st = LC.gridStats(g); let observed = 0, conflicts = 0;
    for (let k = 0; k < g.data.length; k++) {
      if (L ? L.seen[k] > 0 : g.data[k] !== UNKNOWN) observed++;
      if (L && L.seen[k] >= 3 && L.hits[k]) { const p = L.hits[k] / L.seen[k]; if (p > 0.25 && p < 0.75) conflicts++; }
    }
    return { pointsOnObstacle: n ? on / n : 0, meanDist: n ? sd / n : 0, occCells: st.occ, freeArea: st.free * g.res * g.res,
             unknownPct: st.unk / st.cells, sharpness: observed ? (st.free + st.occ) / observed : 0, conflicts };
  }

  // ==========================================================================
  //  11. Recalage global des poses (alignement de tous les scans entre eux)
  // ==========================================================================
  /**
   * Correction géométrique (document « cartographie » §6 et §9) : chaque scan est relocalisé sur la
   * carte construite avec tous les AUTRES scans (multi-résolution sur la grid map, puis ICP
   * point-segment sur la carte vectorielle, méthodes du Jour 2), en partant de sa pose actuelle.
   * Les poses sont mises à jour au fur et à mesure (Gauss-Seidel) et on refait passes fois.
   * Garde-fous : correction refusée si elle dépasse maxCorr / maxRot, ou si le score diminue.
   * scans : [{ name, a, d, i, pose }]. onProgress(fraction, texte) peut être async.
   */
  async function refinePoses(scans, opt = {}, onProgress) {
    const o = Object.assign({ passes: 4, rxy: 0.3, rth: 8, maxCorr: 0.30, maxRot: 8, growing: true, stopPos: 0.005, stopAng: 0.1 }, opt);
    const poses = scans.map(s => Object.assign({}, s.pose)), log = [], N = scans.length;
    const pts = scans.map(s => LL.uniformSubsample(LL.localPoints(s)));
    const total = o.passes * N; let done = 0;
    // un essai : relocaliser le scan k sur la carte des scans `refIdx` (poses actuelles)
    const attempt = (k, refIdx, pass) => {
      const R = LL.buildReferences(refIdx.map(j => Object.assign({}, scans[j], { pose: poses[j] })), -1);
      const P = pts[k], before = LL.vecScore(R.index, P, poses[k].x, poses[k].y, poses[k].th, 0.05);
      const r1 = LL.gridMultiRes(R.field, P, poses[k], { rxy: o.rxy, rth: o.rth }), r2 = LL.vecICP(R.index, P, r1);
      const after = LL.vecScore(R.index, P, r2.x, r2.y, r2.th, 0.05), e = LL.poseError(r2, poses[k]);
      const ok = e.pos <= o.maxCorr && e.ang <= o.maxRot && after >= before - 1e-3;
      log.push({ pass, name: scans[k].name, refs: refIdx.length, dx: r2.x - poses[k].x, dy: r2.y - poses[k].y, dth: normDeg(r2.th - poses[k].th), before, after, accepted: ok });
      if (ok) poses[k] = { x: r2.x, y: r2.y, th: normDeg(r2.th) };
      return ok ? e : { pos: 0, ang: 0 };
    };
    for (let pass = 1; pass <= o.passes; pass++) {
      let maxPos = 0, maxAng = 0;
      // passe 1 « progressive » : le scan 1 fixe le repère, les autres sont recalés du plus proche au plus
      // loin, chacun sur les scans DÉJÀ recalés (référence cohérente) ; passes suivantes : sur tous les autres
      const order = [...Array(N).keys()].slice(1);
      if (pass === 1 && o.growing) order.sort((a, b) => Math.hypot(poses[a].x - poses[0].x, poses[a].y - poses[0].y) - Math.hypot(poses[b].x - poses[0].x, poses[b].y - poses[0].y));
      const doneSet = [0];
      for (const k of order) {
        if (onProgress) await onProgress(done / total, `passe ${pass} · ${scans[k].name || 'scan ' + (k + 1)}`);
        done++;
        const refIdx = pass === 1 && o.growing ? doneSet.slice() : [...Array(N).keys()].filter(j => j !== k);
        const e = attempt(k, refIdx, pass);
        maxPos = Math.max(maxPos, e.pos); maxAng = Math.max(maxAng, e.ang);
        doneSet.push(k);
      }
      if (pass > 1 && maxPos < o.stopPos && maxAng < o.stopAng) { done = total; break; }   // convergé
    }
    if (onProgress) await onProgress(1, 'terminé');
    return { poses, log };
  }

  // ==========================================================================
  //  11 bis. Édition : gomme et filtres « distance minimum »
  // ==========================================================================
  // Les fonctions travaillent sur un résultat R = { grid, vmap, omap } (celui de processMap).
  // fill : valeur donnée aux cellules effacées, 'free' (libre) ou 'unknown' (inconnue).
  const fillValue = fill => fill === 'unknown' ? UNKNOWN : FREE_T - 25;
  /** Parties d'un segment hors du disque (centre cx, cy, rayon r) : [] si entièrement dedans, [s] si non touché */
  function clipSegmentDisc(s, cx, cy, r) {
    const dx = s.x2 - s.x1, dy = s.y2 - s.y1, a = dx * dx + dy * dy, fx = s.x1 - cx, fy = s.y1 - cy;
    if (a < 1e-12) return Math.hypot(fx, fy) < r ? [] : [s];
    const b = 2 * (fx * dx + fy * dy), c = fx * fx + fy * fy - r * r, disc = b * b - 4 * a * c;
    if (disc <= 0) return [s];
    const sq = Math.sqrt(disc), t1 = (-b - sq) / (2 * a), t2 = (-b + sq) / (2 * a);
    if (t2 <= 0 || t1 >= 1) return [s];
    if ((Math.min(t2, 1) - Math.max(t1, 0)) * Math.sqrt(a) < 1e-6) return [s];     // le disque ne fait qu'effleurer le segment
    const at = t => [s.x1 + dx * t, s.y1 + dy * t], out = [];
    if (t1 > 0) { const [x, y] = at(t1); out.push(Object.assign({}, s, { x2: x, y2: y })); }
    if (t2 < 1) { const [x, y] = at(t2); out.push(Object.assign({}, s, { x1: x, y1: y })); }
    return out;
  }
  /**
   * Gomme : efface dans un disque (x, y, r) les cellules non libres de la grille, les parties de
   * segments (un segment coupé en deux garde ses propriétés) et les cercles qui touchent le disque.
   * opt : { grid, segments, circles (booléens), fill, minPiece (m, morceaux plus courts supprimés) }
   */
  function eraseDisc(R, x, y, r, opt = {}) {
    const o = Object.assign({ grid: true, segments: true, circles: true, fill: 'free', minPiece: 0.05 }, opt), st = { cells: 0, segCut: 0, segRemoved: 0, circles: 0, cutIds: [] };
    if (o.grid && R.grid) {
      const g = R.grid, v = fillValue(o.fill);
      const i0 = Math.max(0, Math.floor((x - r - g.x0) / g.res)), i1 = Math.min(g.W - 1, Math.floor((x + r - g.x0) / g.res));
      const j0 = Math.max(0, Math.floor((y - r - g.y0) / g.res)), j1 = Math.min(g.H - 1, Math.floor((y + r - g.y0) / g.res));
      for (let j = j0; j <= j1; j++) for (let i = i0; i <= i1; i++) {
        const [cx, cy] = LC.cellCenter(g, i, j); if (Math.hypot(cx - x, cy - y) > r) continue;
        const k = j * g.W + i;
        if (g.data[k] > FREE_T && g.data[k] !== v) { g.data[k] = v; st.cells++; }
      }
    }
    if (o.segments && R.vmap) {
      const out = [];
      for (const f of R.vmap.features) {
        const P = clipSegmentDisc(f, x, y, r);
        if (P.length === 1 && P[0] === f) { out.push(f); continue; }
        const keep = P.filter(q => LC.segLen(q) >= o.minPiece);
        if (!keep.length) { st.segRemoved++; continue; }
        st.segCut++; st.cutIds.push(f.id);
        keep.forEach((q, n) => out.push(n === 0 ? Object.assign(q, { id: f.id }) : Object.assign(q, { id: R.vmap.nextId++ })));
      }
      R.vmap.features = out;
    }
    if (o.circles && R.omap) {
      const n0 = R.omap.items.length;
      R.omap.items = R.omap.items.filter(it => Math.hypot(it.x - x, it.y - y) >= r + it.r);
      st.circles = n0 - R.omap.items.length;
    }
    return st;
  }
  /** Filtre de taille : segments plus courts que minSegLen, cercles de diamètre < minCircleD,
   *  taches d'obstacle de la grille (composantes connexes) d'étendue < minBlob. 0 = désactivé. */
  function filterSmall(R, opt = {}) {
    const { minSegLen = 0, minCircleD = 0, minBlob = 0, fill = 'free' } = opt, st = { segments: 0, circles: 0, blobs: 0, cells: 0 };
    if (minSegLen > 0 && R.vmap) { const n0 = R.vmap.features.length; R.vmap.features = R.vmap.features.filter(f => LC.segLen(f) >= minSegLen); st.segments = n0 - R.vmap.features.length; }
    if (minCircleD > 0 && R.omap) { const n0 = R.omap.items.length; R.omap.items = R.omap.items.filter(it => 2 * it.r >= minCircleD); st.circles = n0 - R.omap.items.length; }
    if (minBlob > 0 && R.grid) {
      const { labels, obstacles } = gridObstacles(R.grid, null), small = new Set(obstacles.filter(o => Math.max(o.len, o.wid) < minBlob).map(o => o.id)), v = fillValue(fill);
      for (let k = 0; k < labels.length; k++) if (small.has(labels[k])) { R.grid.data[k] = v; st.cells++; }
      st.blobs = small.size;
    }
    return st;
  }
  /** Filtre de distance : tout ce qui est à moins de `radius` d'une des positions du robot (objets
   *  touchant le robot, câbles, jambes de l'opérateur) est effacé, comme avec la gomme. */
  function filterNearPoses(R, poses, radius, opt = {}) {
    const st = { cells: 0, segCut: 0, segRemoved: 0, circles: 0 };
    for (const p of poses) { const r = eraseDisc(R, p.x, p.y, radius, opt); for (const k in st) st[k] += r[k]; }   // cutIds non cumulés
    return st;
  }
  /** Après une édition : obstacles de la grille, coins de la carte vectorielle, carte de coût et qualité recalculés */
  function refreshDerived(R, opt = {}) {
    const o = Object.assign({}, R.opt || {}, opt);
    const { labels, obstacles } = gridObstacles(R.grid, R.layers, o.obstacleOpt);
    R.labels = labels; R.obstacles = obstacles;
    R.vmap.corners = LC.computeCorners(R.vmap.features, LC.VM_DEFAULTS);
    R.cost = costmap(R.grid, o.cost);
    if (R.pre && R.poses) R.quality.clean = mapQuality(R.grid, R.pre.map((p, k) => ({ robot: R.poses[k], pts: p.pts })), R.layers);
    return R;
  }

  // ==========================================================================
  //  12. Chaîne complète
  // ==========================================================================
  const PIPE_DEFAULTS = {
    refine: false, refineOpt: {},
    pre: {}, res: 0.05,
    mobile: true, mobileOpt: {},
    obsolete: false, obsoleteOpt: { age: 10 },
    decay: false, decayOpt: { age: 10, step: 16 },
    speckles: true, speckleOpt: {},
    smooth: true, smoothOpt: {},
    close: true, closeOpt: {},
    features: {}, obstacleOpt: {}, vmOpt: { extract: { minLen: 0.3 } },
    regularize: true, regOpt: {},
    filters: { minSegLen: 0, minCircleD: 0, minBlob: 0, nearPoses: 0, fill: 'free' },
    cost: { robotRadius: 0.15, inflation: 0.35 }
  };
  /**
   * scans : [{ name, a, d, i, pose, scale? }] -> tout ce qu'il faut pour afficher et exporter :
   * { poses, refineLog, pre, gridRaw, grid, layers, mobile, obstacles, labels, omap, vmapRaw, vmap,
   *   regReport, cost, quality: { raw, clean }, steps: [{ step, detail, ms }] }
   */
  async function processMap(scans, opt = {}, onProgress) {
    const o = Object.assign({}, PIPE_DEFAULTS, opt), steps = [], T = () => now();
    const prog = async (f, t) => { if (onProgress) await onProgress(f, t); };
    let t0, poses = scans.map(s => Object.assign({}, s.pose)), refineLog = null;
    if (o.refine && scans.length > 1) {
      t0 = T();
      const r = await refinePoses(scans, o.refineOpt, (f, t) => prog(0.6 * f, 'Recalage des poses : ' + t));
      poses = r.poses; refineLog = r.log;
      const acc = r.log.filter(l => l.accepted && (Math.hypot(l.dx, l.dy) > 0.005 || Math.abs(l.dth) > 0.2));
      steps.push({ step: 'Recalage global des poses', detail: `${acc.length} corrections acceptées sur ${r.log.length} essais`, ms: T() - t0 });
    }
    const base = o.refine ? 0.6 : 0;
    await prog(base, 'Prétraitement des scans');
    t0 = T();
    const pre = scans.map((s, k) => Object.assign({ name: s.name }, preprocessScan(s, poses[k], Object.assign({ scale: s.scale ?? 1 }, o.pre))));
    const data = pre.map((p, k) => ({ robot: { x: poses[k].x, y: poses[k].y }, pts: p.pts }));
    const rm = pre.reduce((a, p) => { for (const k in p.removed) a[k] = (a[k] || 0) + p.removed[k]; return a; }, {});
    const tot = pre.reduce((a, p) => a + p.total, 0);
    steps.push({ step: 'Prétraitement', detail: `${tot - pre.reduce((a, p) => a + p.kept, 0)} points écartés sur ${tot} : hors portée ${rm.range}, intensité ${rm.intensity}, isolés ${rm.isolated}, mixtes ${rm.mixed}`, ms: T() - t0 });

    await prog(base + 0.1 * (1 - base), 'Grid map enrichie');
    t0 = T();
    const { grid: gridRaw, layers } = buildLayeredGrid(data, { res: o.res });
    steps.push({ step: 'Grid map enrichie (impacts, observations, horodatage)', detail: `${gridRaw.W} × ${gridRaw.H} cellules, ${data.length} scans`, ms: T() - t0 });
    const qRaw = mapQuality(gridRaw, data, layers);

    const grid = cloneGrid(gridRaw); let mobile = new Uint8Array(grid.data.length);
    const stepG = (name, fn) => { const t = T(), r = fn(); steps.push({ step: name, detail: r.detail, ms: T() - t }); return r; };
    if (o.decay) stepG('Décroissance temporelle', () => { const r = decayGrid(grid, layers, o.decayOpt); return { detail: `${r.changed} cellules rapprochées de « inconnu »` }; });
    if (o.obsolete) stepG('Obstacles obsolètes', () => { const r = clearObsolete(grid, layers, o.obsoleteOpt); return { detail: `${r.freed} cellules libérées, ${r.unknown} remises à inconnu` }; });
    if (o.mobile) stepG('Obstacles mobiles (persistance)', () => { const r = removeMobile(grid, layers, o.mobileOpt); mobile = r.mobile; return { detail: `${r.removed} cellules « traces » (impact dans moins d'un tiers des observations) rendues libres, dont ${r.wasOcc} encore occupées et ${r.wasUnk} inconnues` }; });
    if (o.speckles) stepG('Taches isolées', () => { const r = removeSpeckles(grid, layers, o.speckleOpt); return { detail: `${r.removed} cellules de bruit retirées, ${r.keptInterest} petits obstacles confirmés gardés` }; });
    if (o.smooth) stepG('Lissage (majoritaire)', () => { const r = smoothGrid(grid, o.smoothOpt); return { detail: `${r.changed} cellules inconnues tranchées` }; });
    if (o.close) stepG('Fermeture des murs', () => { const r = closeGaps(grid, o.closeOpt); return { detail: `${r.filled} trous bouchés` }; });

    await prog(base + 0.45 * (1 - base), 'Features (segments, cercles)');
    t0 = T();
    const vmapRaw = LC.createVectorMap(), omap = createObstacleMap(); let nSeg = 0, nCirc = 0;
    for (const { robot, pts } of data) {
      const f = extractFeatures(pts, robot, o.features); nSeg += f.segments.length; nCirc += f.circles.length;
      LC.updateVectorMap(vmapRaw, robot, pts, Object.assign({}, o.vmOpt, (o.features.method || 'split') !== 'split' ? { segments: f.segments } : {}));
      updateObstacleMap(omap, robot, pts, f.circles, o.omOpt);
    }
    const pr = pruneObstaclesOnWalls(omap, vmapRaw.features, o.pruneOpt);
    steps.push({ step: 'Carte vectorielle + obstacles circulaires', detail: `${vmapRaw.features.length} segments ; ${nCirc} cercles observés -> ${omap.items.length} obstacles circulaires suivis (${omap.items.filter(i => i.conf >= 3).length} de confiance ≥ 3, ${omap.items.filter(i => i.mobile).length} mobiles), ${pr.removed.length} écartés car posés sur un mur`, ms: T() - t0 });

    let vmap = vmapRaw, regReport = null;
    if (o.regularize) {
      t0 = T();
      const r = regularizeVectorMap(vmapRaw, o.regOpt); vmap = r.map; regReport = r.report;
      steps.push({ step: 'Régularisation de la carte vectorielle', detail: `${r.report.before} → ${r.report.after} segments ; ${r.report.snapped ?? 0} redressés (axes ${(r.report.theta0 ?? 0).toFixed(1)}° / ${((r.report.theta0 ?? 0) + 90).toFixed(1)}°), ${r.report.aligned} alignés, ${r.report.merged} fusions, ${r.report.joined} coins raccordés, ${r.report.trimmed} dépassements rognés`, ms: T() - t0 });
    }
    // filtres « distance minimum » : taille des éléments et distance aux positions du robot
    const F = o.filters || {}, fill = F.fill || 'free';
    const tmp = { grid, vmap: vmap === vmapRaw ? JSON.parse(JSON.stringify(vmapRaw)) : vmap, omap, poses };   // la carte « avant régularisation » reste intacte
    if (F.minSegLen > 0 || F.minCircleD > 0 || F.minBlob > 0) {
      t0 = T(); const r = filterSmall(tmp, Object.assign({ fill }, F));
      steps.push({ step: 'Filtre de taille', detail: `${r.segments} segments de moins de ${Math.round((F.minSegLen || 0) * 100)} cm, ${r.circles} cercles de moins de ${Math.round((F.minCircleD || 0) * 100)} cm de diamètre, ${r.blobs} taches de moins de ${Math.round((F.minBlob || 0) * 100)} cm (${r.cells} cellules) supprimés`, ms: T() - t0 });
    }
    if (F.nearPoses > 0) {
      t0 = T(); const r = filterNearPoses(tmp, poses, F.nearPoses, { fill });
      steps.push({ step: 'Filtre de distance au robot', detail: `à moins de ${Math.round(F.nearPoses * 100)} cm d'une position du robot : ${r.cells} cellules, ${r.segCut} segments coupés (${r.segRemoved} supprimés), ${r.circles} cercles supprimés`, ms: T() - t0 });
    }
    vmap = tmp.vmap;
    if (vmap !== vmapRaw) vmap.corners = LC.computeCorners(vmap.features, LC.VM_DEFAULTS);
    const qClean = mapQuality(grid, data, layers);

    await prog(base + 0.8 * (1 - base), 'Obstacles de la grille');
    t0 = T();
    const { labels, obstacles } = gridObstacles(grid, layers, o.obstacleOpt);
    const cnt = c => obstacles.filter(x => x.cls === c).length;
    steps.push({ step: 'Obstacles (composantes connexes)', detail: `${obstacles.length} : ${cnt('structure')} structures, ${cnt('objet')} objets, ${cnt('petit')} petits`, ms: T() - t0 });

    await prog(base + 0.85 * (1 - base), 'Carte de coût');
    t0 = T();
    const cost = costmap(grid, o.cost);
    steps.push({ step: 'Carte de coût', detail: `robot ${Math.round(o.cost.robotRadius * 100)} cm + marge ${Math.round(o.cost.inflation * 100)} cm ; ${cost.stats.navigable.toFixed(1)} m² accessibles au centre du robot`, ms: T() - t0 });
    await prog(1, 'terminé');
    return { poses, refineLog, pre, gridRaw, grid, layers, mobile, obstacles, labels, omap, vmapRaw, vmap, regReport, cost,
             quality: { raw: qRaw, clean: qClean }, steps, opt: { obstacleOpt: o.obstacleOpt, cost: o.cost },
             manual: { cells: 0, segCut: 0, segRemoved: 0, circles: 0 } };
  }

  function reportToMarkdown(R) {
    const q = R.quality, pc = v => (v * 100).toFixed(1) + ' %';
    let s = '# Traitement de la carte\n\n## Étapes\n\n| Étape | Résultat | Temps |\n|---|---|---|\n';
    for (const st of R.steps) s += `| ${st.step} | ${st.detail} | ${st.ms.toFixed(0)} ms |\n`;
    s += '\n## Qualité de la grid map\n\n| Mesure | Brute | Traitée |\n|---|---|---|\n';
    s += `| Points des scans sur un obstacle (≤ 1,5 cellule) | ${pc(q.raw.pointsOnObstacle)} | ${pc(q.clean.pointsOnObstacle)} |\n`;
    s += `| Distance moyenne point → obstacle | ${(q.raw.meanDist * 100).toFixed(1)} cm | ${(q.clean.meanDist * 100).toFixed(1)} cm |\n`;
    s += `| Cellules occupées | ${q.raw.occCells} | ${q.clean.occCells} |\n`;
    s += `| Surface libre | ${q.raw.freeArea.toFixed(1)} m² | ${q.clean.freeArea.toFixed(1)} m² |\n`;
    s += `| Netteté (cellules observées tranchées) | ${pc(q.raw.sharpness)} | ${pc(q.clean.sharpness)} |\n`;
    s += `| Cellules en conflit (persistance 25–75 %) | ${q.raw.conflicts} | ${q.clean.conflicts} |\n`;
    if (R.refineLog) {
      s += '\n## Recalage des poses\n\n| Passe | Scan | dx (cm) | dy (cm) | dθ (°) | Score avant | Score après | Acceptée |\n|---|---|---|---|---|---|---|---|\n';
      for (const l of R.refineLog) s += `| ${l.pass} | ${l.name} | ${(l.dx * 100).toFixed(1)} | ${(l.dy * 100).toFixed(1)} | ${l.dth.toFixed(2)} | ${l.before.toFixed(3)} | ${l.after.toFixed(3)} | ${l.accepted ? 'oui' : 'non'} |\n`;
    }
    s += '\n## Obstacles de la grille\n\n| Classe | Nombre | Surface |\n|---|---|---|\n';
    for (const c of ['structure', 'objet', 'petit']) { const L = R.obstacles.filter(o => o.cls === c); s += `| ${c} | ${L.length} | ${L.reduce((a, o) => a + o.area, 0).toFixed(2)} m² |\n`; }
    return s;
  }

  return {
    PRE_DEFAULTS, FEAT_DEFAULTS, OM_DEFAULTS, REG_DEFAULTS, PIPE_DEFAULTS, COST,
    preprocessScan, clusterScan, fitCircle, douglasPeucker, dpSegments, ransacLines, smallObjectCircle, extractFeatures,
    createObstacleMap, updateObstacleMap, pruneObstaclesOnWalls, obstaclesToCSV,
    createLayers, updateGridLayered, buildLayeredGrid, cloneGrid, cloneLayers,
    decayGrid, clearObsolete, removeMobile, labelComponents, removeSpeckles, smoothGrid, closeGaps, gridObstacles,
    dominantDirection, snapDirections, alignCollinear, mergeKeepProps, joinCorners, trimCollisions, regularizeVectorMap,
    costmap, costmapToPGM, planPath, mapQuality, refinePoses,
    clipSegmentDisc, eraseDisc, filterSmall, filterNearPoses, refreshDerived, processMap, reportToMarkdown
  };
});
