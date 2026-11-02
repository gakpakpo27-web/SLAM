/* ============================================================================
 *  lidar_core.js – Grid map 2D et carte vectorielle (segments) à partir de scans LD06
 *  Fonctions pures, sans dépendance : utilisables dans le navigateur
 *  (window.LidarCore, intégrées à lidar_viewer.html) et sous Node.js (require).
 *
 *  Conventions
 *   - Unités : mètres, degrés pour les poses/affichages, radians en interne.
 *   - Pose du robot : { x, y, th } (th en degrés, sens trigonométrique).
 *   - Angles lidar : sens trigonométrique, comme getPoints() de la librairie LD06.
 *   - Segment : { x1, y1, x2, y2 } ; le « premier point » est (x1, y1).
 * ========================================================================== */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.LidarCore = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  // --------------------------------------------------------------------------
  // Tolérances numériques
  // --------------------------------------------------------------------------
  // EPS : tolérance géométrique en mètres (1 µm). Bien en dessous de la précision du
  // LD06 (~1 cm) mais très au-dessus des erreurs d'arrondi des doubles (~1e-15 m sur 10 m).
  // Elle sert pour : « point sur un segment », « intersection à une extrémité »,
  // « longueurs égales », « colinéarité ».
  const EPS = 1e-6;
  // EPS_SIN : deux directions sont parallèles si |sin(angle)| <= EPS_SIN.
  const EPS_SIN = 1e-9;
  const D2R = Math.PI / 180, R2D = 180 / Math.PI;

  // ==========================================================================
  //  1. Conversion d'un scan polaire en points globaux
  // ==========================================================================
  /**
   * scan : { a: [deg], d: [mm], i?: [0..255] } ; pose : { x, y, th(deg) }
   * filtre : distances hors [dMin, dMax] (m) et intensité < iMin supprimées.
   * Renvoie les points dans l'ordre angulaire : [{ x, y, r }]
   */
  function polarToGlobal(scan, pose, opt = {}) {
    const { dMin = 0.05, dMax = 12, iMin = 0, scale = 1 } = opt;
    const idx = [];
    for (let k = 0; k < scan.a.length; k++) {
      const r = scan.d[k] / 1000;
      if (!(r >= dMin && r <= dMax)) continue;
      if (scan.i && scan.i[k] < iMin) continue;
      idx.push(k);
    }
    idx.sort((p, q) => scan.a[p] - scan.a[q]);
    const c = Math.cos(pose.th * D2R), s = Math.sin(pose.th * D2R);
    return idx.map(k => {
      const r = scan.d[k] / 1000 * scale, a = scan.a[k] * D2R;
      const lx = r * Math.cos(a), ly = r * Math.sin(a);
      return { x: pose.x + c * lx - s * ly, y: pose.y + s * lx + c * ly, r };
    });
  }

  // ==========================================================================
  //  2. Grid map (carte d'occupation)
  // ==========================================================================
  // Valeur d'une cellule : uint8. 0 = libre certain, 255 = occupé certain, 127 = inconnu.
  // Mise à jour additive saturée : équivalent à un filtre bayésien en log-odds quantifié
  // (valeur ≈ 127 + k·log(p/(1-p))).
  const UNKNOWN = 127, FREE_T = 85, OCC_T = 170;

  function createGrid({ xMin, yMin, xMax, yMax, res = 0.05, init = UNKNOWN }) {
    if (!(res > 0)) throw new Error('résolution invalide');
    const W = Math.max(1, Math.ceil((xMax - xMin) / res - 1e-9));
    const H = Math.max(1, Math.ceil((yMax - yMin) / res - 1e-9));
    if (W * H > 40e6) throw new Error(`grille trop grande (${W}×${H}) : augmenter la résolution`);
    return { res, x0: xMin, y0: yMin, W, H, init, data: new Uint8Array(W * H).fill(init) };
  }

  // (x, y) -> indices (i colonne selon x, j ligne selon y)
  function worldToCell(g, x, y) {
    return [Math.floor((x - g.x0) / g.res), Math.floor((y - g.y0) / g.res)];
  }
  function cellCenter(g, i, j) { return [g.x0 + (i + 0.5) * g.res, g.y0 + (j + 0.5) * g.res]; }
  function inGrid(g, i, j) { return i >= 0 && j >= 0 && i < g.W && j < g.H; }
  function getCell(g, i, j) { return inGrid(g, i, j) ? g.data[j * g.W + i] : undefined; }

  // Bresenham, toutes directions. Appelle visit(i, j) pour chaque cellule, extrémités comprises.
  function bresenham(i0, j0, i1, j1, visit) {
    let dx = Math.abs(i1 - i0), dy = -Math.abs(j1 - j0);
    const sx = i0 < i1 ? 1 : -1, sy = j0 < j1 ? 1 : -1;
    let err = dx + dy, i = i0, j = j0, n = 0;
    for (;;) {
      visit(i, j); n++;
      if (i === i1 && j === j1) break;
      const e2 = 2 * err;
      if (e2 >= dy) { err += dy; i += sx; }
      if (e2 <= dx) { err += dx; j += sy; }
    }
    return n;
  }

  // Cellules dont le centre est dans le triangle (a, b, c) (bords inclus)
  function fillTriangle(g, a, b, c, visit) {
    const x0 = Math.min(a.x, b.x, c.x), x1 = Math.max(a.x, b.x, c.x), y0 = Math.min(a.y, b.y, c.y), y1 = Math.max(a.y, b.y, c.y);
    const i0 = Math.max(0, Math.floor((x0 - g.x0) / g.res)), i1 = Math.min(g.W - 1, Math.floor((x1 - g.x0) / g.res));
    const j0 = Math.max(0, Math.floor((y0 - g.y0) / g.res)), j1 = Math.min(g.H - 1, Math.floor((y1 - g.y0) / g.res));
    const area = cross(b.x - a.x, b.y - a.y, c.x - a.x, c.y - a.y);
    if (Math.abs(area) < 1e-12) return;
    const sg = Math.sign(area), tol = -1e-9;
    for (let j = j0; j <= j1; j++) {
      const yc = g.y0 + (j + 0.5) * g.res;
      for (let i = i0; i <= i1; i++) {
        const xc = g.x0 + (i + 0.5) * g.res;
        if (sg * cross(b.x - a.x, b.y - a.y, xc - a.x, yc - a.y) >= tol &&
            sg * cross(c.x - b.x, c.y - b.y, xc - b.x, yc - b.y) >= tol &&
            sg * cross(a.x - c.x, a.y - c.y, xc - c.x, yc - c.y) >= tol) visit(i, j);
      }
    }
  }

  /**
   * Met à jour la grille g avec un scan déjà exprimé dans le repère de la carte.
   * robot : { x, y } position du lidar ; pts : [{ x, y }] points d'impact.
   * p.model = 'proba' (incréments saturés) ou 'binary' (affectation directe).
   * Chaque cellule n'est mise à jour qu'une fois par scan, et une cellule d'impact
   * n'est jamais libérée par un autre faisceau du même scan.
   */
  function updateGrid(g, robot, pts, p = {}) {
    const { model = 'proba', free = 45, occ = 70, fillBetween = false } = p;   // 1 observation suffit à franchir les seuils 85 / 170
    const freeSet = new Set(), occSet = new Set();
    const [ri, rj] = worldToCell(g, robot.x, robot.y);
    let outside = 0;
    for (const pt of pts) {
      const [i, j] = worldToCell(g, pt.x, pt.y);
      if (inGrid(g, i, j)) occSet.add(j * g.W + i); else outside++;
      // cellules traversées (sans la cellule d'impact) ; seules celles dans la grille comptent
      bresenham(ri, rj, i, j, (a, b) => {
        if (a === i && b === j) return;
        if (inGrid(g, a, b)) freeSet.add(b * g.W + a);
      });
    }
    // Option : loin du robot, deux faisceaux voisins sont écartés de plus d'une cellule et des
    // cellules libres ne sont traversées par aucun faisceau (stries). Si deux points successifs
    // (ordre angulaire) sont sur la même surface, le triangle robot–p(k)–p(k+1) est vu libre.
    if (fillBetween && pts.length > 1) {
      for (let k = 0; k < pts.length; k++) {
        const a = pts[k], b = pts[(k + 1) % pts.length];
        const ra = Math.hypot(a.x - robot.x, a.y - robot.y), rb = Math.hypot(b.x - robot.x, b.y - robot.y);
        const gap = Math.hypot(a.x - b.x, a.y - b.y);
        if (gap > Math.max(0.25, 0.05 * Math.max(ra, rb))) continue;           // discontinuité : surfaces différentes
        // le bord du triangle est reculé d'une cellule : les cellules du mur entre deux impacts restent intactes
        const ka = Math.max(0, (ra - g.res) / ra), kb = Math.max(0, (rb - g.res) / rb);
        const a2 = { x: robot.x + (a.x - robot.x) * ka, y: robot.y + (a.y - robot.y) * ka };
        const b2 = { x: robot.x + (b.x - robot.x) * kb, y: robot.y + (b.y - robot.y) * kb };
        fillTriangle(g, robot, a2, b2, (i, j) => freeSet.add(j * g.W + i));
      }
    }
    for (const k of occSet) freeSet.delete(k);         // l'impact est prioritaire
    const d = g.data;
    if (model === 'binary') {
      for (const k of freeSet) d[k] = 0;
      for (const k of occSet) d[k] = 255;
    } else {
      for (const k of freeSet) d[k] = Math.max(0, d[k] - free);
      for (const k of occSet) d[k] = Math.min(255, d[k] + occ);
    }
    return { freed: freeSet.size, occupied: occSet.size, outside, robotInside: inGrid(g, ri, rj) };
  }

  // Outil 1 : scan + pose -> nouvelle carte d'occupation (dimensionnée sur le scan)
  function scanToGrid(robot, pts, p = {}) {
    const { res = 0.05, margin = 0.5, init = UNKNOWN } = p;
    let xMin = robot.x, xMax = robot.x, yMin = robot.y, yMax = robot.y;
    for (const q of pts) { if (q.x < xMin) xMin = q.x; if (q.x > xMax) xMax = q.x; if (q.y < yMin) yMin = q.y; if (q.y > yMax) yMax = q.y; }
    const g = createGrid({ xMin: xMin - margin, yMin: yMin - margin, xMax: xMax + margin, yMax: yMax + margin, res, init });
    const stats = updateGrid(g, robot, pts, p);
    return { grid: g, stats };
  }

  // Agrandit la grille (en conservant son contenu et sa résolution) pour couvrir une zone
  function ensureCovers(g, xMin, yMin, xMax, yMax, margin = 0.5) {
    const nx0 = Math.min(g.x0, xMin - margin), ny0 = Math.min(g.y0, yMin - margin);
    const nx1 = Math.max(g.x0 + g.W * g.res, xMax + margin), ny1 = Math.max(g.y0 + g.H * g.res, yMax + margin);
    // aligner sur la grille existante pour ne pas rééchantillonner
    const di = Math.ceil((g.x0 - nx0) / g.res - 1e-9), dj = Math.ceil((g.y0 - ny0) / g.res - 1e-9);
    const W = di + g.W + Math.max(0, Math.ceil((nx1 - (g.x0 + g.W * g.res)) / g.res - 1e-9));
    const H = dj + g.H + Math.max(0, Math.ceil((ny1 - (g.y0 + g.H * g.res)) / g.res - 1e-9));
    if (di === 0 && dj === 0 && W === g.W && H === g.H) return g;
    const n = { res: g.res, x0: g.x0 - di * g.res, y0: g.y0 - dj * g.res, W, H, init: g.init, data: new Uint8Array(W * H).fill(g.init) };
    for (let j = 0; j < g.H; j++) n.data.set(g.data.subarray(j * g.W, (j + 1) * g.W), (j + dj) * W + di);
    return n;
  }

  function gridStats(g) {
    let free = 0, occ = 0, unk = 0;
    for (const v of g.data) { if (v <= FREE_T) free++; else if (v >= OCC_T) occ++; else unk++; }
    return { free, occ, unk, cells: g.W * g.H, bytes: g.data.length };
  }

  // Sérialisation (JSON + base64) et export PGM/YAML (format map_server de ROS)
  function b64encode(u8) {
    if (typeof Buffer !== 'undefined') return Buffer.from(u8).toString('base64');
    let s = ''; for (let k = 0; k < u8.length; k += 0x8000) s += String.fromCharCode.apply(null, u8.subarray(k, k + 0x8000));
    return btoa(s);
  }
  function b64decode(str) {
    if (typeof Buffer !== 'undefined') return new Uint8Array(Buffer.from(str, 'base64'));
    const s = atob(str), u = new Uint8Array(s.length); for (let k = 0; k < s.length; k++) u[k] = s.charCodeAt(k); return u;
  }
  function gridToJSON(g) {
    return { format: 'ld06-gridmap', version: 1, res: g.res, x0: g.x0, y0: g.y0, W: g.W, H: g.H, init: g.init,
             encoding: 'uint8 base64, ligne j=0 en bas (y min)', data: b64encode(g.data) };
  }
  function gridFromJSON(o) {
    if (!o || o.format !== 'ld06-gridmap') throw new Error('fichier de grille non reconnu');
    const data = b64decode(o.data);
    if (data.length !== o.W * o.H) throw new Error('taille des données incohérente');
    return { res: o.res, x0: o.x0, y0: o.y0, W: o.W, H: o.H, init: o.init ?? UNKNOWN, data };
  }
  function gridToPGM(g, name = 'carte') {
    const header = `P5\n# ${name} res=${g.res}\n${g.W} ${g.H}\n255\n`;
    const out = new Uint8Array(header.length + g.W * g.H);
    for (let k = 0; k < header.length; k++) out[k] = header.charCodeAt(k);
    let o = header.length;
    for (let j = g.H - 1; j >= 0; j--) for (let i = 0; i < g.W; i++) out[o++] = 255 - g.data[j * g.W + i]; // blanc = libre
    const yaml = `image: ${name}.pgm\nresolution: ${g.res}\norigin: [${g.x0.toFixed(4)}, ${g.y0.toFixed(4)}, 0.0]\n` +
                 `negate: 0\noccupied_thresh: ${(OCC_T / 255).toFixed(3)}\nfree_thresh: ${(FREE_T / 255).toFixed(3)}\n`;
    return { pgm: out, yaml };
  }

  // ==========================================================================
  //  3. Segments
  // ==========================================================================
  const seg = (x1, y1, x2, y2, extra) => Object.assign({ x1, y1, x2, y2 }, extra || {});
  const segLen = s => Math.hypot(s.x2 - s.x1, s.y2 - s.y1);
  const segAngle = s => Math.atan2(s.y2 - s.y1, s.x2 - s.x1);            // rad, orienté
  const cross = (ax, ay, bx, by) => ax * by - ay * bx;

  /** Angle entre deux segments, en degrés.
   *  lines : angle entre les droites, dans [0, 90] (l'orientation des segments est ignorée)
   *  oriented : angle orienté de s1 vers s2, dans ]-180, 180] */
  function angleBetween(s1, s2) {
    let d = (segAngle(s2) - segAngle(s1)) * R2D;
    d = ((d + 180) % 360 + 360) % 360 - 180; if (d === -180) d = 180;
    let l = Math.abs(d) % 180; if (l > 90) l = 180 - l;
    return { lines: l, oriented: d };
  }

  // Distance d'un point à la droite support d'un segment, et au segment lui-même
  function pointLineDist(px, py, s) {
    const L = segLen(s); if (L <= EPS) return Math.hypot(px - s.x1, py - s.y1);
    return Math.abs(cross(s.x2 - s.x1, s.y2 - s.y1, px - s.x1, py - s.y1)) / L;
  }
  function pointSegDist(px, py, s) {
    const dx = s.x2 - s.x1, dy = s.y2 - s.y1, L2 = dx * dx + dy * dy;
    let t = L2 > 0 ? ((px - s.x1) * dx + (py - s.y1) * dy) / L2 : 0;
    t = Math.max(0, Math.min(1, t));
    return Math.hypot(px - (s.x1 + t * dx), py - (s.y1 + t * dy));
  }

  /**
   * Deux segments sont « proches » si :
   *  - angle entre leurs droites <= angTol (°)
   *  - écart latéral : distance des extrémités du segment le plus court à la droite du plus long
   *    (la droite du plus long est la plus fiable ; prolonger celle d'un petit segment bruité amplifie
   *    son erreur d'orientation) <= distTol (m)
   *  - trou longitudinal entre leurs projections sur la direction moyenne <= gapTol (m)
   *    (0 s'ils se chevauchent)
   */
  function areClose(s1, s2, opt = {}) {
    const { angTol = 5, distTol = 0.08, gapTol = 0.15 } = opt;
    const angle = angleBetween(s1, s2).lines;
    const [lg, sh] = segLen(s1) >= segLen(s2) ? [s1, s2] : [s2, s1];
    const lateral = Math.max(pointLineDist(sh.x1, sh.y1, lg), pointLineDist(sh.x2, sh.y2, lg));
    const u = meanDirection(s1, s2);
    const pr = (x, y) => x * u[0] + y * u[1];
    const a = [pr(s1.x1, s1.y1), pr(s1.x2, s1.y2)].sort((p, q) => p - q);
    const b = [pr(s2.x1, s2.y1), pr(s2.x2, s2.y2)].sort((p, q) => p - q);
    const gap = Math.max(0, Math.max(a[0], b[0]) - Math.min(a[1], b[1]));
    const angOk = angle <= angTol + 1e-12, latOk = lateral <= distTol + EPS, gapOk = gap <= gapTol + EPS;
    return { close: angOk && latOk && gapOk, angle, lateral, gap, angOk, latOk, gapOk };
  }

  // Direction moyenne pondérée par la longueur (moyenne sur l'angle doublé : indépendante du sens)
  function meanDirection(s1, s2, w1 = segLen(s1), w2 = segLen(s2)) {
    const t1 = segAngle(s1), t2 = segAngle(s2);
    const phi = 0.5 * Math.atan2(w1 * Math.sin(2 * t1) + w2 * Math.sin(2 * t2), w1 * Math.cos(2 * t1) + w2 * Math.cos(2 * t2));
    let u = [Math.cos(phi), Math.sin(phi)];
    if (u[0] * Math.cos(t1) + u[1] * Math.sin(t1) < 0) u = [-u[0], -u[1]];   // garder le sens de s1
    return u;
  }

  /**
   * Fusion de deux segments : droite de direction moyenne (pondérée par la longueur)
   * passant par le barycentre des milieux (pondéré), extrémités = projections extrêmes
   * des 4 extrémités. Le résultat garde le sens de s1.
   * w1, w2 : poids optionnels (par défaut les longueurs ; la carte vectorielle utilise longueur × confiance).
   */
  function mergeSegments(s1, s2, w1 = segLen(s1), w2 = segLen(s2)) {
    const W = w1 + w2 || 1;
    const u = meanDirection(s1, s2, w1, w2);
    const cx = (w1 * (s1.x1 + s1.x2) / 2 + w2 * (s2.x1 + s2.x2) / 2) / W;
    const cy = (w1 * (s1.y1 + s1.y2) / 2 + w2 * (s2.y1 + s2.y2) / 2) / W;
    const ts = [[s1.x1, s1.y1], [s1.x2, s1.y2], [s2.x1, s2.y1], [s2.x2, s2.y2]].map(([x, y]) => (x - cx) * u[0] + (y - cy) * u[1]);
    const t0 = Math.min(...ts), t1 = Math.max(...ts);
    return seg(cx + t0 * u[0], cy + t0 * u[1], cx + t1 * u[0], cy + t1 * u[1], { n: (s1.n || 0) + (s2.n || 0) });
  }

  /**
   * Intersection de deux segments.
   * type : 'point'      -> se coupent (point sur les deux segments, tolérance EPS)
   *        'lines'      -> les droites se coupent mais hors d'au moins un segment
   *        'parallel'   -> parallèles distincts
   *        'collinear'  -> sur la même droite ; overlap = partie commune (segment ou point) ou null
   * t, u : position du point sur s1 et s2 (0 = premier point, 1 = second)
   */
  function intersect(s1, s2) {
    const rx = s1.x2 - s1.x1, ry = s1.y2 - s1.y1, qx = s2.x2 - s2.x1, qy = s2.y2 - s2.y1;
    const L1 = Math.hypot(rx, ry), L2 = Math.hypot(qx, qy);
    if (L1 <= EPS || L2 <= EPS) return { type: 'degenerate' };
    const den = cross(rx, ry, qx, qy), wx = s2.x1 - s1.x1, wy = s2.y1 - s1.y1;
    if (Math.abs(den) <= EPS_SIN * L1 * L2) {
      // parallèles : même droite si le premier point de s2 est sur la droite de s1
      if (Math.abs(cross(rx, ry, wx, wy)) / L1 > EPS) return { type: 'parallel' };
      const ta = (wx * rx + wy * ry) / (L1 * L1), tb = ((s2.x2 - s1.x1) * rx + (s2.y2 - s1.y1) * ry) / (L1 * L1);
      const lo = Math.max(0, Math.min(ta, tb)), hi = Math.min(1, Math.max(ta, tb));
      let overlap = null;
      if ((hi - lo) * L1 >= -EPS) overlap = seg(s1.x1 + lo * rx, s1.y1 + lo * ry, s1.x1 + hi * rx, s1.y1 + hi * ry);
      return { type: 'collinear', overlap };
    }
    const t = cross(wx, wy, qx, qy) / den, u = cross(wx, wy, rx, ry) / den;
    const x = s1.x1 + t * rx, y = s1.y1 + t * ry;
    // tolérance exprimée en distance (m) le long de chaque segment
    const onS1 = t * L1 >= -EPS && (1 - t) * L1 >= -EPS;
    const onS2 = u * L2 >= -EPS && (1 - u) * L2 >= -EPS;
    return { type: onS1 && onS2 ? 'point' : 'lines', x, y, t, u, onS1, onS2 };
  }

  /**
   * Découpe de « target » par « cutter », en conservant le plus long côté.
   * Règles :
   *  - il faut une intersection sur les deux segments (extrémités de cutter incluses, à EPS près)
   *    ET strictement à l'intérieur de target (à plus de EPS de ses deux extrémités) ;
   *  - intersection à une extrémité de target -> rien ne change ;
   *  - segments colinéaires (ou parallèles) -> rien ne change ;
   *  - longueurs des deux côtés égales à EPS près -> on garde le côté contenant le premier point (x1, y1).
   * Renvoie { changed, segment, kept: 'premier'|'second'|null, reason, point? }
   */
  function cutSegment(target, cutter) {
    const same = { changed: false, segment: seg(target.x1, target.y1, target.x2, target.y2), kept: null };
    const r = intersect(target, cutter);
    if (r.type === 'degenerate') return { ...same, reason: 'segment de longueur nulle' };
    if (r.type === 'collinear') return { ...same, reason: 'segments colinéaires : pas de découpe' };
    if (r.type === 'parallel') return { ...same, reason: 'segments parallèles : pas d’intersection' };
    if (!r.onS1 || !r.onS2) return { ...same, reason: 'les droites se coupent hors d’un des segments', point: [r.x, r.y] };
    const L = segLen(target), dFirst = r.t * L, dSecond = (1 - r.t) * L;
    if (dFirst <= EPS || dSecond <= EPS) return { ...same, reason: 'intersection à une extrémité : rien ne change', point: [r.x, r.y] };
    const keepFirst = Math.abs(dFirst - dSecond) <= EPS ? true : dFirst > dSecond;
    const segment = keepFirst ? seg(target.x1, target.y1, r.x, r.y) : seg(r.x, r.y, target.x2, target.y2);
    return {
      changed: true, segment, kept: keepFirst ? 'premier' : 'second', point: [r.x, r.y],
      reason: Math.abs(dFirst - dSecond) <= EPS ? 'côtés égaux (à EPS près) : côté du premier point conservé'
                                                 : `côté ${keepFirst ? 'du premier point' : 'du second point'} conservé (le plus long)`,
      lengths: [dFirst, dSecond]
    };
  }

  // --------------------------------------------------------------------------
  //  Extraction de segments à partir d'un nuage de points ordonné angulairement
  // --------------------------------------------------------------------------
  // Ajustement par moindres carrés totaux (ACP) : droite qui minimise les distances perpendiculaires
  function fitLine(P) {
    let mx = 0, my = 0; for (const p of P) { mx += p.x; my += p.y; } mx /= P.length; my /= P.length;
    let sxx = 0, sxy = 0, syy = 0;
    for (const p of P) { const ax = p.x - mx, ay = p.y - my; sxx += ax * ax; sxy += ax * ay; syy += ay * ay; }
    const phi = 0.5 * Math.atan2(2 * sxy, sxx - syy), ux = Math.cos(phi), uy = Math.sin(phi);
    let e2 = 0; for (const p of P) { const d = -(p.x - mx) * uy + (p.y - my) * ux; e2 += d * d; }
    return { mx, my, ux, uy, rmse: Math.sqrt(e2 / P.length) };
  }
  function fitSegment(P) {
    const f = fitLine(P), a = P[0], b = P[P.length - 1];
    const ta = (a.x - f.mx) * f.ux + (a.y - f.my) * f.uy, tb = (b.x - f.mx) * f.ux + (b.y - f.my) * f.uy;
    return seg(f.mx + ta * f.ux, f.my + ta * f.uy, f.mx + tb * f.ux, f.my + tb * f.uy, { n: P.length, rmse: f.rmse });
  }

  /**
   * Nuage de points -> segments (méthode « split and merge »)
   *  1. Segmentation : nouveau groupe quand deux points successifs sont séparés de plus de
   *     gapBase + gapK·r (le seuil grandit avec la distance r car les points s'espacent).
   *  2. Découpe récursive (Iterative End Point Fit) : on coupe au point le plus éloigné de la
   *     corde tant que cet écart dépasse splitTol.
   *  3. Ajustement de chaque morceau par moindres carrés totaux, extrémités projetées.
   *  4. Fusion des morceaux voisins colinéaires (areClose avec les tolérances de fusion).
   *  5. Filtrage : au moins minPts points et minLen mètres.
   * pts : [{ x, y }] dans l'ordre angulaire autour de origin.
   */
  function extractSegments(pts, origin, opt = {}) {
    const { gapBase = 0.10, gapK = 0.03, splitTol = 0.04, minPts = 6, minLen = 0.25,
            mergeAng = 4, mergeDist = 0.05, mergeGap = 0.12 } = opt;
    const n = pts.length; if (n < minPts) return [];
    const range = p => Math.hypot(p.x - origin.x, p.y - origin.y);
    const thr = (p, q) => gapBase + gapK * Math.max(range(p), range(q));
    // commencer après la plus grande rupture pour ne pas couper un mur à l'angle 0°/360°
    let start = 0, worst = -1;
    for (let k = 0; k < n; k++) {
      const p = pts[k], q = pts[(k + 1) % n], d = Math.hypot(p.x - q.x, p.y - q.y) - thr(p, q);
      if (d > worst) { worst = d; start = (k + 1) % n; }
    }
    const P = pts.slice(start).concat(pts.slice(0, start));
    const clusters = []; let cur = [P[0]];
    for (let k = 1; k < n; k++) {
      const p = P[k - 1], q = P[k];
      if (Math.hypot(p.x - q.x, p.y - q.y) > thr(p, q)) { clusters.push(cur); cur = []; }
      cur.push(q);
    }
    clusters.push(cur);

    const out = [];
    for (const C of clusters) {
      if (C.length < minPts) continue;
      // découpe récursive (pile)
      const pieces = [], stack = [[0, C.length - 1]];
      while (stack.length) {
        const [i0, i1] = stack.pop();
        const ch = seg(C[i0].x, C[i0].y, C[i1].x, C[i1].y);
        let dmax = -1, imax = -1;
        for (let k = i0 + 1; k < i1; k++) {
          const d = segLen(ch) > EPS ? pointLineDist(C[k].x, C[k].y, ch) : Math.hypot(C[k].x - C[i0].x, C[k].y - C[i0].y);
          if (d > dmax) { dmax = d; imax = k; }
        }
        if (dmax > splitTol && imax - i0 >= 2 && i1 - imax >= 2) { stack.push([imax, i1]); stack.push([i0, imax]); }
        else pieces.push([i0, i1]);
      }
      pieces.sort((a, b) => a[0] - b[0]);
      let segs = pieces.filter(([a, b]) => b - a + 1 >= Math.max(2, Math.min(minPts, 4)))
                       .map(([a, b]) => fitSegment(C.slice(a, b + 1)));
      // fusion des voisins colinéaires
      for (let k = 0; k + 1 < segs.length;) {
        if (areClose(segs[k], segs[k + 1], { angTol: mergeAng, distTol: mergeDist, gapTol: mergeGap }).close) {
          segs.splice(k, 2, mergeSegments(segs[k], segs[k + 1]));
        } else k++;
      }
      for (const s of segs) if ((s.n || 0) >= minPts && segLen(s) >= minLen) out.push(s);
    }
    return out;
  }

  // Fusion itérative de tous les segments proches d'une liste (carte vectorielle globale)
  function mergeAll(segs, opt = {}) {
    let list = segs.map(s => ({ ...s })), merges = 0, changed = true;
    while (changed) {
      changed = false;
      outer: for (let i = 0; i < list.length; i++) for (let j = i + 1; j < list.length; j++) {
        if (areClose(list[i], list[j], opt).close) {
          const m = mergeSegments(list[i], list[j]);
          list.splice(j, 1); list[i] = m; merges++; changed = true; break outer;
        }
      }
    }
    return { segments: list, merges };
  }

  function segmentsToText(segs) {
    return '# Carte vectorielle (segments) x1,y1,x2,y2 en metres\n' +
      segs.map(s => [s.x1, s.y1, s.x2, s.y2].map(v => v.toFixed(4)).join(',')).join('\n') + '\n';
  }
  function segmentsFromText(txt) {
    const out = [];
    for (const line of txt.split(/\r?\n/)) {
      if (!line.trim() || line.trim().startsWith('#')) continue;
      const v = line.split(/[,;\s]+/).map(Number);
      if (v.length >= 4 && v.slice(0, 4).every(isFinite)) out.push(seg(v[0], v[1], v[2], v[3]));
    }
    return out;
  }


  // ==========================================================================
  //  4. Carte vectorielle (liste de features : segments + coins)
  // ==========================================================================
  // Feature segment : { id, x1, y1, x2, y2, conf, nObs, firstSeen, lastSeen, n }
  //   conf      : compteur de confiance (augmente quand le segment est revu, diminue quand il est traversé)
  //   nObs      : nombre de scans où il a été observé ; firstSeen / lastSeen : numéros de scan (horodatage)
  // Coin : { x, y, th (deg, bissectrice), a, b (ids des segments), conf }
  const VM_DEFAULTS = {
    angTol: 5, distTol: 0.10, gapTol: 0.20,                 // association : même tolérances que « proches »
    confInit: 1, confHit: 1, confMiss: 1, confMax: 10,      // confiance
    absence: true, absMinRays: 2, absMargin: 0.15, absMinAngle: 10, absClusterGap: 0.30,
    maxAge: 0,                                              // durée de vie en scans (0 = infinie)
    minLen: 0.30, mergeRedundant: true,                     // nettoyage
    cornerAngle: 60, cornerDist: 0.30,                      // coins
    extract: {}                                             // paramètres de extractSegments
  };
  function createVectorMap() { return { features: [], corners: [], scans: 0, nextId: 1 }; }
  const cloneFeature = (f, geom, id) => Object.assign({}, f, { x1: geom.x1, y1: geom.y1, x2: geom.x2, y2: geom.y2 }, id ? { id } : {});
  const pointAt = (f, u) => [f.x1 + (f.x2 - f.x1) * u, f.y1 + (f.y2 - f.y1) * u];

  /**
   * Intègre un scan (déjà dans le repère de la carte) à la carte vectorielle.
   * Étapes (document « Construction d'une carte vectorielle 2D ») :
   *  3. extraction des segments du scan ;
   *  5. gestion des absences : features existantes traversées par les faisceaux -> raccourcies,
   *     scindées, ou confiance diminuée (et supprimées à 0) ;
   *  4. association : chaque segment extrait est fusionné avec les features « proches », sinon ajouté ;
   *  6. nettoyage : fusion des features redondantes, suppression des trop courtes et des trop anciennes ;
   *     puis calcul des coins (points d'intérêt).
   * Renvoie un compte rendu (ids concernés, parties supprimées pour l'affichage).
   */
  function updateVectorMap(map, robot, pts, opt = {}) {
    const o = Object.assign({}, VM_DEFAULTS, opt);
    const t = ++map.scans;
    const st = { scan: t, extracted: 0, added: [], merged: [], shortened: [], split: [], weakened: [], removed: [], expired: [], fused: 0, ghosts: [] };

    const segs = extractSegments(pts, robot, o.extract);
    st.extracted = segs.length;

    if (o.absence && map.features.length) absenceCheck(map, robot, pts, o, st);

    // association / fusion
    for (const s of segs) {
      const cands = map.features.filter(f => areClose(f, s, o).close);
      if (!cands.length) {
        const f = Object.assign(seg(s.x1, s.y1, s.x2, s.y2), { id: map.nextId++, conf: o.confInit, nObs: 1, firstSeen: t, lastSeen: t, n: s.n || 0 });
        map.features.push(f); st.added.push(f.id); continue;
      }
      // on part de la feature la plus sûre, on y fusionne les autres candidates puis l'observation
      cands.sort((a, b) => b.conf * segLen(b) - a.conf * segLen(a));
      let g = cands[0], w = segLen(g) * g.conf;
      const keep = cands[0];
      for (const f of cands.slice(1)) {
        const wf = segLen(f) * f.conf;
        g = mergeSegments(g, f, w, wf); w += wf;
        keep.nObs += f.nObs; keep.firstSeen = Math.min(keep.firstSeen, f.firstSeen); keep.conf = Math.max(keep.conf, f.conf);
        map.features.splice(map.features.indexOf(f), 1); st.removed.push(f.id);
      }
      g = mergeSegments(g, s, w, segLen(s));           // l'observation pèse sa longueur × 1
      Object.assign(keep, { x1: g.x1, y1: g.y1, x2: g.x2, y2: g.y2 });
      if (keep.lastSeen !== t) { keep.nObs++; keep.conf = Math.min(o.confMax, keep.conf + o.confHit); }
      keep.lastSeen = t; keep.n = (keep.n || 0) + (s.n || 0);
      if (!st.merged.includes(keep.id) && !st.added.includes(keep.id)) st.merged.push(keep.id);
    }

    cleanVectorMap(map, o, st);
    map.corners = computeCorners(map.features, o);
    st.removed = st.removed.filter(id => !map.features.some(f => f.id === id));
    return st;
  }

  // Gestion des absences : un faisceau robot -> impact qui traverse une feature (loin de l'impact)
  // prouve que la feature n'est plus là, au moins à cet endroit.
  function absenceCheck(map, robot, pts, o, st) {
    const step = Math.max(1, Math.ceil(pts.length / 900)), hits = new Map();
    for (let k = 0; k < pts.length; k += step) {
      const p = pts[k], ray = seg(robot.x, robot.y, p.x, p.y), L = segLen(ray);
      if (L < 2 * o.absMargin) continue;
      for (const f of map.features) {
        const r = intersect(ray, f);
        if (r.type !== 'point') continue;
        if ((1 - r.t) * L < o.absMargin) continue;                   // le faisceau s'arrête sur la feature : elle est vue
        if (angleBetween(ray, f).lines < o.absMinAngle) continue;    // faisceau rasant : intersection peu fiable
        let h = hits.get(f.id); if (!h) hits.set(f.id, h = []); h.push(r.u);
      }
    }
    for (const [id, us] of hits) {
      const f = map.features.find(x => x.id === id); if (!f) continue;
      const L = segLen(f);
      us.sort((a, b) => a - b);
      // regrouper les traversées voisines en intervalles ; un intervalle compte s'il a assez de faisceaux
      const iv = []; let cur = [us[0], us[0], 1];
      for (let k = 1; k < us.length; k++) {
        if ((us[k] - cur[1]) * L <= o.absClusterGap) { cur[1] = us[k]; cur[2]++; }
        else { iv.push(cur); cur = [us[k], us[k], 1]; }
      }
      iv.push(cur);
      const holes = iv.filter(v => v[2] >= o.absMinRays);
      if (!holes.length) continue;
      // morceaux restants = complément des trous, gardés s'ils sont assez longs
      const pieces = []; let start = 0;
      for (const [a, b] of holes) { if ((a - start) * L >= o.minLen) pieces.push([start, a]); start = b; }
      if ((1 - start) * L >= o.minLen) pieces.push([start, 1]);
      for (const [a, b] of holes) { const [x1, y1] = pointAt(f, a), [x2, y2] = pointAt(f, b); st.ghosts.push(seg(x1, y1, x2, y2)); }
      if (!pieces.length) {
        f.conf -= o.confMiss; st.weakened.push(f.id);
        if (f.conf <= 0) { map.features.splice(map.features.indexOf(f), 1); st.removed.push(f.id); st.ghosts.push(seg(f.x1, f.y1, f.x2, f.y2)); }
        continue;
      }
      const geom = pieces.map(([a, b]) => { const [x1, y1] = pointAt(f, a), [x2, y2] = pointAt(f, b); return seg(x1, y1, x2, y2); });
      if (pieces.length === 1 && pieces[0][0] === 0 && pieces[0][1] === 1) continue;
      Object.assign(f, { x1: geom[0].x1, y1: geom[0].y1, x2: geom[0].x2, y2: geom[0].y2 });
      if (geom.length === 1) st.shortened.push(f.id);
      else {
        st.split.push(f.id);
        for (const g of geom.slice(1)) { const nf = cloneFeature(f, g, map.nextId++); map.features.push(nf); st.split.push(nf.id); }
      }
    }
  }

  // Nettoyage : fusion des redondants, durée de vie, longueur minimale
  function cleanVectorMap(map, o, st = {}) {
    if (o.mergeRedundant) {
      let changed = true;
      while (changed) {
        changed = false;
        outer: for (let i = 0; i < map.features.length; i++) for (let j = i + 1; j < map.features.length; j++) {
          const a = map.features[i], b = map.features[j];
          if (!areClose(a, b, o).close) continue;
          const [k, d] = a.conf * segLen(a) >= b.conf * segLen(b) ? [a, b] : [b, a];
          const g = mergeSegments(k, d, segLen(k) * k.conf, segLen(d) * d.conf);
          Object.assign(k, { x1: g.x1, y1: g.y1, x2: g.x2, y2: g.y2, conf: Math.max(k.conf, d.conf), nObs: Math.max(k.nObs, d.nObs),
                             firstSeen: Math.min(k.firstSeen, d.firstSeen), lastSeen: Math.max(k.lastSeen, d.lastSeen), n: (k.n || 0) + (d.n || 0) });
          map.features.splice(map.features.indexOf(d), 1);
          if (st.removed) st.removed.push(d.id); st.fused = (st.fused || 0) + 1;
          changed = true; break outer;
        }
      }
    }
    map.features = map.features.filter(f => {
      if (o.maxAge > 0 && map.scans - f.lastSeen > o.maxAge) { if (st.expired) st.expired.push(f.id); return false; }
      if (segLen(f) < o.minLen) { if (st.removed) st.removed.push(f.id); return false; }
      return true;
    });
  }

  // Coins : deux segments presque perpendiculaires (angle >= cornerAngle) dont les droites se coupent
  // à moins de cornerDist d'une extrémité de chacun. th = bissectrice orientée vers l'intérieur des segments.
  function computeCorners(features, o = VM_DEFAULTS) {
    const out = [];
    for (let i = 0; i < features.length; i++) for (let j = i + 1; j < features.length; j++) {
      const a = features[i], b = features[j];
      if (angleBetween(a, b).lines < o.cornerAngle) continue;
      const r = intersect(a, b);
      if (r.type !== 'point' && r.type !== 'lines') continue;
      const far = (f) => {                               // extrémité la plus éloignée du coin, si l'autre est proche
        const d1 = Math.hypot(f.x1 - r.x, f.y1 - r.y), d2 = Math.hypot(f.x2 - r.x, f.y2 - r.y);
        if (Math.min(d1, d2) > o.cornerDist) return null;
        return d1 > d2 ? [f.x1, f.y1] : [f.x2, f.y2];
      };
      const fa = far(a), fb = far(b); if (!fa || !fb) continue;
      const ua = [fa[0] - r.x, fa[1] - r.y], ub = [fb[0] - r.x, fb[1] - r.y];
      const la = Math.hypot(...ua), lb = Math.hypot(...ub);
      const th = Math.atan2(ua[1] / la + ub[1] / lb, ua[0] / la + ub[0] / lb) * R2D;
      if (out.some(c => Math.hypot(c.x - r.x, c.y - r.y) < 0.05)) continue;
      out.push({ x: r.x, y: r.y, th, a: a.id, b: b.id, conf: Math.min(a.conf ?? 1, b.conf ?? 1), angle: angleBetween(a, b).lines });
    }
    return out;
  }

  function vectorMapToText(map) {
    let s = '# Carte vectorielle (segments) : x1,y1,x2,y2,confiance,observations,premier_scan,dernier_scan\n';
    for (const f of map.features) s += [f.x1, f.y1, f.x2, f.y2].map(v => v.toFixed(4)).join(',') + `,${f.conf},${f.nObs},${f.firstSeen},${f.lastSeen}\n`;
    s += '# Coins : C,x,y,theta_deg\n';
    for (const c of map.corners) s += `C,${c.x.toFixed(4)},${c.y.toFixed(4)},${c.th.toFixed(2)}\n`;
    return s;
  }
  // Lecture : accepte aussi le format simple x1,y1,x2,y2 (carte initiale connue, ex. murs de la pièce)
  function vectorMapFromText(txt, opt = {}) {
    const o = Object.assign({}, VM_DEFAULTS, opt), map = createVectorMap();
    for (const line of txt.split(/\r?\n/)) {
      const l = line.trim(); if (!l || l.startsWith('#') || l.startsWith('C')) continue;
      const v = l.split(/[,;\s]+/).map(Number);
      if (v.length < 4 || !v.slice(0, 4).every(isFinite)) continue;
      map.features.push(Object.assign(seg(v[0], v[1], v[2], v[3]), {
        id: map.nextId++, conf: isFinite(v[4]) ? v[4] : o.confMax, nObs: isFinite(v[5]) ? v[5] : 1,
        firstSeen: isFinite(v[6]) ? v[6] : 0, lastSeen: isFinite(v[7]) ? v[7] : 0, n: 0 }));
    }
    map.scans = Math.max(0, ...map.features.map(f => f.lastSeen));
    map.corners = computeCorners(map.features, o);
    return map;
  }

  return {
    EPS, EPS_SIN, UNKNOWN, FREE_T, OCC_T,
    polarToGlobal,
    createGrid, worldToCell, cellCenter, inGrid, getCell, bresenham, fillTriangle, updateGrid, scanToGrid, ensureCovers,
    gridStats, gridToJSON, gridFromJSON, gridToPGM,
    seg, segLen, segAngle, angleBetween, pointLineDist, pointSegDist, areClose, meanDirection, mergeSegments,
    intersect, cutSegment, fitLine, fitSegment, extractSegments, mergeAll, segmentsToText, segmentsFromText,
    VM_DEFAULTS, createVectorMap, updateVectorMap, cleanVectorMap, computeCorners, vectorMapToText, vectorMapFromText
  };
});
