/* ============================================================================
 *  lidar_explore.js – Exploration autonome : le robot cartographie seul la salle
 *  Dépend de lidar_core.js (LidarCore), lidar_loc.js (LidarLoc), lidar_maptools.js (LidarMapTools).
 *  Navigateur : window.LidarExplore ; Node : require('./lidar_explore.js').
 *
 *  Principe (exploration par « frontières », Yamauchi 1997) :
 *   1. la carte se construit au fur et à mesure (cartographe de lidar_loc.js : chaque scan est recalé
 *      sur la carte déjà faite, en partant du déplacement commandé) ;
 *   2. une FRONTIÈRE est une cellule libre qui touche une cellule encore inconnue : c'est là qu'il
 *      reste quelque chose à voir ;
 *   3. le robot choisit la frontière la plus intéressante (proche, grande), y va par le plus court
 *      chemin sûr (carte de coût : obstacles gonflés du rayon du robot), en s'arrêtant pour scanner ;
 *   4. pendant le trajet, chaque scan sert aussi de pare-chocs : un obstacle devant (pied de chaise,
 *      personne) arrête l'avance, il est ajouté à la carte de coût et le trajet est recalculé ;
 *   5. plus de frontière atteignable : la salle est entièrement vue. Le robot rentre à son point de
 *      départ (option) et annonce « cartographie terminée ».
 *
 *  Le robot avance par petits pas commandés (rotation sur place, puis avance) : « stop and go ».
 *  Les scans pris à l'arrêt se recalent beaucoup mieux que pendant le mouvement.
 *
 *  Conventions : poses { x, y, th (deg) }, x vers l'avant du robot, y vers sa gauche, th sens trigo.
 * ========================================================================== */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory(require('./lidar_core.js'), require('./lidar_loc.js'), require('./lidar_maptools.js'));
  else root.LidarExplore = factory(root.LidarCore, root.LidarLoc, root.LidarMapTools);
})(typeof self !== 'undefined' ? self : this, function (LC, LL, LM) {
  'use strict';
  const D2R = Math.PI / 180, R2D = 180 / Math.PI;
  const { FREE_T, OCC_T } = LC;
  const normDeg = a => ((a + 180) % 360 + 360) % 360 - 180;
  const COST = LM.COST;

  const EXPLORE_DEFAULTS = {
    robotRadius: 0.17,      // demi-largeur du robot + marge (m)
    inflation: 0.30,        // zone où le trajet préfère s'écarter des obstacles (m)
    minFrontier: 6,         // taille mini d'une frontière (cellules ; 6 × 5 cm = 30 cm)
    maxRadius: 0,           // ne pas explorer au-delà de cette distance du départ (m ; 0 = sans limite)
    goalTol: 0.25,          // but atteint à moins de (m)
    blackR: 0.45,           // une frontière déjà visitée (ou impossible) est ignorée dans ce rayon (m)
    vLin: 0.25,             // vitesse réelle en translation (m/s) – à calibrer
    vRot: 90,               // vitesse réelle de rotation (°/s) – à calibrer
    stepMax: 0.35,          // avance maximale par commande (m)
    rotMax: 60,             // rotation maximale par commande (°)
    headTol: 20,            // écart de cap toléré avant de tourner (°)
    safety: 0.12,           // distance gardée devant le robot (m)
    minMs: 80, maxMs: 1800, // durée d'une commande (le firmware plafonne à 2 s)
    returnHome: true,       // revenir au départ à la fin
    maxCmds: 1500,          // sécurité : nombre maximal de commandes
    goalPatience: 40,       // commandes maximum pour atteindre un but
    lostWait: 6,            // scans « perdu » avant de reculer pour revenir sur la carte connue
    escapeR: 0.35,          // départ trop près d'un obstacle : on autorise à s'en dégager dans ce rayon (m)
    gainW: 1.0              // poids de la taille de la frontière dans le choix (m de trajet par m de frontière)
  };

  function createExplorer(opt = {}) {
    const o = { ...EXPLORE_DEFAULTS, ...opt };
    return { o, phase: 'exploration', start: o.start ? { ...o.start } : { x: 0, y: 0, th: 0 }, goal: null, path: null, kfSeen: -1,
             black: [], virt: [], nCmd: 0, goalCmds: 0, blocked: 0, lost: 0, backedUp: false, last: null, frontiers: [], log: [],
             stats: { goals: 0, blocked: 0, replans: 0, dist: 0, rot: 0 }, done: false, reason: '',
             meas: null, calib: { fwd: [], rot: [], checked: false, msg: '' } };
  }

  // --------------------------------------------------------------------------
  // Carte de coût de l'exploration : celle de lidar_maptools.js + obstacles vus en route (« virtuels »)
  function exploreCostmap(X, grid, mode = {}) {
    const rr = X.o.robotRadius - (mode.shrink || 0);
    const cm = LM.costmap(grid, { robotRadius: rr, inflation: X.o.inflation });
    const r = rr + 0.04, R = Math.ceil(r / cm.res);
    for (const v of (mode.noVirt ? [] : X.virt)) {
      const ci = Math.floor((v.x - cm.x0) / cm.res), cj = Math.floor((v.y - cm.y0) / cm.res);
      for (let dj = -R; dj <= R; dj++) for (let di = -R; di <= R; di++) {
        const i = ci + di, j = cj + dj; if (i < 0 || j < 0 || i >= cm.W || j >= cm.H) continue;
        if (Math.hypot(di, dj) * cm.res <= r) { const k = j * cm.W + i; if (cm.cost[k] < COST.INSCRIBED) cm.cost[k] = COST.INSCRIBED; }
      }
    }
    return cm;
  }

  // Dijkstra depuis la position du robot : longueur (pondérée par le coût) du meilleur trajet vers chaque cellule
  function reachability(cm, pose, o) {
    const W = cm.W, H = cm.H, n = W * H, dist = new Float64Array(n).fill(Infinity), from = new Int32Array(n).fill(-1);
    const si = Math.floor((pose.x - cm.x0) / cm.res), sj = Math.floor((pose.y - cm.y0) / cm.res);
    if (si < 0 || sj < 0 || si >= W || sj >= H) return { dist, from, S: -1 };
    const S = sj * W + si, escR2 = (o.escapeR / cm.res) ** 2;
    const pass = (k, i, j) => {
      const c = cm.cost[k];
      if (c === COST.LETHAL) return false;
      if (c === COST.UNKNOWN) return false;
      if (c === COST.INSCRIBED) return (i - si) ** 2 + (j - sj) ** 2 <= escR2;    // se dégager d'un départ trop près d'un mur
      return true;
    };
    const heap = [], push = (k, f) => { heap.push([f, k]); let c = heap.length - 1; while (c) { const p = (c - 1) >> 1; if (heap[p][0] <= heap[c][0]) break; [heap[p], heap[c]] = [heap[c], heap[p]]; c = p; } };
    const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let c = 0; for (;;) { const l = 2 * c + 1, r = l + 1; let m = c; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === c) break; [heap[m], heap[c]] = [heap[c], heap[m]]; c = m; } } return top; };
    dist[S] = 0; push(S, 0);
    while (heap.length) {
      const [d, k] = pop(); if (d > dist[k]) continue;
      const i = k % W, j = (k - i) / W;
      for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
        if (!di && !dj) continue;
        const a = i + di, b = j + dj; if (a < 0 || b < 0 || a >= W || b >= H) continue;
        const m = b * W + a; if (!pass(m, a, b)) continue;
        if (di && dj && (!pass(j * W + a, a, j) || !pass(b * W + i, i, b))) continue;
        const c = cm.cost[m] >= COST.INSCRIBED ? 252 : cm.cost[m];
        const nd = d + (di && dj ? Math.SQRT2 : 1) * cm.res * (1 + 2 * c / 252);
        if (nd < dist[m]) { dist[m] = nd; from[m] = k; push(m, nd); }
      }
    }
    return { dist, from, S };
  }

  /**
   * Frontières : cellules libres voisines (4-connexité) d'une cellule inconnue, regroupées en amas
   * (8-connexité). Chaque amas : { cells, size, len (m), x, y (centre), best (cellule atteignable la plus proche), d }.
   */
  function frontiers(grid, R, o, start) {
    const W = grid.W, H = grid.H, g = grid.data, isFree = v => v <= FREE_T, isUnk = v => v > FREE_T && v < OCC_T;
    const fr = new Uint8Array(W * H), out = [];
    const far = (i, j) => o.maxRadius > 0 && Math.hypot(grid.x0 + (i + 0.5) * grid.res - start.x, grid.y0 + (j + 0.5) * grid.res - start.y) > o.maxRadius;
    for (let j = 1; j < H - 1; j++) for (let i = 1; i < W - 1; i++) {
      const k = j * W + i; if (!isFree(g[k]) || far(i, j)) continue;
      if (isUnk(g[k - 1]) || isUnk(g[k + 1]) || isUnk(g[k - W]) || isUnk(g[k + W])) fr[k] = 1;
    }
    const seen = new Uint8Array(W * H);
    for (let k0 = 0; k0 < W * H; k0++) {
      if (!fr[k0] || seen[k0]) continue;
      const cells = [], st = [k0]; seen[k0] = 1;
      while (st.length) {
        const k = st.pop(); cells.push(k); const i = k % W, j = (k - i) / W;
        for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
          const a = i + di, b = j + dj; if (a < 0 || b < 0 || a >= W || b >= H) continue;
          const m = b * W + a; if (fr[m] && !seen[m]) { seen[m] = 1; st.push(m); }
        }
      }
      if (cells.length < o.minFrontier) continue;
      let sx = 0, sy = 0, best = -1, bd = Infinity;
      for (const k of cells) {
        const i = k % W, j = (k - i) / W; sx += i; sy += j;
        if (R && R.dist[k] < bd) { bd = R.dist[k]; best = k; }
      }
      const c = k => ({ x: grid.x0 + (k % W + 0.5) * grid.res, y: grid.y0 + (Math.floor(k / W) + 0.5) * grid.res });
      out.push({ cells, size: cells.length, len: cells.length * grid.res, x: grid.x0 + (sx / cells.length + 0.5) * grid.res, y: grid.y0 + (sy / cells.length + 0.5) * grid.res,
                 best: best >= 0 && isFinite(bd) ? { k: best, ...c(best) } : null, d: bd });
    }
    return out;
  }

  // Trajet : remontée de l'arbre de Dijkstra puis lissage par visibilité
  function pathTo(cm, R, k) {
    if (!isFinite(R.dist[k])) return null;
    const W = cm.W, cells = [];
    for (let m = k; m >= 0; m = m === R.S ? -1 : R.from[m]) { cells.push(m); if (cells.length > W * cm.H) return null; }
    cells.reverse();
    const P = cells.map(m => ({ i: m % W, j: Math.floor(m / W), x: cm.x0 + (m % W + 0.5) * cm.res, y: cm.y0 + (Math.floor(m / W) + 0.5) * cm.res }));
    const clear = (p, q) => { let ok = true; LC.bresenham(p.i, p.j, q.i, q.j, (a, b) => { const c = cm.cost[b * W + a]; if (c >= COST.INSCRIBED || c > 160) ok = false; }); return ok; };
    const path = [P[0]]; let a = 0;
    while (a < P.length - 1) { let b = P.length - 1; while (b > a + 1 && !clear(P[a], P[b])) b--; path.push(P[b]); a = b; }
    return path;
  }

  // Place libre devant le robot, d'après le scan courant (points dans le repère du robot)
  function freeAhead(local, o, dir = 1) {
    let free = Infinity;
    const half = o.robotRadius + 0.06;                                   // couloir un peu plus large que le robot (dérive latérale)
    for (const q of local) {
      const r = Math.hypot(q.x, q.y); if (r < 0.12) continue;               // le robot lui-même
      const x = dir * q.x;
      if (x > 0 && Math.abs(q.y) < half) free = Math.min(free, x - o.robotRadius - o.safety);
    }
    return free;
  }

  // place libre dans la direction a (° par rapport à l'avant du robot)
  function freeDir(local, o, a) {
    const c = Math.cos(-a * D2R), s = Math.sin(-a * D2R);
    return freeAhead(local.map(q => ({ x: c * q.x - s * q.y, y: s * q.x + c * q.y })), o);
  }
  const cmdMs = (X, mv, amount) => {
    const o = X.o, ms = mv.startsWith('rotate') ? Math.abs(amount) / o.vRot * 1000 : Math.abs(amount) / o.vLin * 1000;
    return Math.round(Math.max(o.minMs, Math.min(o.maxMs, ms)));
  };
  function command(X, mv, amount) {
    const ms = cmdMs(X, mv, amount), o = X.o;
    const v = mv.startsWith('rotate') ? o.vRot * ms / 1000 : o.vLin * ms / 1000;
    const motion = { forward: { x: v, y: 0, th: 0 }, backward: { x: -v, y: 0, th: 0 }, rotateleft: { x: 0, y: 0, th: v }, rotateright: { x: 0, y: 0, th: -v } }[mv];
    X.nCmd++; X.goalCmds++; X.waits = 0; X.stuck = 0;
    if (mv.startsWith('rotate')) X.stats.rot += v; else X.stats.dist += v;
    X.meas = { pose: X.pose ? { ...X.pose } : null, motion };
    return (X.last = { mv, ms, motion });
  }

  /**
   * Contrôle et auto-calibration sur les premiers mouvements : le recalage des scans mesure le vrai déplacement.
   *  - avance : le robot doit partir droit devant le LIDAR (0° du lidar = avant du robot). Sinon il irait de travers :
   *    on s'arrête en donnant le décalage d'angle à régler ;
   *  - rotation : de même signe que la commande. Sinon le scan est en miroir (option « miroir ») ;
   *  - vitesses : rapport mesuré / commandé -> vLin et vRot corrigés (durées des commandes plus justes).
   * Renvoie un message d'arrêt, ou null.
   */
  function calibrate(X, pose) {
    const M = X.meas; X.meas = null;
    if (!M || !M.pose) return null;
    const rel = LL.relPose(M.pose, pose), C = X.calib, o = X.o;
    if (Math.abs(M.motion.x) > 0.08) {
      const d = Math.hypot(rel.x, rel.y);
      if (d > 0.05) C.fwd.push({ ang: Math.atan2(rel.y * Math.sign(M.motion.x), rel.x * Math.sign(M.motion.x)) * R2D, ratio: d / Math.abs(M.motion.x) });
    } else if (Math.abs(M.motion.th) > 12) C.rot.push(rel.th / M.motion.th);
    const med = a => a.slice().sort((x, y) => x - y)[a.length >> 1];
    if (C.checked && Math.abs(M.motion.x) > 0.08) {                  // ensuite : le robot avance-t-il encore ?
      C.stuck = (C.fwd.length && C.fwd[C.fwd.length - 1].ratio < 0.3) || Math.hypot(rel.x, rel.y) < 0.3 * Math.abs(M.motion.x) ? (C.stuck || 0) + 1 : 0;
      if (C.stuck >= 4) return 'le robot n’avance plus alors qu’il en reçoit l’ordre (coincé contre un obstacle bas, roue qui patine ?)';
    }
    if (!C.checked && C.fwd.length >= 2 && C.rot.length >= 2) {
      C.checked = true;
      const ang = med(C.fwd.map(f => f.ang)), rr = med(C.rot);
      if (rr < 0) return C.msg = 'les rotations mesurées sont à l’envers de la commande : le scan est en miroir. Coche (ou décoche) « miroir » dans les options d’affichage, puis relance.';
      if (Math.abs(ang) > 25) return C.msg = `le robot avance à ${ang.toFixed(0)}° de l’avant du lidar : le 0° du lidar n’est pas l’avant du robot. Mets le décalage d’angle à ${(((-ang) % 360) + 360) % 360 | 0}° de plus dans les options d’affichage, puis relance.`;
      const fl = med(C.fwd.map(f => f.ratio));
      if (fl > 0.4 && fl < 2.5) o.vLin *= fl;
      if (rr > 0.4 && rr < 2.5) o.vRot *= rr;
      C.msg = `cap vérifié (écart ${ang.toFixed(0)}°) · vitesses mesurées : ${o.vLin.toFixed(2)} m/s, ${o.vRot.toFixed(0)}°/s`;
      X.log.push(C.msg);
    }
    return null;
  }

  // Plan : d'abord avec tout ce qu'on sait (obstacles vus en route compris) ; si rien n'est atteignable, on réessaie
  // sans les obstacles vus en route (ils ont pu partir, ou être mal placés), puis en serrant un peu le rayon du robot.
  function choosePlan(X, Mp, local) {
    X.virt = X.virt.filter(v => X.nCmd - v.n < 25);                      // un obstacle vu en route est oublié après 25 commandes
    const modes = [{}, { noVirt: true }, { noVirt: true, shrink: 0.03 }];
    const phase0 = X.phase;
    for (let m = 0; m < modes.length; m++) {
      X.phase = phase0;
      const ok = planWith(X, Mp, modes[m]);
      if (ok) { if (m) { X.log.push(m === 1 ? 'passage rouvert sans les obstacles vus en route' : 'passage étroit : rayon du robot réduit de 3 cm'); if (modes[m].noVirt) X.virt = []; } return true; }
      if (X.phase === 'terminé' && X.home) return false;
    }
    return false;
  }
  function planWith(X, Mp, mode) {
    const o = X.o, pose = Mp.pose;
    const cm = exploreCostmap(X, Mp.grid, mode), R = reachability(cm, pose, o);
    X.cm = cm; X.R = R; X.stats.replans++;
    const F = frontiers(Mp.grid, R, o, X.start);
    const near = (p, L) => L.some(b => Math.hypot(b.x - p.x, b.y - p.y) < o.blackR);
    X.frontiers = F.map(f => ({ x: f.x, y: f.y, len: f.len, ok: !!f.best && !near(f.best, X.black), best: f.best, d: f.d }));
    if (X.phase === 'exploration') {
      let best = null, bs = Infinity;
      for (const f of X.frontiers) {
        if (!f.ok) continue;
        const s = f.d - o.gainW * Math.min(f.len, 3);
        if (s < bs) { bs = s; best = f; }
      }
      if (best) { X.goal = { x: best.best.x, y: best.best.y, k: best.best.k, kind: 'frontière' }; X.path = pathTo(cm, R, best.best.k); X.goalCmds = 0; X.stats.goals++; return true; }
      X.phase = o.returnHome ? 'retour' : 'terminé';
      if (!X.loggedEnd) { X.loggedEnd = true; X.log.push(`plus de frontière atteignable après ${X.nCmd} commandes`); }
    }
    if (X.phase === 'retour') {
      if (Math.hypot(pose.x - X.start.x, pose.y - X.start.y) <= o.goalTol) { X.phase = 'terminé'; X.home = true; return false; }
      const si = Math.floor((X.start.x - cm.x0) / cm.res), sj = Math.floor((X.start.y - cm.y0) / cm.res);
      // départ inaccessible tel quel (cellule « inscrite ») : on prend la cellule atteignable la plus proche
      let k = -1, bd = Infinity;
      for (let dj = -8; dj <= 8; dj++) for (let di = -8; di <= 8; di++) {
        const i = si + di, j = sj + dj; if (i < 0 || j < 0 || i >= cm.W || j >= cm.H) continue;
        const m = j * cm.W + i; if (!isFinite(R.dist[m])) continue;
        const d = Math.hypot(di, dj); if (d < bd) { bd = d; k = m; }
      }
      if (k < 0) { X.phase = 'terminé'; X.unreach = true; return false; }
      X.goal = { x: cm.x0 + (k % cm.W + 0.5) * cm.res, y: cm.y0 + (Math.floor(k / cm.W) + 0.5) * cm.res, k, kind: 'départ' };
      X.path = pathTo(cm, R, k); X.goalCmds = 0; return true;
    }
    return false;
  }

  /**
   * Décision : à appeler quand le robot est arrêté et que le cartographe a intégré un scan pris à l'arrêt.
   * Mp : cartographe (createMapper de lidar_loc.js) ; local : points du dernier scan dans le repère du robot.
   * Renvoie { mv, ms, motion } (commande à envoyer), { wait: true } ou { done: true, reason }.
   */
  function explorerStep(X, Mp, local) {
    const o = X.o;
    if (X.done) return { done: true, reason: X.reason };
    const finish = reason => { X.done = true; X.phase = 'terminé'; X.reason = reason; X.path = null; return { done: true, reason }; };
    if (X.nCmd >= o.maxCmds) return finish(`arrêt de sécurité : ${o.maxCmds} commandes`);
    if (!Mp.grid) return { wait: true };
    // perdu : on ne bouge plus ; s'il ne se retrouve pas, on annule le dernier déplacement
    if (Mp.status === 'perdu') {
      X.lost++;
      if (X.lost >= o.lostWait && X.last && !X.backedUp) {
        X.backedUp = true; const L = X.last;
        const inv = { forward: 'backward', backward: 'forward', rotateleft: 'rotateright', rotateright: 'rotateleft' }[L.mv];
        X.log.push('perdu : retour en arrière'); X.nCmd++;
        return { mv: inv, ms: L.ms, motion: { x: -L.motion.x, y: -L.motion.y, th: -L.motion.th } };
      }
      if (X.lost > o.lostWait * 4) return finish('robot perdu : la carte ne correspond plus au scan');
      return { wait: true };
    }
    X.lost = 0; X.backedUp = false;
    const pose = Mp.pose;
    X.waits = (X.waits || 0) + 1;
    if (X.waits > 60) return finish('arrêt : plus aucun ordre possible depuis 60 scans (robot coincé ?)');
    const bad = calibrate(X, pose); X.pose = { ...pose };
    if (bad) return finish('arrêt : ' + bad);
    // phase de contrôle : petits mouvements SANS prédiction donnée au recalage, pour mesurer sans a priori le vrai
    // déplacement (cap du lidar, miroir, vitesses). Avance seulement s'il y a de la place devant ET derrière
    // (si le lidar était monté à l'envers, « avancer » reculerait).
    if (!X.calib.checked) {
      X.calStep = (X.calStep || 0) + 1;
      if (X.calStep > 14) { X.calib.checked = true; X.log.push('contrôle du cap incomplet (pas assez de place)'); }
      else {
        const nf = X.calib.fwd.length, nr = X.calib.rot.length;
        const room = freeAhead(local, o) > 0.25 && freeAhead(local, o, -1) > 0.25;
        let c;
        if (nf <= nr && room) c = command(X, 'forward', 0.15);
        else c = command(X, (nr % 2 ? 'rotateright' : 'rotateleft'), 20);
        return { ...c, noPrior: true };
      }
    }
    if (X.kfSeen !== Mp.kfs.length || !X.path || X.replan) {          // carte enrichie : on refait le plan
      X.kfSeen = Mp.kfs.length; X.replan = false;
      if (!choosePlan(X, Mp, local)) {
        if (X.phase === 'terminé') return finish(X.home ? 'cartographie terminée, robot revenu au départ' : X.unreach ? 'cartographie terminée (départ inaccessible : arrêt sur place)' : 'cartographie terminée');
      }
      if (!X.path) { X.black.push({ x: X.goal.x, y: X.goal.y }); X.replan = true; return { wait: true }; }
    }
    // suivi du trajet
    while (X.path.length > 1 && Math.hypot(X.path[0].x - pose.x, X.path[0].y - pose.y) < 0.2) X.path.shift();
    const tgt = X.path[0], dg = Math.hypot(X.goal.x - pose.x, X.goal.y - pose.y);
    if (dg <= o.goalTol || (X.path.length === 1 && Math.hypot(tgt.x - pose.x, tgt.y - pose.y) <= o.goalTol)) {
      X.black.push({ x: X.goal.x, y: X.goal.y }); X.path = null; X.replan = true;
      if (X.goal.kind === 'départ') return finish('cartographie terminée, robot revenu au départ');
      return { wait: true };
    }
    if (X.goalCmds > o.goalPatience) { X.log.push('but abandonné (trop long)'); X.black.push({ x: X.goal.x, y: X.goal.y }); X.path = null; X.replan = true; return { wait: true }; }
    const head = Math.atan2(tgt.y - pose.y, tgt.x - pose.x) * R2D, err = normDeg(head - pose.th);
    if (Math.abs(err) > o.headTol) return command(X, err > 0 ? 'rotateleft' : 'rotateright', Math.min(Math.abs(err), o.rotMax));
    const want = Math.min(o.stepMax, Math.hypot(tgt.x - pose.x, tgt.y - pose.y) + 0.05), free = freeAhead(local, o);
    if (free < 0.06) {
      // obstacle devant, absent de la carte (pied de chaise, personne...) : on le note et on recalcule
      X.blocked++; X.stats.blocked++;
      const c = Math.cos(pose.th * D2R), s = Math.sin(pose.th * D2R), ahead = o.robotRadius + o.safety + Math.max(0, free) + 0.05;
      const v = { x: pose.x + c * ahead, y: pose.y + s * ahead, n: X.nCmd };
      if (!X.virt.some(w => Math.hypot(w.x - v.x, w.y - v.y) < 0.1)) X.virt.push(v);
      X.path = null; X.replan = true;
      if (X.blocked >= 3) {
        X.blocked = 0;
        if (freeAhead(local, o, -1) > 0.15) return command(X, 'backward', 0.15);
        // coincé devant ET derrière : on tourne sur place vers la direction la plus dégagée
        let bestA = 0, bestF = -Infinity;
        for (const a of [45, -45, 90, -90, 135, -135]) { const f = freeDir(local, o, a); if (f > bestF + 0.05) { bestF = f; bestA = a; } }
        if (bestF > 0.1) return command(X, bestA > 0 ? 'rotateleft' : 'rotateright', Math.min(Math.abs(bestA), o.rotMax));
        X.stuck = (X.stuck || 0) + 1;
        if (X.stuck > 5) return finish('arrêt : robot coincé de tous les côtés, à dégager à la main');
      }
      return { wait: true };
    }
    X.blocked = 0;
    return command(X, 'forward', Math.min(want, free));
  }

  // --------------------------------------------------------------------------
  // Simulation complète (tests, banc) : robot simulé avec odométrie imparfaite et collisions
  function simMove(scene, pose, motion, rng, o, bias = 1) {
    // le robot réel ne fait pas exactement ce qu'on lui demande : ±10 %, petite dérive, et vitesses mal calibrées (bias)
    const g = () => (rng() - 0.5) * 0.2;
    const real = { x: motion.x * bias * (1 + g()), y: motion.y + (rng() - 0.5) * 0.02 * Math.abs(motion.x), th: motion.th * bias * (1 + g()) + (rng() - 0.5) * 2 * Math.abs(motion.x) };
    const n = 10; let p = { ...pose }, hit = false;
    for (let k = 1; k <= n; k++) {
      const q = LL.compose(pose, { x: real.x * k / n, y: real.y * k / n, th: real.th * k / n });
      if (collides(scene.walls, q, o.robotRadius - 0.05)) { hit = true; break; }
      p = q;
    }
    return { pose: p, hit };
  }
  function collides(walls, p, r) {
    for (const w of walls) {
      const ex = w.x2 - w.x1, ey = w.y2 - w.y1, L2 = ex * ex + ey * ey;
      const t = L2 > 0 ? Math.max(0, Math.min(1, ((p.x - w.x1) * ex + (p.y - w.y1) * ey) / L2)) : 0;
      if (Math.hypot(w.x1 + t * ex - p.x, w.y1 + t * ey - p.y) < r) return true;
    }
    return false;
  }
  /** Lance une exploration simulée complète. Renvoie { X, Mp, truth, collisions, steps, kfErr, poseErr (à chaque décision) }. */
  function simulateExploration(opt = {}) {
    const seed = opt.seed ?? 3, scene = opt.scene || LL.classroomScene(seed, { legs: opt.legs ?? 40 }), rng = LL.mulberry32(seed + 77);
    const start = opt.start || { x: 1.5, y: 1.5, th: 0 };
    const X = createExplorer({ ...opt.explore, start });
    const Mp = LL.createMapper({ tours: 2, start, ...opt.mapper });
    let truth = { ...start }, collisions = 0, steps = 0, motion = null;
    const kfTruth = [], poseErr = [];
    const off = opt.lidarOffset || 0;
    const scan = () => { const s = LL.simulateScan(scene.walls, { ...truth, th: truth.th + off }, { tours: 1 }, rng); if (opt.mirror) s.a = s.a.map(a => (360 - a) % 360); return s; };
    const feed = () => { const s = scan(), r = LL.mapperUpdate(Mp, s, motion); motion = null; if (r.keyframe) kfTruth.push({ est: r.keyframe.pose, truth: { ...truth } }); return s; };
    for (let k = 0; k < 2; k++) feed();
    while (steps < (opt.maxSteps ?? 3000)) {
      steps++;
      const s = feed(), local = LL.localPoints(s);
      if (Mp.field) poseErr.push(LL.poseError(Mp.pose, truth));
      const c = explorerStep(X, Mp, local);
      if (c.done) break;
      if (c.wait) continue;
      const m = simMove(scene, truth, c.motion, rng, X.o, opt.bias ?? 1); truth = m.pose; if (m.hit) { collisions++; if (opt.onHit) opt.onHit({ X, Mp, truth, c, local }); }
      motion = c.noPrior ? null : c.motion;
      if (opt.onStep) opt.onStep({ X, Mp, truth, c });
    }
    if (Mp.pending) { const kf = LL.mapperCommit(Mp); if (kf) kfTruth.push({ est: kf.pose, truth: { ...truth } }); }
    const kfErr = kfTruth.map(t => LL.poseError(t.est, t.truth));
    return { X, Mp, scene, truth, collisions, steps, kfErr, poseErr };
  }
  /** Part de la surface libre réelle (où le robot pourrait se tenir) qui est connue libre sur la carte. */
  function coverage(grid, scene, res = 0.1) {
    let n = 0, ok = 0; const b = scene.bounds;
    for (let y = b.y0 + res / 2; y < b.y1; y += res) for (let x = b.x0 + res / 2; x < b.x1; x += res) {
      if (!scene.free(x, y, 0.05)) continue; n++;
      const i = Math.floor((x - grid.x0) / grid.res), j = Math.floor((y - grid.y0) / grid.res);
      if (i >= 0 && j >= 0 && i < grid.W && j < grid.H && grid.data[j * grid.W + i] <= FREE_T) ok++;
    }
    return n ? ok / n : 0;
  }

  return { EXPLORE_DEFAULTS, createExplorer, explorerStep, exploreCostmap, reachability, frontiers, pathTo, freeAhead, freeDir,
           simulateExploration, simMove, collides, coverage };
});
