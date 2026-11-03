/* ============================================================================
 *  Tests de lidar_core.js – lancer :  node test_lidar_core.js
 *  (les mêmes tests sont exécutables dans lidar_viewer.html, onglet Segments)
 * ========================================================================== */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
    if (require.main === module) {
      let LL = null, LM = null;
      try { LL = require('./lidar_loc.js'); } catch (e) { /* localisation absente */ }
      try { LM = require('./lidar_maptools.js'); } catch (e) { /* traitement absent */ }
      const res = module.exports.runTests(require('./lidar_core.js'), LL, LM);
      for (const r of res.results) console.log(`${r.ok ? '  OK ' : 'ÉCHEC'}  [${r.group}] ${r.name}${r.ok ? '' : '  -> ' + r.msg}`);
      console.log(`\n${res.passed}/${res.total} tests réussis`);
      process.exit(res.passed === res.total ? 0 : 1);
    }
  } else root.LidarTests = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  function runTests(LC, LL, LM) {
    const { seg, EPS } = LC;
    const results = []; let group = '';
    const near = (a, b, tol = 1e-9) => Math.abs(a - b) <= tol;
    const sameSeg = (s, x1, y1, x2, y2, tol = 1e-9) => near(s.x1, x1, tol) && near(s.y1, y1, tol) && near(s.x2, x2, tol) && near(s.y2, y2, tol);
    const fmt = s => `(${[s.x1, s.y1, s.x2, s.y2].map(v => +v.toFixed(6)).join(', ')})`;
    function test(name, fn) {
      try { const m = fn(); results.push({ group, name, ok: m === true || m === undefined, msg: m === true || m === undefined ? '' : String(m) }); }
      catch (e) { results.push({ group, name, ok: false, msg: 'exception : ' + e.message }); }
    }
    const expect = (cond, msg) => cond ? true : msg;

    // ------------------------------------------------------------------ découpe
    group = 'Découpe';
    const T = seg(0, 0, 10, 0);          // segment à couper, premier point (0,0)

    test('croisement à 3 m du premier point : on garde le côté du second (7 m)', () => {
      const r = LC.cutSegment(T, seg(3, -1, 3, 1));
      return expect(r.changed && r.kept === 'second' && sameSeg(r.segment, 3, 0, 10, 0), fmt(r.segment) + ' ' + r.reason);
    });
    test('croisement à 7 m : on garde le côté du premier point (7 m)', () => {
      const r = LC.cutSegment(T, seg(7, -1, 7, 1));
      return expect(r.changed && r.kept === 'premier' && sameSeg(r.segment, 0, 0, 7, 0), fmt(r.segment));
    });
    test('croisement oblique (45°)', () => {
      const r = LC.cutSegment(T, seg(1, -2, 5, 2));
      return expect(r.changed && sameSeg(r.segment, 3, 0, 10, 0, 1e-12), fmt(r.segment));
    });
    test('côtés exactement égaux : côté du premier point', () => {
      const r = LC.cutSegment(T, seg(5, -1, 5, 1));
      return expect(r.changed && r.kept === 'premier' && sameSeg(r.segment, 0, 0, 5, 0), fmt(r.segment));
    });
    test('côtés égaux à EPS près (écart 1e-7 m < EPS) : côté du premier point', () => {
      const x = 5 + 0.5e-7;                       // côté premier = 5,00000005 ; second = 4,99999995 (écart 1e-7)
      const r = LC.cutSegment(T, seg(x, -1, x, 1));
      return expect(r.changed && r.kept === 'premier', r.kept + ' ' + r.reason);
    });
    test('égalité testée dans l’autre sens (second côté plus long de 1e-7 m) : côté du premier point', () => {
      const x = 5 - 0.5e-7;
      const r = LC.cutSegment(T, seg(x, -1, x, 1));
      return expect(r.changed && r.kept === 'premier', r.kept);
    });
    test('écart 1e-5 m > EPS : on garde vraiment le plus long', () => {
      const x = 5 - 0.5e-5;                       // second côté plus long de 1e-5
      const r = LC.cutSegment(T, seg(x, -1, x, 1));
      return expect(r.changed && r.kept === 'second', r.kept);
    });
    test('le « premier point » dépend du sens du segment (segment inversé)', () => {
      const r = LC.cutSegment(seg(10, 0, 0, 0), seg(5, -1, 5, 1));
      return expect(r.changed && sameSeg(r.segment, 10, 0, 5, 0), fmt(r.segment));
    });
    test('intersection exactement au premier point : rien ne change', () => {
      const r = LC.cutSegment(T, seg(0, -1, 0, 1));
      return expect(!r.changed && sameSeg(r.segment, 0, 0, 10, 0), r.reason);
    });
    test('intersection exactement au second point : rien ne change', () => {
      const r = LC.cutSegment(T, seg(10, -1, 10, 1));
      return expect(!r.changed, r.reason);
    });
    test('intersection à 5e-7 m d’une extrémité (< EPS) : considérée à l’extrémité', () => {
      const r = LC.cutSegment(T, seg(5e-7, -1, 5e-7, 1));
      return expect(!r.changed, r.reason);
    });
    test('intersection à 5e-6 m d’une extrémité (> EPS) : découpe', () => {
      const r = LC.cutSegment(T, seg(5e-6, -1, 5e-6, 1));
      return expect(r.changed && r.kept === 'second', r.reason);
    });
    test('cutter qui touche target par son extrémité (T) : découpe', () => {
      const r = LC.cutSegment(T, seg(4, 0, 4, 3));
      return expect(r.changed && sameSeg(r.segment, 4, 0, 10, 0), r.reason);
    });
    test('cutter qui s’arrête à 1e-3 m avant target : rien ne change', () => {
      const r = LC.cutSegment(T, seg(4, 1e-3, 4, 3));
      return expect(!r.changed && r.reason.includes('hors'), r.reason);
    });
    test('cutter qui s’arrête à 5e-7 m de target (< EPS) : découpe', () => {
      const r = LC.cutSegment(T, seg(4, 5e-7, 4, 3));
      return expect(r.changed, r.reason);
    });
    test('droites sécantes mais intersection hors de target : rien ne change', () => {
      const r = LC.cutSegment(T, seg(12, -1, 12, 1));
      return expect(!r.changed, r.reason);
    });
    test('colinéaires qui se chevauchent : rien ne change', () => {
      const r = LC.cutSegment(T, seg(2, 0, 6, 0));
      return expect(!r.changed && r.reason.includes('colinéaires'), r.reason);
    });
    test('colinéaires bout à bout : rien ne change', () => {
      const r = LC.cutSegment(T, seg(10, 0, 15, 0));
      return expect(!r.changed && r.reason.includes('colinéaires'), r.reason);
    });
    test('parallèles distincts : rien ne change', () => {
      const r = LC.cutSegment(T, seg(0, 1, 10, 1));
      return expect(!r.changed && r.reason.includes('parallèles'), r.reason);
    });
    test('presque parallèles (0,01°) qui se croisent dans target : découpe', () => {
      const a = 0.01 * Math.PI / 180;
      const c = seg(4 - 3 * Math.cos(a), -3 * Math.sin(a), 4 + 3 * Math.cos(a), 3 * Math.sin(a));
      const r = LC.cutSegment(T, c);
      return expect(r.changed && near(r.point[0], 4, 1e-6), r.reason + ' ' + r.point);
    });
    test('segment de longueur nulle : rien ne change', () => {
      const r = LC.cutSegment(seg(1, 1, 1, 1), seg(0, 0, 2, 2));
      return expect(!r.changed, r.reason);
    });
    test('segments quelconques non alignés sur les axes', () => {
      const t = seg(1, 1, 7, 4), r = LC.cutSegment(t, seg(2, 4, 4, 0));    // intersection en (3, 2)
      return expect(r.changed && sameSeg(r.segment, 3, 2, 7, 4, 1e-12), fmt(r.segment));
    });

    // ------------------------------------------------------------------ intersection
    group = 'Intersection';
    test('X simple', () => { const r = LC.intersect(seg(0, 0, 2, 2), seg(0, 2, 2, 0)); return expect(r.type === 'point' && near(r.x, 1) && near(r.y, 1), JSON.stringify(r)); });
    test('droites sécantes hors des segments -> type lines', () => { const r = LC.intersect(seg(0, 0, 1, 0), seg(3, -1, 3, 1)); return expect(r.type === 'lines' && !r.onS1 && r.onS2, r.type); });
    test('extrémité commune -> point (t = 1, u = 0)', () => { const r = LC.intersect(seg(0, 0, 1, 0), seg(1, 0, 1, 1)); return expect(r.type === 'point' && near(r.t, 1) && near(r.u, 0), JSON.stringify(r)); });
    test('colinéaires avec recouvrement -> partie commune', () => { const r = LC.intersect(seg(0, 0, 10, 0), seg(12, 0, 6, 0)); return expect(r.type === 'collinear' && r.overlap && sameSeg(r.overlap, 6, 0, 10, 0), JSON.stringify(r)); });
    test('colinéaires disjoints -> pas de partie commune', () => { const r = LC.intersect(seg(0, 0, 1, 0), seg(2, 0, 3, 0)); return expect(r.type === 'collinear' && r.overlap === null, JSON.stringify(r)); });
    test('colinéaires bout à bout -> partie commune réduite à un point', () => { const r = LC.intersect(seg(0, 0, 1, 0), seg(1, 0, 2, 0)); return expect(r.type === 'collinear' && r.overlap && LC.segLen(r.overlap) <= EPS, JSON.stringify(r)); });
    test('parallèles', () => expect(LC.intersect(seg(0, 0, 1, 1), seg(0, 1, 1, 2)).type === 'parallel'));

    // ------------------------------------------------------------------ angle
    group = 'Angle';
    test('perpendiculaires : 90°', () => expect(near(LC.angleBetween(seg(0, 0, 1, 0), seg(0, 0, 0, 1)).lines, 90)));
    test('sens opposés : droites à 0°, angle orienté 180°', () => { const a = LC.angleBetween(seg(0, 0, 1, 0), seg(5, 1, 3, 1)); return expect(near(a.lines, 0) && near(a.oriented, 180), JSON.stringify(a)); });
    test('30° et 150° donnent le même angle entre droites (30°)', () => {
      const s = d => seg(0, 0, Math.cos(d * Math.PI / 180), Math.sin(d * Math.PI / 180));
      const a = LC.angleBetween(seg(0, 0, 1, 0), s(30)).lines, b = LC.angleBetween(seg(0, 0, 1, 0), s(150)).lines;
      return expect(near(a, 30, 1e-9) && near(b, 30, 1e-9), a + ' ' + b);
    });
    test('angle orienté signé (-45°)', () => expect(near(LC.angleBetween(seg(0, 0, 1, 0), seg(0, 0, 1, -1)).oriented, -45)));

    // ------------------------------------------------------------------ proximité et fusion
    group = 'Proximité / fusion';
    const opt = { angTol: 5, distTol: 0.08, gapTol: 0.15 };
    test('même mur, légèrement décalé et en recouvrement : proches', () => expect(LC.areClose(seg(0, 0, 3, 0), seg(2, 0.03, 5, 0.05), opt).close));
    test('même direction, sens opposé : proches', () => expect(LC.areClose(seg(0, 0, 3, 0), seg(5, 0.02, 2.5, 0.02), opt).close));
    test('angle trop grand (10°) : pas proches', () => { const r = LC.areClose(seg(0, 0, 3, 0), seg(2, 0, 5, 0.53), opt); return expect(!r.close && !r.angOk, JSON.stringify(r)); });
    test('parallèles à 30 cm : pas proches (écart latéral)', () => { const r = LC.areClose(seg(0, 0, 3, 0), seg(0, 0.3, 3, 0.3), opt); return expect(!r.close && !r.latOk); });
    test('alignés mais trou de 1 m : pas proches (écart longitudinal)', () => { const r = LC.areClose(seg(0, 0, 3, 0), seg(4, 0, 6, 0), opt); return expect(!r.close && !r.gapOk && near(r.gap, 1, 1e-9), JSON.stringify(r)); });
    test('alignés, trou de 10 cm : proches', () => expect(LC.areClose(seg(0, 0, 3, 0), seg(3.1, 0, 6, 0), opt).close));
    test('petit morceau bruité (2° d’inclinaison) au bout d’un long mur : proches', () => {
      const r = LC.areClose(seg(0.5, 5, 8, 5), seg(0.48, 5.01, -0.01, 4.99), opt);
      return expect(r.close, JSON.stringify(r));
    });
    test('fusion de deux morceaux alignés -> un seul segment couvrant les deux', () => {
      const m = LC.mergeSegments(seg(0, 0, 3, 0), seg(2, 0, 6, 0));
      return expect(sameSeg(m, 0, 0, 6, 0, 1e-12), fmt(m));
    });
    test('fusion avec sens opposé : le résultat garde le sens du premier', () => {
      const m = LC.mergeSegments(seg(0, 0, 3, 0), seg(6, 0, 2, 0));
      return expect(sameSeg(m, 0, 0, 6, 0, 1e-12), fmt(m));
    });
    test('fusion pondérée par la longueur (le long segment domine)', () => {
      const m = LC.mergeSegments(seg(0, 0, 9, 0), seg(9, 0.1, 10, 0.1));    // 9 m à y=0, 1 m à y=0,1
      return expect(near(m.y1, 0.01, 1e-9) && near(m.y2, 0.01, 1e-9) && near(m.x1, 0) && near(m.x2, 10), fmt(m));
    });
    test('mergeAll : 3 morceaux d’un même mur -> 1 segment', () => {
      const r = LC.mergeAll([seg(0, 0, 2, 0), seg(1.9, 0.01, 4, 0.01), seg(4.05, 0, 6, 0), seg(0, 3, 0, 5)], opt);
      return expect(r.segments.length === 2 && r.merges === 2, r.segments.length + ' segments');
    });

    // ------------------------------------------------------------------ extraction
    group = 'Extraction de segments';
    // scan simulé : robot en (2, 1.5) dans une pièce 6 x 4 m, un poteau 20 x 20 cm
    const ROOM = [seg(0, 0, 6, 0), seg(6, 0, 6, 4), seg(6, 4, 0, 4), seg(0, 4, 0, 0),
                  seg(4, 2, 4.2, 2), seg(4.2, 2, 4.2, 2.2), seg(4.2, 2.2, 4, 2.2), seg(4, 2.2, 4, 2)];
    function simScan(robot, noise, walls = ROOM) {
      let rnd = 12345; const rand = () => ((rnd = (rnd * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
      const pts = [];
      for (let a = 0; a < 360; a += 0.8) {
        const dx = Math.cos(a * Math.PI / 180), dy = Math.sin(a * Math.PI / 180);
        const ray = seg(robot.x, robot.y, robot.x + 20 * dx, robot.y + 20 * dy);
        let best = Infinity;
        for (const w of walls) { const r = LC.intersect(ray, w); if (r.type === 'point' && r.t * 20 < best) best = r.t * 20; }
        const d = best + (rand() - 0.5) * 2 * noise;
        pts.push({ x: robot.x + d * dx, y: robot.y + d * dy });
      }
      return pts;
    }
    // le poteau masque une partie du mur de droite : ce mur sort en 2 morceaux (occlusion),
    // qui se regroupent sur une même droite. On compte donc les droites distinctes.
    const wallLines = S => LC.mergeAll(S.filter(s => LC.segLen(s) > 1), { angTol: 2, distTol: 0.05, gapTol: 5 }).segments;
    test('pièce rectangulaire sans bruit : 4 murs retrouvés (le mur masqué par le poteau en 2 morceaux)', () => {
      const S = LC.extractSegments(simScan({ x: 2, y: 1.5 }, 0), { x: 2, y: 1.5 });
      const long = S.filter(s => LC.segLen(s) > 1), lines = wallLines(S);
      return expect(lines.length === 4 && long.length === 5, `${long.length} segments longs sur ${lines.length} droites`);
    });
    test('faces visibles du poteau (20 cm) détectées', () => {
      const S = LC.extractSegments(simScan({ x: 2, y: 1.5 }, 0), { x: 2, y: 1.5 }, { minLen: 0.1, minPts: 4 });
      const post = S.filter(s => LC.segLen(s) < 0.3 && s.x1 > 3.9 && s.x1 < 4.3);
      return expect(post.length >= 1, `${post.length} face(s)`);
    });
    test('avec bruit ±1,5 cm : toujours 4 murs (droites), orientés à moins de 1° des axes', () => {
      const S = LC.extractSegments(simScan({ x: 2, y: 1.5 }, 0.015), { x: 2, y: 1.5 });
      const long = S.filter(s => LC.segLen(s) > 1);
      const ok = long.every(s => { const a = LC.angleBetween(s, seg(0, 0, 1, 0)).lines; return a < 1 || a > 89; });
      return expect(wallLines(S).length === 4 && ok, `${long.length} murs, angles ${long.map(s => LC.angleBetween(s, seg(0, 0, 1, 0)).lines.toFixed(2))}`);
    });
    test('mur du bas retrouvé à moins de 1 cm de y = 0', () => {
      const S = LC.extractSegments(simScan({ x: 2, y: 1.5 }, 0.015), { x: 2, y: 1.5 });
      const b = S.find(s => Math.abs(s.y1) < 0.2 && Math.abs(s.y2) < 0.2 && LC.segLen(s) > 1);
      return expect(b && Math.abs((b.y1 + b.y2) / 2) < 0.01, b ? fmt(b) : 'mur absent');
    });

    // ------------------------------------------------------------------ grid map
    group = 'Grid map';
    test('dimensions : 2 m × 3 m à 0,1 m -> 20 × 30 cellules', () => { const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 2, yMax: 3, res: 0.1 }); return expect(g.W === 20 && g.H === 30, g.W + '×' + g.H); });
    test('conversion monde -> cellule (bords et valeurs négatives)', () => {
      const g = LC.createGrid({ xMin: -1, yMin: -1, xMax: 1, yMax: 1, res: 0.1 });
      const a = LC.worldToCell(g, -1, -1), b = LC.worldToCell(g, 0.05, -0.05), c = LC.worldToCell(g, 0.999, 0.999);
      return expect(a[0] === 0 && a[1] === 0 && b[0] === 10 && b[1] === 9 && c[0] === 19 && c[1] === 19, JSON.stringify([a, b, c]));
    });
    test('Bresenham : extrémités incluses, cellules contiguës, sens indifférent', () => {
      const A = [], B = [];
      LC.bresenham(0, 0, 7, -3, (i, j) => A.push(i + ',' + j)); LC.bresenham(7, -3, 0, 0, (i, j) => B.push(i + ',' + j));
      const contig = A.every((c, k) => { if (!k) return true; const [i, j] = c.split(',').map(Number), [p, q] = A[k - 1].split(',').map(Number); return Math.abs(i - p) <= 1 && Math.abs(j - q) <= 1; });
      return expect(A[0] === '0,0' && A[A.length - 1] === '7,-3' && A.length === 8 && contig && B.length === 8, A.join(' '));
    });
    test('un faisceau : cellules traversées libres, cellule d’impact occupée', () => {
      const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 2, yMax: 1, res: 0.1 });
      LC.updateGrid(g, { x: 0.05, y: 0.55 }, [{ x: 1.55, y: 0.55 }], { model: 'binary' });
      const row = []; for (let i = 0; i < 20; i++) row.push(LC.getCell(g, i, 5));
      const ok = row.slice(0, 15).every(v => v === 0) && row[15] === 255 && row.slice(16).every(v => v === LC.UNKNOWN);
      return expect(ok, row.join(' '));
    });
    test('mise à jour probabiliste : valeurs saturées dans [0, 255]', () => {
      const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 1, yMax: 1, res: 0.1 });
      for (let k = 0; k < 20; k++) LC.updateGrid(g, { x: 0.05, y: 0.05 }, [{ x: 0.95, y: 0.05 }], { free: 15, occ: 60 });
      return expect(LC.getCell(g, 9, 0) === 255 && LC.getCell(g, 3, 0) === 0, LC.getCell(g, 9, 0) + ' ' + LC.getCell(g, 3, 0));
    });
    test('une cellule d’impact n’est pas libérée par un autre faisceau du même scan', () => {
      const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 2, yMax: 1, res: 0.1 });
      // impact en (1.05, 0.55) ; un second faisceau plus long traverse la même cellule
      LC.updateGrid(g, { x: 0.05, y: 0.55 }, [{ x: 1.05, y: 0.55 }, { x: 1.85, y: 0.56 }], { model: 'binary' });
      return expect(LC.getCell(g, 10, 5) === 255, String(LC.getCell(g, 10, 5)));
    });
    test('point hors de la carte : le rayon est quand même libéré dans la carte', () => {
      const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 1, yMax: 1, res: 0.1 });
      const st = LC.updateGrid(g, { x: 0.05, y: 0.55 }, [{ x: 3, y: 0.55 }], { model: 'binary' });
      return expect(st.outside === 1 && LC.getCell(g, 9, 5) === 0, JSON.stringify(st));
    });
    test('scan d’une pièce : murs occupés, centre libre, extérieur inconnu', () => {
      const pts = simScan({ x: 2, y: 1.5 }, 0.01);
      const { grid: g } = LC.scanToGrid({ x: 2, y: 1.5 }, pts, { res: 0.05, margin: 0.5 });
      const v = (x, y) => LC.getCell(g, ...LC.worldToCell(g, x, y));
      return expect(v(3, 0.0) >= LC.OCC_T && v(3, 2) <= LC.FREE_T && v(-0.3, 2) === LC.UNKNOWN, [v(3, 0), v(3, 2), v(-0.3, 2)].join(' '));
    });
    test('remplissage entre faisceaux : plus de cellules libres, impacts conservés', () => {
      // faisceaux espacés de 2° : loin du robot, Bresenham seul laisse des trous
      const robot = { x: 2, y: 1.5 }, all = simScan(robot, 0), sparse = all.filter((_, k) => k % 3 === 0);
      const a = LC.scanToGrid(robot, sparse, { res: 0.05 }).grid, b = LC.scanToGrid(robot, sparse, { res: 0.05, fillBetween: true }).grid;
      const fa = LC.gridStats(a).free, fb = LC.gridStats(b).free;
      const impactsOk = sparse.every(q => LC.getCell(b, ...LC.worldToCell(b, q.x, q.y)) >= LC.OCC_T);
      return expect(fb > fa * 1.05 && impactsOk, `libres ${fa} -> ${fb}, impacts ${impactsOk}`);
    });
    test('remplissage : le mur ne se troue pas entre deux impacts éloignés', () => {
      const robot = { x: 2, y: 1.5 }, sparse = simScan(robot, 0).filter((_, k) => k % 3 === 0);
      const g = LC.scanToGrid(robot, sparse, { res: 0.05, fillBetween: true }).grid;
      // mur du haut (y = 4) entre x = 0,5 et 5,5 : aucune cellule ne doit être marquée libre
      let freed = 0;
      for (let x = 0.5; x <= 5.5; x += 0.05) { const v = LC.getCell(g, ...LC.worldToCell(g, x, 4.0)); if (v <= LC.FREE_T) freed++; }
      return expect(freed === 0, freed + ' cellules du mur libérées');
    });
    test('remplissage : pas de remplissage à travers une discontinuité (derrière le poteau)', () => {
      const robot = { x: 2, y: 1.5 }, g = LC.scanToGrid(robot, simScan(robot, 0), { res: 0.05, fillBetween: true }).grid;
      // point juste derrière le poteau (ombre), vu depuis le robot : doit rester inconnu
      return expect(LC.getCell(g, ...LC.worldToCell(g, 4.6, 2.35)) === LC.UNKNOWN, String(LC.getCell(g, ...LC.worldToCell(g, 4.6, 2.35))));
    });
    test('agrandissement de la carte : contenu conservé au même endroit', () => {
      const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 1, yMax: 1, res: 0.1 });
      g.data[3 * g.W + 4] = 200;
      const n = LC.ensureCovers(g, -1, -0.5, 2, 1);
      return expect(LC.getCell(n, ...LC.worldToCell(n, 0.45, 0.35)) === 200 && n.W > g.W, n.W + '×' + n.H);
    });
    test('export / import JSON sans perte', () => {
      const { grid: g } = LC.scanToGrid({ x: 2, y: 1.5 }, simScan({ x: 2, y: 1.5 }, 0.01), { res: 0.1 });
      const h = LC.gridFromJSON(JSON.parse(JSON.stringify(LC.gridToJSON(g))));
      return expect(h.W === g.W && h.data.every((v, k) => v === g.data[k]));
    });

    // ------------------------------------------------------------------ carte vectorielle
    group = 'Carte vectorielle';
    // salle 8 x 5 m ; mur du haut en deux parties autour d'une porte (x de 3 à 4) éventuellement fermée
    const hall = (doorClosed, box) => {
      // mur de couloir en y = 7 : visible seulement par la porte ouverte
      const w = [seg(0, 0, 8, 0), seg(8, 0, 8, 5), seg(8, 5, 4, 5), seg(3, 5, 0, 5), seg(0, 5, 0, 0), seg(-2, 7, 10, 7)];
      if (doorClosed) w.push(seg(4, 5, 3, 5));
      if (box) w.push(seg(box[0], box[1], box[0] + 1.5, box[1]), seg(box[0] + 1.5, box[1], box[0] + 1.5, box[1] + 0.6),
                      seg(box[0] + 1.5, box[1] + 0.6, box[0], box[1] + 0.6), seg(box[0], box[1] + 0.6, box[0], box[1]));
      return w;
    };
    const R1 = { x: 2, y: 2 }, R2 = { x: 6, y: 3 };
    const longF = M => M.features.filter(f => LC.segLen(f) > 1);
    const findF = (M, pred) => M.features.filter(pred);
    const onTop = f => Math.abs(f.y1 - 5) < 0.1 && Math.abs(f.y2 - 5) < 0.1;

    test('carte vide + 1 scan : les murs deviennent des features (confiance 1, 1 observation)', () => {
      const M = LC.createVectorMap(), st = LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      const ok = longF(M).length >= 4 && M.features.every(f => f.conf === 1 && f.nObs === 1 && f.firstSeen === 1);
      return expect(ok && st.added.length === M.features.length, `${M.features.length} features, ${st.added.length} ajoutées`);
    });
    test('même scan intégré deux fois : aucune nouvelle feature, confiance 2', () => {
      const M = LC.createVectorMap(), pts = simScan(R1, 0.01, hall(true));
      LC.updateVectorMap(M, R1, pts); const n = M.features.length;
      const st = LC.updateVectorMap(M, R1, pts);
      return expect(M.features.length === n && st.added.length === 0 && M.features.every(f => f.conf === 2 && f.nObs === 2 && f.lastSeen === 2),
                    `${n} -> ${M.features.length}, ajoutées ${st.added.length}`);
    });
    test('deux poses différentes : les murs vus deux fois sont fusionnés, pas dupliqués', () => {
      const M = LC.createVectorMap();
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      LC.updateVectorMap(M, R2, simScan(R2, 0.01, hall(true)));
      const bottom = findF(M, f => Math.abs(f.y1) < 0.1 && Math.abs(f.y2) < 0.1 && LC.segLen(f) > 1);
      return expect(bottom.length === 1 && bottom[0].nObs === 2 && LC.segLen(bottom[0]) > 7.5, `${bottom.length} murs du bas, L = ${bottom[0] && LC.segLen(bottom[0]).toFixed(2)}`);
    });
    test('porte ouverte : le mur du haut est scindé en deux, l’ouverture correspond à la porte', () => {
      const M = LC.createVectorMap();
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      const top0 = findF(M, onTop);
      const st = LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(false)), { mergeRedundant: false });
      const top = findF(M, onTop).sort((a, b) => Math.min(a.x1, a.x2) - Math.min(b.x1, b.x2));
      const gapL = top.length === 2 ? Math.max(top[0].x1, top[0].x2) : NaN, gapR = top.length === 2 ? Math.min(top[1].x1, top[1].x2) : NaN;
      return expect(top0.length === 1 && st.split.length >= 2 && top.length === 2 && Math.abs(gapL - 3) < 0.15 && Math.abs(gapR - 4) < 0.15,
                    `avant ${top0.length}, après ${top.length} morceaux, ouverture ${gapL.toFixed(2)} -> ${gapR.toFixed(2)}`);
    });
    test('obstacle déplacé : le segment devenu traversé est raccourci ou supprimé', () => {
      const M = LC.createVectorMap();
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true, [4.5, 1.5])));       // caisse en (4.5..6, 1.5..2.1)
      const face = findF(M, f => Math.abs(f.x1 - 4.5) < 0.1 && Math.abs(f.x2 - 4.5) < 0.1);
      // la caisse disparaît : les faisceaux traversent son ancienne face
      let st; for (let k = 0; k < 3; k++) st = LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      const after = findF(M, f => Math.abs(f.x1 - 4.5) < 0.1 && Math.abs(f.x2 - 4.5) < 0.1);
      return expect(face.length === 1 && after.length === 0, `face avant ${face.length}, après ${after.length}`);
    });
    test('feature entièrement traversée : confiance diminuée, puis supprimée à 0', () => {
      const M = LC.createVectorMap();
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true, [4.5, 1.5])));
      const id = findF(M, f => Math.abs(f.x1 - 4.5) < 0.1 && Math.abs(f.x2 - 4.5) < 0.1)[0].id;
      const st = LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      return expect(st.weakened.includes(id) && st.removed.includes(id) && !M.features.some(f => f.id === id), JSON.stringify({ w: st.weakened, r: st.removed }));
    });
    test('confiance élevée : une seule absence ne supprime pas la feature', () => {
      const M = LC.createVectorMap(), withBox = simScan(R1, 0.01, hall(true, [4.5, 1.5]));
      for (let k = 0; k < 3; k++) LC.updateVectorMap(M, R1, withBox);             // vue 3 fois : confiance 3
      const f = findF(M, f => Math.abs(f.x1 - 4.5) < 0.1 && Math.abs(f.x2 - 4.5) < 0.1)[0];
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      return expect(f && M.features.includes(f) && f.conf === 2, f ? 'conf ' + f.conf : 'face absente');
    });
    test('mur hors du champ de vision : ni traversé ni modifié', () => {
      const M = LC.createVectorMap();
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      const before = JSON.stringify(findF(M, f => Math.abs(f.y1) < 0.1 && Math.abs(f.y2) < 0.1));
      // nouveau scan ne contenant que les points de la moitié haute (le mur du bas n'est pas vu)
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)).filter(p => p.y > 2));
      const bottom = findF(M, f => Math.abs(f.y1) < 0.1 && Math.abs(f.y2) < 0.1);
      return expect(bottom.length === 1 && bottom[0].conf === 1 && before.includes(`"x1":${bottom[0].x1}`), JSON.stringify(bottom.map(f => f.conf)));
    });
    test('durée de vie : feature non revue depuis plus de maxAge scans supprimée', () => {
      const M = LC.createVectorMap();
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      const n0 = M.features.length;
      let st; for (let k = 0; k < 3; k++) st = LC.updateVectorMap(M, R1, [], { maxAge: 2 });   // scans vides
      return expect(n0 > 0 && M.features.length === 0 && st.expired.length === n0, `${n0} -> ${M.features.length}`);
    });
    test('coins : les 4 coins de la salle trouvés à 2 cm près, angle ≈ 90°', () => {
      const M = LC.createVectorMap();
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      LC.updateVectorMap(M, R2, simScan(R2, 0.01, hall(true)));
      const want = [[0, 0], [8, 0], [8, 5], [0, 5]];
      const found = want.map(([x, y]) => M.corners.find(c => Math.hypot(c.x - x, c.y - y) < 0.02));
      return expect(found.every(Boolean) && found.every(c => Math.abs(c.angle - 90) < 2), `${M.corners.length} coins, trouvés ${found.filter(Boolean).length}/4`);
    });
    test('orientation d’un coin : bissectrice vers l’intérieur (coin (0,0) -> 45°)', () => {
      const C = LC.computeCorners([Object.assign(seg(0, 0, 3, 0), { id: 1 }), Object.assign(seg(0, 2, 0, 0), { id: 2 })]);
      return expect(C.length === 1 && near(C[0].th, 45, 1e-9) && near(C[0].x, 0) && near(C[0].y, 0), JSON.stringify(C));
    });
    test('fichier texte : aller-retour avec propriétés, et lecture du format simple x1,y1,x2,y2', () => {
      const M = LC.createVectorMap();
      LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true))); LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      const N = LC.vectorMapFromText(LC.vectorMapToText(M));
      const same = N.features.length === M.features.length && N.features.every((f, k) => near(f.x1, M.features[k].x1, 1e-4) && f.conf === M.features[k].conf && f.nObs === M.features[k].nObs);
      const S2 = LC.vectorMapFromText('# murs connus\n0,0,8,0\n8,0,8,5\n');
      return expect(same && S2.features.length === 2 && S2.features[0].conf === LC.VM_DEFAULTS.confMax && S2.corners.length === 1, `${N.features.length}/${M.features.length}, simple ${S2.features.length}`);
    });
    test('carte initiale connue + scan : les murs connus sont renforcés, pas dupliqués', () => {
      const M = LC.vectorMapFromText('0,0,8,0\n8,0,8,5\n0,5,0,0\n');
      const st = LC.updateVectorMap(M, R1, simScan(R1, 0.01, hall(true)));
      const left = findF(M, f => Math.abs(f.x1) < 0.1 && Math.abs(f.x2) < 0.1);
      return expect(left.length === 1 && left[0].nObs === 2 && st.merged.length >= 3, `${left.length} murs gauches, fusions ${st.merged.length}`);
    });

    // ------------------------------------------------------------------ localisation (Jour 2)
    if (LL) {
      group = 'Localisation';
      test('pose relative et composition : a ∘ (a⁻¹ ∘ b) = b', () => {
        const a = { x: 1, y: 2, th: 30 }, b = { x: -0.5, y: 4, th: -100 }, c = LL.compose(a, LL.relPose(a, b));
        return expect(near(c.x, b.x, 1e-9) && near(c.y, b.y, 1e-9) && near(c.th, b.th, 1e-9), JSON.stringify(c));
      });
      test('perturbation : distance et angle exacts, reproductible avec la même graine', () => {
        const t = { x: 0, y: 0, th: 0 }, p1 = LL.perturb(t, 0.5, 20, LL.mulberry32(3)), p2 = LL.perturb(t, 0.5, 20, LL.mulberry32(3));
        return expect(near(Math.hypot(p1.x, p1.y), 0.5, 1e-12) && near(Math.abs(p1.th), 20, 1e-12) && p1.x === p2.x && p1.th === p2.th, JSON.stringify(p1));
      });
      test('champ de distance : distance euclidienne exacte à une cellule occupée', () => {
        const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 2, yMax: 2, res: 0.1 }); g.data[5 * g.W + 5] = 255;
        const F = LL.distanceField(g);
        return expect(near(F.dist[5 * g.W + 5], 0) && near(F.dist[8 * g.W + 9], 0.5, 1e-6) && near(F.dist[5 * g.W + 15], 1.0, 1e-6), F.dist[8 * g.W + 9]);
      });
      // scène simulée commune : carte construite avec 7 scans à poses exactes, un scan à localiser
      const sim = (() => {
        const sc = LL.classroomScene(3), rng = LL.mulberry32(5);
        const mp = [{ x: 2, y: 1.5, th: 10 }, { x: 6.5, y: 1.5, th: 80 }, { x: 11, y: 1.2, th: -30 }, { x: 2, y: 5.5, th: 170 }, { x: 6.5, y: 5, th: -100 }, { x: 12, y: 6, th: 45 }, { x: 2.5, y: 8.5, th: 0 }];
        const R = LL.buildReferences(mp.map(p => Object.assign(LL.simulateScan(sc.walls, p, { tours: 3 }, rng), { pose: p })), -1);
        const truth = { x: 7.3, y: 3.2, th: 33 }, pts = LL.uniformSubsample(LL.localPoints(LL.simulateScan(sc.walls, truth, { tours: 1 }, rng)), { max: 350 });
        const rp = { x: 7.7, y: 3.4, th: 18 }, all = LL.localPoints(LL.simulateScan(sc.walls, rp, { tours: 3 }, rng));
        return { R, truth, pts, ref: { pts: LL.uniformSubsample(all, { cell: 0.03, minR: 0.3, max: 3000 }), ptsAll: all }, rel: LL.relPose(rp, truth) };
      })();
      const ok = (r, t, tp = 0.05, ta = 2) => { const e = LL.poseError(r, t); return expect(e.pos <= tp && e.ang <= ta, `${(e.pos * 100).toFixed(1)} cm, ${e.ang.toFixed(2)}°`); };
      const near1 = LL.perturb(sim.truth, 0.2, 8, LL.mulberry32(1)), far1 = LL.perturb(sim.truth, 1.0, 40, LL.mulberry32(2));
      test('grid map · recherche exhaustive (départ à 1 m / 40°)', () => ok(LL.gridBrute(sim.R.field, sim.pts, far1), sim.truth));
      test('grid map · multi-résolution (départ à 1 m / 40°)', () => ok(LL.gridMultiRes(sim.R.field, sim.pts, far1), sim.truth));
      test('grid map · descente de gradient (départ à 20 cm / 8°)', () => ok(LL.gridGradient(sim.R.field, sim.pts, near1), sim.truth));
      test('carte vectorielle · recherche exhaustive (départ à 1 m / 40°)', () => ok(LL.vecBrute(sim.R.index, sim.pts, far1), sim.truth));
      test('carte vectorielle · ICP point-segment (départ à 20 cm / 8°)', () => ok(LL.vecICP(sim.R.index, sim.pts, near1), sim.truth));
      test('carte vectorielle · calcul direct (sans pose initiale)', () => ok(LL.vecDirect(sim.R.index, sim.pts, { x: 0, y: 0, th: 0 }), sim.truth));
      const relNear = LL.perturb(sim.rel, 0.2, 8, LL.mulberry32(4)), relFar = LL.perturb(sim.rel, 1.0, 40, LL.mulberry32(6));
      test('entre scans · ICP point-à-point (départ à 20 cm / 8°)', () => ok(LL.scanICPPoint(sim.ref, sim.pts, relNear), sim.rel));
      test('entre scans · ICP point-à-ligne (départ à 20 cm / 8°)', () => ok(LL.scanICPLine(sim.ref, sim.pts, relNear), sim.rel));
      test('entre scans · corrélation multi-résolution (départ à 1 m / 40°)', () => ok(LL.scanCorrelative(sim.ref, sim.pts, relFar), sim.rel));
      test('méthode locale hors de son domaine : le gradient ne converge pas depuis 1 m / 40° (attendu)', () => {
        const e = LL.poseError(LL.gridGradient(sim.R.field, sim.pts, far1), sim.truth); return expect(e.pos > 0.05 || e.ang > 2, 'a convergé quand même');
      });
      // robot en direct : carte fixe, aucune pose de départ, puis suivi et robot « kidnappé »
      const live = (() => { const D = LL.demoScenario(3); return { D, M: LL.prepareMap(D.refs.grid, D.refs.vmap.features) }; })();
      const lvScan = (p) => LL.uniformSubsample(LL.localPoints(LL.simulateScan(live.D.scene.walls, p, { tours: 1 }, live.D.rng)), { max: 350 });
      test('robot en direct · localisation globale sans pose de départ (toute la carte, 360°), jugée « sûr »', () => {
        const truth = { x: 7.3, y: 3.2, th: 123 }, g = LL.globalLocalize(live.M, lvScan(truth)), e = LL.poseError(g.pose, truth);
        return expect(e.pos <= 0.05 && e.ang <= 2 && g.confidence === 'sûr', `${(e.pos * 100).toFixed(1)} cm ${e.ang.toFixed(2)}° ${g.confidence}`);
      });
      test('robot en direct · suivi sur 40 pas, puis robot déplacé à la main : relocalisé au scan suivant', () => {
        const T = LL.createTracker(live.M), P = live.D.path; let okN = 0;
        for (let k = 0; k < 40; k++) { LL.trackerUpdate(T, lvScan(P[k]), k ? LL.relPose(P[k - 1], P[k]) : null); const e = LL.poseError(T.pose, P[k]); if (e.pos <= 0.05 && e.ang <= 2) okN++; }
        const far = P[Math.floor(P.length * 0.7)];
        const r = LL.trackerUpdate(T, lvScan(far), null), e = LL.poseError(T.pose, far);
        return expect(okN === 40 && e.pos <= 0.05 && e.ang <= 2 && r.mode === 'relocalisation', `${okN}/40, après kidnapping ${(e.pos * 100).toFixed(1)} cm (${r.mode}, ${r.status})`);
      });
      test('robot en direct · la part de points qui collent à la carte chute quand la pose est fausse', () => {
        const truth = live.D.path[10], pts = lvScan(truth), good = LL.inlierRatio(live.M.field, pts, truth), bad = LL.inlierRatio(live.M.field, pts, { ...truth, x: truth.x + 0.5, th: truth.th + 20 });
        return expect(good > 0.9 && bad < 0.5, `${good.toFixed(2)} / ${bad.toFixed(2)}`);
      });
      test('banc : mêmes poses initiales pour toutes les méthodes, un essai par cas et par niveau', () => {
        // (exécution synchrone d'une mini configuration via la fonction résumé)
        const rows = [{ ref: 'grid', method: 'a', label: 'A', level: 0, dp: 0, da: 0, epos: 0.01, eang: 0.1, ms: 1, ok: true, okCoarse: true },
                      { ref: 'grid', method: 'a', label: 'A', level: 0, dp: 0, da: 0, epos: 0.3, eang: 9, ms: 3, ok: false, okCoarse: false }];
        const S = LL.summarize(rows);
        return expect(S.length === 1 && S[0].success === 0.5 && near(S[0].meanMs, 2), JSON.stringify(S[0]));
      });
    }


    if (LM && LL) {
      group = 'Traitement';
      const S = LC.seg;
      // scan synthétique : un point par degré sur un cercle de rayon r autour du lidar
      const ring = (r, extra = []) => { const a = [], d = []; for (let k = 0; k < 360; k++) { a.push(k); d.push(r * 1000); } for (const [aa, dd] of extra) { a.push(aa); d.push(dd); } return { a, d }; };
      test('prétraitement : portée, point isolé et point mixte retirés, le reste gardé', () => {
        const sc = { a: [], d: [] };
        for (let k = 0; k < 360; k++) { sc.a.push(k); sc.d.push(k >= 100 && k < 110 ? 1000 : 3000); }  // objet à 1 m devant un mur à 3 m
        sc.a.push(200.4); sc.d.push(1500);                // point isolé (rien autour)
        sc.a.push(0.5); sc.d.push(30);                    // trop proche : le robot lui-même
        const i110 = sc.a.indexOf(110); sc.d[i110] = 2000; // point mixte entre l'objet (1 m) et le mur (3 m)
        const r = LM.preprocessScan(sc, { x: 0, y: 0, th: 0 });
        return expect(r.removed.range === 1 && r.removed.isolated === 1 && r.removed.mixed === 1 && r.kept === 360 - 1, JSON.stringify(r.removed) + ' gardés ' + r.kept);
      });
      test('segmentation en clusters : rupture de distance et trou angulaire', () => {
        const pts = [];
        for (let k = 0; k < 40; k++) pts.push({ x: 2, y: -1 + k * 0.025 });          // mur A
        for (let k = 0; k < 20; k++) pts.push({ x: 1, y: 0.6 + k * 0.025 });         // objet plus proche : rupture de distance
        for (let k = 0; k < 20; k++) pts.push({ x: 1 - k * 0.02, y: 1.5 });          // après un trou angulaire (points à 1,5 m)
        const C = LM.clusterScan(pts, { x: 0, y: 0 }, { angGap: 4 });
        return expect(C.length === 3 && C.map(c => c.length).sort((a, b) => a - b).join() === '20,20,40', C.map(c => c.length).join());
      });
      test('cercle : ajustement exact sur un quart d’arc, et sur des points bruités (±5 mm)', () => {
        const P = [], Q = [], rng = LL.mulberry32(9);
        for (let k = 0; k <= 30; k++) { const t = (200 + k * 3) * Math.PI / 180; P.push({ x: 1 + 0.3 * Math.cos(t), y: 2 + 0.3 * Math.sin(t) }); Q.push({ x: P[k].x + (rng() - 0.5) * 0.01, y: P[k].y + (rng() - 0.5) * 0.01 }); }
        const a = LM.fitCircle(P), b = LM.fitCircle(Q);
        return expect(Math.hypot(a.x - 1, a.y - 2) < 1e-6 && near(a.r, 0.3, 1e-6) && Math.hypot(b.x - 1, b.y - 2) < 0.02 && Math.abs(b.r - 0.3) < 0.02, JSON.stringify([a, b]));
      });
      test('Douglas-Peucker : une polyligne en L garde ses 3 sommets', () => {
        const P = []; for (let k = 0; k <= 20; k++) P.push({ x: k * 0.05, y: 0 }); for (let k = 1; k <= 20; k++) P.push({ x: 1, y: k * 0.05 });
        const v = LM.douglasPeucker(P, 0.02); return expect(v.join() === '0,20,40', v.join());
      });
      test('RANSAC : deux murs perpendiculaires retrouvés malgré 25 % de points aberrants', () => {
        const P = [], rng = LL.mulberry32(4);
        for (let k = 0; k < 60; k++) P.push({ x: k * 0.05, y: (rng() - 0.5) * 0.01 });
        for (let k = 0; k < 60; k++) P.push({ x: 3 + (rng() - 0.5) * 0.01, y: 0.2 + k * 0.05 });
        for (let k = 0; k < 40; k++) P.push({ x: rng() * 3, y: 0.3 + rng() * 2.5 });
        const segs = LM.ransacLines(P, { tol: 0.03, minInliers: 15, rng: LL.mulberry32(1) }).filter(s => LC.segLen(s) > 1.5);
        return expect(segs.length === 2 && Math.abs(LC.angleBetween(segs[0], segs[1]).lines - 90) < 1, segs.map(fmt).join(' '));
      });
      test('features : pied de chaise (petit) et poteau Ø 30 cm (cercle) reconnus, mur en segment', () => {
        const walls = [S(-3, 2, 3, 2), S(0.98, 0.98, 1.02, 0.98), S(1.02, 0.98, 1.02, 1.02), S(1.02, 1.02, 0.98, 1.02), S(0.98, 1.02, 0.98, 0.98)];
        const sc = LL.simulateScan(walls, { x: 0, y: 0, th: 0 }, { tours: 2, noise: 0.003, angNoise: 0, dropout: 0, step: 0.5 }, LL.mulberry32(2));
        // poteau : on remplace par des points exacts sur un cercle centré (−1,5 ; 0,8), r = 0,15 (face visible)
        const pts = LC.polarToGlobal(sc, { x: 0, y: 0, th: 0 });
        const c0 = { x: -1.5, y: 0.8 }, bear = Math.atan2(c0.y, c0.x);
        for (let k = -60; k <= 60; k += 3) { const t = bear + Math.PI + k * Math.PI / 180; pts.push({ x: c0.x + 0.15 * Math.cos(t), y: c0.y + 0.15 * Math.sin(t) }); }
        pts.sort((p, q) => Math.atan2(p.y, p.x) - Math.atan2(q.y, q.x));
        const F = LM.extractFeatures(pts, { x: 0, y: 0 });
        const leg = F.circles.find(c => Math.hypot(c.x - 1, c.y - 1) < 0.05), post = F.circles.find(c => c.kind === 'cercle');
        const wall = F.segments.find(s => LC.segLen(s) > 3);
        return expect(leg && leg.kind === 'petit' && post && Math.hypot(post.x - c0.x, post.y - c0.y) < 0.01 && Math.abs(post.r - 0.15) < 0.01 && wall,
          JSON.stringify({ leg, post, nseg: F.segments.length }));
      });
      test('obstacles circulaires : fusion des observations, puis absence (faisceaux qui traversent) -> mobile puis supprimé', () => {
        const M = LM.createObstacleMap(), robot = { x: 0, y: 0 };
        LM.updateObstacleMap(M, robot, [], [{ x: 1, y: 0, r: 0.05, kind: 'petit' }]);
        LM.updateObstacleMap(M, robot, [], [{ x: 1.02, y: 0, r: 0.05, kind: 'petit' }]);
        const it = M.items[0], ok1 = M.items.length === 1 && it.nObs === 2 && it.conf === 2 && near(it.x, 1.01, 1e-9);
        const through = []; for (let k = -2; k <= 2; k++) through.push({ x: 3, y: k * 0.01 });   // le mur à 3 m est vu à travers
        LM.updateObstacleMap(M, robot, through, []);
        const ok2 = M.items.length === 1 && M.items[0].misses === 1 && M.items[0].conf === 1;
        LM.updateObstacleMap(M, robot, through, []);
        return expect(ok1 && ok2 && M.items.length === 0, JSON.stringify(M.items));
      });
      test('grid map enrichie : impacts, observations et horodatage par cellule', () => {
        const g = LC.createGrid({ xMin: -1, yMin: -1, xMax: 3, yMax: 1, res: 0.1 }), L = LM.createLayers(g);
        LM.updateGridLayered(g, L, { x: 0, y: 0 }, [{ x: 2.05, y: 0.05 }]);
        LM.updateGridLayered(g, L, { x: 0, y: 0 }, [{ x: 2.55, y: 0.05 }]);       // 2e scan : la cellule à 2 m est traversée
        const [i, j] = LC.worldToCell(g, 2.05, 0.05), k = j * g.W + i, [i2, j2] = LC.worldToCell(g, 2.55, 0.05), k2 = j2 * g.W + i2;
        return expect(L.hits[k] === 1 && L.seen[k] === 2 && L.lastOcc[k] === 1 && L.last[k] === 2 && L.hits[k2] === 1 && L.seen[k2] === 1 && L.t === 2, [L.hits[k], L.seen[k], L.lastOcc[k], L.last[k]].join());
      });
      test('obstacle mobile : impact dans 1 scan sur 4 -> trace effacée ; pied vu 3 fois gardé', () => {
        const g = LC.createGrid({ xMin: -1, yMin: -1, xMax: 3, yMax: 1, res: 0.1 }), L = LM.createLayers(g), R = { x: 0, y: 0 };
        LM.updateGridLayered(g, L, R, [{ x: 1.05, y: 0.05 }, { x: 0.05, y: 0.55 }]);
        for (let n = 0; n < 3; n++) LM.updateGridLayered(g, L, R, [{ x: 2.05, y: 0.05 }, { x: 0.05, y: 0.55 }]);
        const [i, j] = LC.worldToCell(g, 1.05, 0.05), [a, b] = LC.worldToCell(g, 0.05, 0.55), r = LM.removeMobile(g, L);
        return expect(r.mobile[j * g.W + i] === 1 && g.data[j * g.W + i] <= LC.FREE_T && r.mobile[b * g.W + a] === 0 && g.data[b * g.W + a] >= LC.OCC_T, JSON.stringify(r.removed));
      });
      test('taches isolées : bruit vu une fois retiré, petit obstacle vu 3 fois gardé', () => {
        const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 1, yMax: 1, res: 0.1 }), L = LM.createLayers(g);
        g.data[2 * g.W + 2] = 200; L.hits[2 * g.W + 2] = 1; g.data[7 * g.W + 7] = 230; L.hits[7 * g.W + 7] = 3;
        const r = LM.removeSpeckles(g, L);
        return expect(g.data[2 * g.W + 2] === LC.UNKNOWN && g.data[7 * g.W + 7] === 230 && r.keptInterest === 1, JSON.stringify(r));
      });
      test('lissage majoritaire : trou inconnu au milieu du libre comblé, mur d’une cellule préservé', () => {
        const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 1, yMax: 1, res: 0.1, init: 0 });
        for (let i = 0; i < 10; i++) g.data[5 * g.W + i] = 255;        // mur horizontal d'une cellule
        g.data[2 * g.W + 2] = LC.UNKNOWN;
        LM.smoothGrid(g); let wall = true; for (let i = 0; i < 10; i++) wall = wall && g.data[5 * g.W + i] === 255;
        return expect(g.data[2 * g.W + 2] <= LC.FREE_T && wall, g.data[2 * g.W + 2]);
      });
      test('fermeture : trou d’une cellule dans un mur bouché, passage franchement libre respecté', () => {
        const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 2, yMax: 1, res: 0.1, init: LC.UNKNOWN });
        for (let i = 0; i < 20; i++) g.data[5 * g.W + i] = 255;
        g.data[5 * g.W + 6] = 120;                                     // trou d'une cellule, état incertain
        g.data[5 * g.W + 14] = 10;                                     // cellule traversée plusieurs fois (porte ouverte)
        const r = LM.closeGaps(g);
        return expect(g.data[5 * g.W + 6] >= LC.OCC_T && g.data[5 * g.W + 14] === 10, JSON.stringify(r));
      });
      test('obstacles de la grille : composantes et classes (petit, objet, structure)', () => {
        const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 4, yMax: 2, res: 0.05, init: 0 });
        g.data[5 * g.W + 5] = g.data[5 * g.W + 6] = 255;                          // 10 cm : petit
        for (let i = 20; i < 30; i++) g.data[10 * g.W + i] = 255;                 // 50 cm : objet
        for (let i = 35; i < 75; i++) g.data[30 * g.W + i] = 255;                 // 2 m : structure
        const { obstacles: O } = LM.gridObstacles(g, null), cls = O.map(o => o.cls).sort().join();
        const big = O.find(o => o.cls === 'structure');
        return expect(O.length === 3 && cls === 'objet,petit,structure' && Math.abs(big.len - 2) < 0.06 && Math.abs(big.angle) < 1e-6, cls + ' ' + (big && big.len));
      });
      test('régularisation : murs redressés exactement parallèles / perpendiculaires, coins raccordés exactement', () => {
        const rot = (s, d) => { const mx = (s.x1 + s.x2) / 2, my = (s.y1 + s.y2) / 2, c = Math.cos(d * Math.PI / 180), n = Math.sin(d * Math.PI / 180);
          const r = (x, y) => [mx + c * (x - mx) - n * (y - my), my + n * (x - mx) + c * (y - my)]; const [a, b] = r(s.x1, s.y1), [e, f] = r(s.x2, s.y2); return S(a, b, e, f); };
        const room = [rot(S(0.1, 0, 3.9, 0), 2), rot(S(4, 0.1, 4, 2.9), -3), rot(S(3.9, 3, 0.1, 3), 1.5), rot(S(0, 2.9, 0, 0.1), -2)];
        const map = { features: room.map((s, k) => Object.assign(s, { id: k + 1, conf: 3, nObs: 3, firstSeen: 1, lastSeen: 1 })), corners: [], scans: 1, nextId: 5 };
        const { map: M, report } = LM.regularizeVectorMap(map);
        // les axes sont la direction dominante mesurée (pas forcément 0°) : tous les murs y sont exactement parallèles ou perpendiculaires
        const angOk = M.features.every(f => { const a = (((LC.segAngle(f) * 180 / Math.PI - report.theta0) % 90) + 90) % 90; return Math.min(a, 90 - a) < 1e-6; });
        const ends = M.features.flatMap(f => [[f.x1, f.y1], [f.x2, f.y2]]);
        const joined = ends.every(([x, y]) => ends.filter(([u, v]) => Math.hypot(u - x, v - y) < 1e-9).length === 2);
        return expect(M.features.length === 4 && angOk && joined && M.corners.length === 4, JSON.stringify(report));
      });
      test('alignement : deux morceaux d’un même mur décalés de 3 cm ramenés sur une droite', () => {
        const r = LM.alignCollinear([Object.assign(S(0, 0, 2, 0), { conf: 1 }), Object.assign(S(2.5, 0.03, 4, 0.03), { conf: 1 })]);
        const [a, b] = r.segments; return expect(near(a.y1, b.y1, 1e-12) && a.y1 > 0 && a.y1 < 0.03, JSON.stringify(r.segments.map(fmt)));
      });
      test('collision : dépassement de 5 cm d’un mur à travers un autre rogné', () => {
        const r = LM.trimCollisions([S(0, -0.05, 0, 2), S(-1, 0, 1, 0)]);
        return expect(r.trimmed === 1 && sameSeg(r.segments[0], 0, 0, 0, 2, 1e-12), fmt(r.segments[0]));
      });
      test('carte de coût et trajet A* : contourne un mur, garde ses distances, refuse une arrivée dans l’obstacle', () => {
        const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 4, yMax: 3, res: 0.05, init: 0 });
        for (let j = 0; j < 45; j++) g.data[j * g.W + 40] = 255;                  // mur x = 2 m, de y = 0 à 2,25 m
        const cm = LM.costmap(g, { robotRadius: 0.15, inflation: 0.3 });
        const k = (x, y) => { const [i, j] = LC.worldToCell(g, x, y); return cm.cost[j * g.W + i]; };
        const costOk = k(2.02, 1) === 254 && k(2.14, 1) === 253 && k(3.5, 1) === 0 && k(2.3, 1) > 0 && k(2.3, 1) < 253;
        const r = LM.planPath(cm, { x: 1, y: 0.5 }, { x: 3, y: 0.5 });
        let clear = r.ok; if (r.ok) for (const c of r.raw) clear = clear && cm.cost[c.j * g.W + c.i] < 253;
        const bad = LM.planPath(cm, { x: 1, y: 0.5 }, { x: 2.02, y: 1 });
        return expect(costOk && clear && r.length > 3.5 && Math.max(...r.path.map(p => p.y)) > 2.25 + 0.15 && !bad.ok, JSON.stringify({ costOk, len: r.length, bad: bad.reason }));
      });
      test('gomme : segment coupé en deux au bord du disque, cellules et cercles effacés, segment entièrement dedans supprimé', () => {
        const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 4, yMax: 2, res: 0.05, init: 0 });
        for (let i = 0; i < 80; i++) g.data[20 * g.W + i] = 255;                    // mur y ≈ 1 m
        const vmap = { features: [Object.assign(S(0, 1, 4, 1), { id: 1, conf: 5 }), Object.assign(S(1.9, 0.5, 2.1, 0.5), { id: 2, conf: 1 })], corners: [], nextId: 3 };
        const omap = { items: [{ id: 1, x: 2, y: 1.3, r: 0.02 }, { id: 2, x: 3.5, y: 0.3, r: 0.02 }] };
        const R = { grid: g, vmap, omap }, st = LM.eraseDisc(R, 2, 0.9, 0.5);
        const [a, b] = vmap.features, cut = vmap.features.length === 2 && near(a.x2, 2 - Math.sqrt(0.24), 1e-9) && near(b.x1, 2 + Math.sqrt(0.24), 1e-9) && a.conf === 5 && b.id === 3;
        const cellsOk = g.data[20 * g.W + 40] === LC.FREE_T - 25 && g.data[20 * g.W + 5] === 255;
        return expect(cut && cellsOk && st.segCut === 1 && st.segRemoved === 1 && st.circles === 1 && omap.items[0].id === 2, JSON.stringify(st) + ' ' + vmap.features.map(fmt).join(' '));
      });
      test('filtres de taille : segments courts, petits cercles et petites taches supprimés ; le reste intact', () => {
        const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 4, yMax: 2, res: 0.05, init: 0 });
        g.data[5 * g.W + 5] = 255;                                                    // tache de 5 cm
        for (let i = 20; i < 60; i++) g.data[30 * g.W + i] = 255;                     // mur de 2 m
        const R = { grid: g, vmap: { features: [S(0, 0, 0.2, 0), S(0, 1, 2, 1)], corners: [] }, omap: { items: [{ x: 1, y: 1, r: 0.01 }, { x: 2, y: 1, r: 0.1 }] } };
        const st = LM.filterSmall(R, { minSegLen: 0.5, minCircleD: 0.05, minBlob: 0.1 });
        return expect(st.segments === 1 && st.circles === 1 && st.blobs === 1 && g.data[5 * g.W + 5] < LC.FREE_T && g.data[30 * g.W + 30] === 255 && R.vmap.features.length === 1, JSON.stringify(st));
      });
      test('filtre de distance au robot : ce qui touche une position de scan est effacé, pas le reste', () => {
        const g = LC.createGrid({ xMin: 0, yMin: 0, xMax: 4, yMax: 2, res: 0.05, init: 0 });
        g.data[20 * g.W + 22] = 255; g.data[20 * g.W + 60] = 255;                      // à 8 cm du robot (1 ; 1), et loin
        const R = { grid: g, vmap: { features: [], corners: [], nextId: 1 }, omap: { items: [{ x: 1.1, y: 1, r: 0.02 }] } };
        const st = LM.filterNearPoses(R, [{ x: 1, y: 1, th: 0 }], 0.2);
        return expect(st.cells === 1 && st.circles === 1 && g.data[20 * g.W + 60] === 255, JSON.stringify(st));
      });
      test('qualité : scans simulés à poses exactes -> plus de 90 % des points sur un obstacle', () => {
        const sc = LL.classroomScene(3, { legs: 0 }), rng = LL.mulberry32(8), poses = [{ x: 2, y: 2, th: 0 }, { x: 6, y: 5, th: 90 }, { x: 11, y: 6, th: 200 }];
        const data = poses.map(p => ({ robot: p, pts: LM.preprocessScan(LL.simulateScan(sc.walls, p, { tours: 2 }, rng), p).pts }));
        const { grid, layers } = LM.buildLayeredGrid(data), q = LM.mapQuality(grid, data, layers);
        return expect(q.pointsOnObstacle > 0.9 && q.meanDist < 0.03, JSON.stringify(q));
      });
    }

    const passed = results.filter(r => r.ok).length;
    return { results, passed, total: results.length };
  }
  return { runTests };
});
