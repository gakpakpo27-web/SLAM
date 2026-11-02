/* ============================================================================
 *  Tests de lidar_core.js – lancer :  node test_lidar_core.js
 *  (les mêmes tests sont exécutables dans lidar_viewer.html, onglet Segments)
 * ========================================================================== */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) {
    module.exports = factory();
    if (require.main === module) {
      const res = module.exports.runTests(require('./lidar_core.js'));
      for (const r of res.results) console.log(`${r.ok ? '  OK ' : 'ÉCHEC'}  [${r.group}] ${r.name}${r.ok ? '' : '  -> ' + r.msg}`);
      console.log(`\n${res.passed}/${res.total} tests réussis`);
      process.exit(res.passed === res.total ? 0 : 1);
    }
  } else root.LidarTests = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  function runTests(LC) {
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
    function simScan(robot, noise) {
      const walls = [seg(0, 0, 6, 0), seg(6, 0, 6, 4), seg(6, 4, 0, 4), seg(0, 4, 0, 0),
                     seg(4, 2, 4.2, 2), seg(4.2, 2, 4.2, 2.2), seg(4.2, 2.2, 4, 2.2), seg(4, 2.2, 4, 2)];
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

    const passed = results.filter(r => r.ok).length;
    return { results, passed, total: results.length };
  }
  return { runTests };
});
