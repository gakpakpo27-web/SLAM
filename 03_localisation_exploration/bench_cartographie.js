#!/usr/bin/env node
/* ============================================================================
 *  bench_cartographie.js – Cartographie automatique (SLAM simple par images clés)
 *    node bench_cartographie.js [--out resultats_cartographie]
 *  Salle simulée vue AU SOL (pieds de 14 tables, 60 chaises, sacs, personnes). Le robot suit un trajet
 *  (pas de 15 cm, rotations de 15°, arrêt de 3 scans tous les 3 pas), odométrie bruitée de 10 %.
 *  Mesures : erreur des positions enregistrées (repère du premier scan), puis localisation globale de
 *  15 scans du sol sur (a) une carte faite robot soulevé (plateaux de tables) et (b) la carte automatique.
 * ========================================================================== */
const LL = require('./lidar_loc.js'), LC = require('./lidar_core.js'), fs = require('fs');
const arg = (k, d) => { const i = process.argv.indexOf('--' + k); return i > 0 ? process.argv[i + 1] : d; };
const out = arg('out', 'resultats_cartographie');
function scenes(seed) {
  const base = LL.classroomScene(seed, { legs: 0 }), S = LC.seg, rng = LL.mulberry32(seed + 50);
  const high = base.walls.slice(), floor = base.walls.slice();
  const box = (L, x, y, w, h = w) => L.push(S(x, y, x + w, y), S(x + w, y, x + w, y + h), S(x + w, y + h, x, y + h), S(x, y + h, x, y));
  const free = [];                                    // zones occupées (pour ne pas poser le robot dessus)
  // 14 tables 1,4 × 0,7 m : plateau vu par le lidar soulevé, seulement 4 pieds au sol
  for (let n = 0; n < 40 && free.length < 14; n++) {
    const x = 1 + rng() * 10.5, y = 0.8 + rng() * 6;
    if (!base.free(x + 0.7, y + 0.35, 1.0) || free.some(([a, b]) => Math.abs(a - x) < 1.8 && Math.abs(b - y) < 1.2)) continue;
    free.push([x, y]); box(high, x, y, 1.4, 0.7);
    for (const [a, b] of [[0, 0], [1.36, 0], [0, 0.66], [1.36, 0.66]]) box(floor, x + a, y + b, 0.04);
  }
  // au sol seulement : 60 chaises (4 pieds), 12 sacs / poubelles, 6 personnes (2 jambes)
  for (let n = 0; n < 60; n++) { const x = 1 + rng() * 11.5, y = 0.8 + rng() * 6.5; if (!base.free(x, y, 0.5)) continue; for (const [a, b] of [[0, 0], [0.4, 0], [0, 0.4], [0.4, 0.4]]) box(floor, x + a, y + b, 0.03); }
  for (let n = 0; n < 12; n++) { const x = 1 + rng() * 11.5, y = 0.8 + rng() * 6.5; if (base.free(x, y, 0.5)) box(floor, x, y, 0.3); }
  for (let n = 0; n < 6; n++) { const x = 1 + rng() * 11.5, y = 0.8 + rng() * 6.5; box(floor, x, y, 0.12); box(floor, x + 0.25, y, 0.12); }
  const ok = (x, y) => base.free(x, y, 0.5) && !free.some(([a, b]) => x > a - 0.4 && x < a + 1.8 && y > b - 0.4 && y < b + 1.1);
  return { high: { walls: high, free: ok }, floor };
}

const md = ['# Cartographie automatique (simulation, robot au sol)', '',
  '| Graine | Scans reçus | Positions enregistrées | Temps | Perdu | Erreur des positions (médiane / max) | Localisation au sol : carte faite soulevé | … carte automatique au sol |',
  '|---|---|---|---|---|---|---|---|'];
for (const seed of [3, 4, 5]) {
  const { high, floor } = scenes(seed), D = LL.demoScenario(seed), rng = LL.mulberry32(seed + 3);
  const gauss = () => Math.sqrt(-2 * Math.log(rng() + 1e-12)) * Math.cos(2 * Math.PI * rng());
  const seq = []; D.path.forEach((p, k) => { seq.push(p); if (k % 3 === 0) seq.push(p, p); });
  const Mp = LL.createMapper(), t0 = Date.now(); let prev = null, lostN = 0;
  for (const truth of seq) {
    let motion = null; if (prev) { const m = LL.relPose(prev, truth); motion = { x: m.x * (1 + 0.1 * gauss()) + 0.01 * (m.x !== 0), y: m.y * (1 + 0.1 * gauss()), th: m.th * (1 + 0.1 * gauss()) }; }
    const r = LL.mapperUpdate(Mp, Object.assign(LL.simulateScan(floor, truth, { tours: 1 }, rng), { meta: truth }), motion);
    if (r.status === 'perdu') lostN++; prev = truth;
  }
  if (Mp.pending) LL.mapperCommit(Mp);
  const E = Mp.kfs.map(kf => LL.poseError(kf.pose, LL.relPose(seq[0], kf.meta)));
  const med = a => a.slice().sort((x, y) => x - y)[Math.floor(a.length / 2)], mx = a => Math.max(...a);
  const highMap = LL.prepareMap(LL.buildReferences([[2, 1.5, 0], [6.5, 1.5, 90], [11, 1.2, 150], [2, 5.5, 0], [6.5, 5, -90], [12, 6, 200], [2.5, 8.5, 0], [7.5, 6.8, 30], [12.5, 2, 90]]
    .map(([x, y, th]) => Object.assign(LL.simulateScan(high.walls, { x, y, th }, { tours: 3 }, rng), { pose: { x, y, th } })), -1).grid);
  const autoMap = LL.prepareMap(Mp.grid);
  let okH = 0, okA = 0, n = 0;
  for (let k = 0; k < 15; ) {
    const x = 1 + rng() * 11.5, y = 1 + rng() * 6.5; if (!high.free(x, y, 0.5)) continue; k++; n++;
    const truth = { x, y, th: rng() * 360 - 180 }, pts = LL.uniformSubsample(LL.localPoints(LL.simulateScan(floor, truth, { tours: 1 }, rng)), { max: 350 });
    const eH = LL.poseError(LL.globalLocalize(highMap, pts).pose, truth); okH += eH.pos <= 0.05 && eH.ang <= 2;
    const eA = LL.poseError(LL.globalLocalize(autoMap, pts).pose, LL.relPose(seq[0], truth)); okA += eA.pos <= 0.05 && eA.ang <= 2;
  }
  const line = `| ${seed} | ${seq.length} | ${Mp.kfs.length} | ${((Date.now() - t0) / 1000).toFixed(1)} s | ${lostN} | ${(med(E.map(e => e.pos)) * 100).toFixed(1)} cm / ${(mx(E.map(e => e.pos)) * 100).toFixed(1)} cm | ${okH}/${n} | ${okA}/${n} |`;
  md.push(line); console.log(line);
}
fs.writeFileSync(out + '.md', md.join('\n') + '\n');
