// Banc : superposition des scans quand le robot TOURNE plus que prévu et après une COUPURE de liaison.
//   node bench_rotation_coupure.js   -> resultats_rotation_coupure.md
// Cartographe de lidar_loc.js (createMapper / mapperUpdate), salle simulée avec 40 pieds de chaises.
const fs = require('fs'), LL = require('./lidar_loc.js');
const evalKfs = kfs => { const t0 = kfs[0].truth; let bad = 0, worst = 0; for (const k of kfs) { const e = LL.poseError(k.est, LL.relPose(t0, k.truth)); worst = Math.max(worst, e.pos); if (e.pos > 0.1 || e.ang > 3) bad++; } return { n: kfs.length, bad, worst }; };
function rotationRun(opt) {
  let n = 0, bad = 0, worst = 0, lost = 0;
  for (const seed of [1, 2, 3, 4, 5, 6]) {
    const sc = LL.classroomScene(seed, { legs: 40 }), rng = LL.mulberry32(seed + 3), Mp = LL.createMapper({ tours: 2, ...opt });
    let p = { x: 2, y: 1.5, th: 0 }; const kfs = [];
    const feed = () => { const r = LL.mapperUpdate(Mp, LL.simulateScan(sc.walls, p, { tours: 1 }, rng), null); if (r.keyframe) kfs.push({ est: r.keyframe.pose, truth: { ...p } }); if (r.status === 'perdu') lost++; };
    feed(); feed();
    for (let k = 0; k < 60; k++) {
      if (k % 5 === 4) p = { ...p, th: p.th + (rng() < 0.5 ? -1 : 1) * (30 + rng() * 90) };      // tourné à la main de 30 à 120°
      else { const q = LL.compose(p, { x: 0.15, y: 0, th: 0 }); p = sc.free(q.x, q.y, 0.3) ? q : { ...p, th: p.th + 90 }; }
      feed(); feed();
    }
    const e = evalKfs(kfs); n += e.n; bad += e.bad; worst = Math.max(worst, e.worst);
  }
  return { n, bad, worst, lost };
}
function gapRun(signal) {
  let n = 0, bad = 0, worst = 0;
  for (const seed of [1, 2, 3, 4, 5, 6, 7, 8]) {
    const sc = LL.classroomScene(seed, { legs: 40 }), rng = LL.mulberry32(seed + 9), Mp = LL.createMapper({ tours: 2 });
    let p = { x: 2, y: 1.5, th: 0 }; const kfs = [];
    const feed = ev => { const r = LL.mapperUpdate(Mp, LL.simulateScan(sc.walls, p, { tours: 1 }, rng), null, ev); if (r.keyframe) kfs.push({ est: r.keyframe.pose, truth: { ...p } }); };
    feed(); feed();
    for (let k = 0; k < 40; k++) {
      let gap = false;
      if (k % 8 === 7) { for (let t = 0; t < 10; t++) { const q = LL.compose(p, { x: 0.08, y: 0, th: (rng() - 0.3) * 20 }); p = sc.free(q.x, q.y, 0.3) ? q : { ...p, th: p.th + 40 }; } gap = true; }
      else { const q = LL.compose(p, { x: 0.12, y: 0, th: 0 }); p = sc.free(q.x, q.y, 0.3) ? q : { ...p, th: p.th + 35 }; }
      feed(signal && gap ? { gap: true } : {}); feed(); feed();
    }
    const e = evalKfs(kfs); n += e.n; bad += e.bad; worst = Math.max(worst, e.worst);
  }
  return { n, bad, worst };
}
const A = rotationRun({ rotSearch: false, kfInl: 0 }), B = rotationRun({});
const G0 = gapRun(false), G1 = gapRun(true);
const cm = v => (v * 100).toFixed(0) + ' cm';
let md = '# Superposition : rotations du robot et coupures de liaison\n\nCommande : `node bench_rotation_coupure.js`. Une position est « mal superposée » si elle est à plus de 10 cm ou 3° de sa vraie place, dans le repère du premier scan.\n\n';
md += '## Robot tourné à la main (30 à 120° entre deux scans, sans commande)\n\n6 parcours de 60 pas.\n\n| Cartographe | Positions enregistrées | Mal superposées | Pire écart | Scans « perdu » |\n|---|---|---|---|---|\n';
md += `| Avant (recherche à ±25° de la prédiction, tout scan posé) | ${A.n} | ${A.bad} | ${cm(A.worst)} | ${A.lost} |\n| Maintenant (toutes orientations, pas de scan douteux, pas de doublon) | ${B.n} | ${B.bad} | ${cm(B.worst)} | ${B.lost} |\n\n`;
md += '## Coupure de liaison (2 s sans scan, robot déplacé de 0,3 à 0,8 m et tourné de 20 à 120°)\n\n8 parcours, 5 coupures chacun.\n\n| | Positions enregistrées | Mal superposées | Pire écart |\n|---|---|---|---|\n';
md += `| Coupure ignorée | ${G0.n} | ${G0.bad} | ${cm(G0.worst)} |\n| Coupure détectée : on attend que le robot se retrouve | ${G1.n} | ${G1.bad} | ${cm(G1.worst)} |\n`;
fs.writeFileSync('resultats_rotation_coupure.md', md); console.log(md);
