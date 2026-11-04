// Banc de l'exploration autonome (lidar_explore.js) en simulation : node bench_exploration.js
// Salle de classe simulée (13,5 × 9,8 m, meuble central, armoire, pieds de chaises), départ au hasard,
// vitesses réelles du robot différentes du réglage (±20 %), odométrie ±10 %. Écrit resultats_exploration.md.
const fs = require('fs'), LL = require('./lidar_loc.js'), LX = require('./lidar_explore.js');
const med = a => a.slice().sort((x, y) => x - y)[a.length >> 1];
const rows = [];
for (let seed = 20; seed < 32; seed++) {
  const rng = LL.mulberry32(seed), sc = LL.classroomScene(seed, { legs: 60 });
  let start; do { start = { x: 0.5 + rng() * 12.5, y: 0.5 + rng() * 9, th: Math.round(rng() * 360 - 180) }; } while (!sc.free(start.x, start.y, 0.3));
  const bias = 0.8 + rng() * 0.4, t0 = Date.now();
  const R = LX.simulateExploration({ seed, legs: 60, start, bias });
  const e = R.poseErr.map(x => x.pos);
  rows.push({ seed, start, bias, reason: R.X.reason, cov: LX.coverage(R.Mp.grid, R.scene), col: R.collisions, cmds: R.X.nCmd, dist: R.X.stats.dist, kfs: R.Mp.kfs.length,
              med: med(e), max: Math.max(...e), home: Math.hypot(R.truth.x - start.x, R.truth.y - start.y), blocked: R.X.stats.blocked, ms: Date.now() - t0 });
  console.log(seed, R.X.reason, (rows[rows.length - 1].cov * 100).toFixed(0) + ' %');
}
const checks = [['lidar tourné de 90°', { lidarOffset: 90 }], ['lidar tourné de 180°', { lidarOffset: 180 }], ['scan en miroir', { mirror: true }]]
  .map(([name, o]) => { const R = LX.simulateExploration({ seed: 7, start: { x: 1.5, y: 1.5, th: 0 }, ...o }); return { name, cmds: R.X.nCmd, col: R.collisions, reason: R.X.reason }; });
const pc = v => (v * 100).toFixed(0) + ' %', cm = v => (v * 100).toFixed(1) + ' cm';
let md = '# Exploration autonome – résultats en simulation\n\n';
md += 'Salle simulée de 13,5 × 9,8 m en L, avec meuble central, armoire et 60 pieds de chaises de 4 cm. Le départ est tiré au hasard. ';
md += 'Les vitesses réelles du robot diffèrent du réglage de 0,8 à 1,2 fois, et l’odométrie a ±10 % d’erreur plus une petite dérive. ';
md += 'Le robot ne connaît pas la salle : il ne voit que ses scans. Commande : `node bench_exploration.js`.\n\n';
md += '| Graine | Départ (x ; y ; cap) | Vitesse réelle | Fin | Couverture | Chocs | Ordres | Distance | Positions | Erreur de pose médiane / max | Arrivée à |\n|---|---|---|---|---|---|---|---|---|---|---|\n';
for (const r of rows) md += `| ${r.seed} | ${r.start.x.toFixed(1)} ; ${r.start.y.toFixed(1)} ; ${r.start.th}° | × ${r.bias.toFixed(2)} | ${r.reason} | ${pc(r.cov)} | ${r.col} | ${r.cmds} | ${r.dist.toFixed(0)} m | ${r.kfs} | ${cm(r.med)} / ${cm(r.max)} | ${cm(r.home)} du départ |\n`;
md += `\n**Bilan.** Cartographie terminée dans ${rows.filter(r => /terminée/.test(r.reason)).length}/${rows.length} cas, avec retour au départ dans ${rows.filter(r => /revenu/.test(r.reason)).length}/${rows.length} cas. `;
md += `Couverture médiane ${pc(med(rows.map(r => r.cov)))} (minimum ${pc(Math.min(...rows.map(r => r.cov)))}). Chocs : ${rows.reduce((a, r) => a + r.col, 0)}. `;
md += `Erreur de pose médiane ${cm(med(rows.map(r => r.med)))}, maximum ${cm(Math.max(...rows.map(r => r.max)))}. `;
md += `${med(rows.map(r => r.cmds))} ordres et ${med(rows.map(r => r.dist)).toFixed(0)} m parcourus en médiane. `;
md += 'La couverture est la part de la surface libre réelle que la carte marque libre.\n\n';
md += '## Contrôle du montage du lidar (premiers mouvements)\n\n| Cas | Ordres avant l’arrêt | Chocs | Message |\n|---|---|---|---|\n';
for (const c of checks) md += `| ${c.name} | ${c.cmds} | ${c.col} | ${c.reason} |\n`;
fs.writeFileSync('resultats_exploration.md', md); console.log(md);
