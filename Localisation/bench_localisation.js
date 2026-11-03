#!/usr/bin/env node
/* ============================================================================
 *  Banc d'évaluation de la localisation (Jour 2)
 *
 *  node bench_localisation.js                       -> simulation (poses exactes)
 *  node bench_localisation.js --real session.json   -> + scans réels (poses de référence = poses de la session)
 *  Options : --draws 5  --seed 7  --out resultats_localisation
 *
 *  Produit <out>.md (tableaux) et <out>.csv (tous les essais).
 *  Pour les scans réels, la session doit contenir des scans superposés (export « Session complète (JSON) »).
 * ========================================================================== */
const fs = require('fs');
const LL = require('./lidar_loc.js');

const args = process.argv.slice(2), arg = (k, d) => { const i = args.indexOf(k); return i >= 0 ? args[i + 1] : d; };
const draws = +arg('--draws', 5), seed = +arg('--seed', 7), out = arg('--out', 'resultats_localisation'), realPath = arg('--real', null);

(async () => {
  let md = `# Résultats du banc de localisation\n\nGénéré le ${new Date().toLocaleString('fr-FR')} · graine ${seed} · ${draws} tirages par niveau.\n\n`;
  let csv = '';
  let lastPct = -1;
  const progress = (f, t) => { const p = Math.floor(f * 20) * 5; if (p !== lastPct) { lastPct = p; console.log(`  ${p} % ${t}`); } };

  console.log('Simulation…');
  const sim = await LL.runLocBench({ scenario: 'sim', seed, draws }, progress);
  console.log('');
  md += LL.benchToMarkdown(sim, 'Simulation (salle en L 13,5 × 9,8 m, 70 pieds de chaises, poses exactes)');
  csv += LL.benchToCSV(sim);

  if (realPath) {
    const sess = JSON.parse(fs.readFileSync(realPath, 'utf8'));
    const scans = sess.scans.filter(s => s.visible !== false).map(s => ({ name: s.name, a: s.a, d: s.d, i: s.i, pose: s.pose }));
    console.log(`Réel : ${scans.length} scans…`);
    const unc = referenceUncertainty(scans, seed);
    const real = await LL.runLocBench({ scenario: 'real', seed, draws, real: { scans } }, progress);
    console.log('');
    md += `## Incertitude des références réelles\n\n${unc.text}\n\n`;
    md += LL.benchToMarkdown(real, `Scans réels (${scans.length} scans, carte construite sans le scan testé)`);
    csv += LL.benchToCSV(real).split('\n').slice(1).join('\n');
  }
  fs.writeFileSync(out + '.md', md); fs.writeFileSync(out + '.csv', csv);
  console.log(`\nÉcrit : ${out}.md, ${out}.csv`);
})();

// Incertitude des poses de référence réelles (issues de la superposition) : chaque scan est recalé par ICP
// point-à-segment sur la carte des AUTRES scans, depuis sa pose de référence perturbée de 3 cm / 1,5°
// (10 essais). L'écart du résultat à la référence mesure la cohérence (et donc l'incertitude) des références.
function referenceUncertainty(scans, seed) {
  const rng = LL.mulberry32(seed + 99), dev = [], devA = [];
  for (let k = 0; k < scans.length; k++) {
    const R = LL.buildReferences(scans, k), pts = LL.uniformSubsample(LL.localPoints(scans[k]), { max: 350 });
    for (let t = 0; t < 10; t++) {
      const r = LL.vecICP(R.index, pts, LL.perturb(scans[k].pose, 0.03, 1.5, rng)), e = LL.poseError(r, scans[k].pose);
      dev.push(e.pos); devA.push(e.ang);
    }
  }
  const q = (a, p) => a.slice().sort((x, y) => x - y)[Math.floor(p * (a.length - 1))];
  return { pos: q(dev, 0.5), ang: q(devA, 0.5),
    text: `Chaque scan réel a été recalé par ICP point-à-segment sur la carte des autres scans, depuis sa pose de référence perturbée de 3 cm et 1,5° (10 essais par scan). ` +
          `Écart à la référence : médiane ${(q(dev, 0.5) * 100).toFixed(1)} cm et ${q(devA, 0.5).toFixed(2)}°, 90e centile ${(q(dev, 0.9) * 100).toFixed(1)} cm et ${q(devA, 0.9).toFixed(2)}°. ` +
          `Les références réelles sont donc connues à environ ±${Math.ceil(q(dev, 0.9) * 100)} cm et ±${q(devA, 0.9).toFixed(1)}° ; une erreur mesurée plus petite que cela n'est pas significative.` };
}
