#!/usr/bin/env node
/* ============================================================================
 *  bench_suivi.js – Évaluation du robot en direct (localisation globale + suivi)
 *    node bench_suivi.js [--real session.json] [--seed 3] [--out resultats_suivi]
 *  1. Localisation globale (aucune pose de départ) sur les scans réels : chaque scan est cherché
 *     sur la carte construite avec les AUTRES scans (validation croisée).
 *  2. Suivi en simulation : le robot parcourt la salle simulée (pas de 15 cm, rotations de 15°) ;
 *     odométrie bruitée (10 % + biais), poses exactes connues ; robot « kidnappé » à mi-parcours.
 * ========================================================================== */
const LL = require('./lidar_loc.js'), LC = require('./lidar_core.js'), fs = require('fs');
const arg = (k, d) => { const i = process.argv.indexOf('--' + k); return i > 0 ? process.argv[i + 1] : d; };
const out = arg('out', 'resultats_suivi'), seed = +arg('seed', 3), realPath = arg('real', null);
const md = ['# Robot en direct : localisation globale et suivi', ''];
const pct = v => (v * 100).toFixed(0) + ' %', med = a => { const b = a.slice().sort((x, y) => x - y); return b.length ? b[Math.floor(b.length / 2)] : NaN; };

// ---------- 1. localisation globale sur scans réels
if (realPath) {
  const S = JSON.parse(fs.readFileSync(realPath, 'utf8')).scans.map(s => ({ name: s.name, a: s.a, d: s.d, i: s.i, pose: s.pose }));
  md.push('## 1. Localisation globale sur les scans réels (sans pose de départ)', '',
    `Carte de chaque essai = les ${S.length - 1} autres scans. Réussite : ≤ 5 cm et ≤ 2°.`, '',
    '| Scan | Erreur position | Erreur angle | Points sur la carte | Confiance | 2e hypothèse / 1re | Bonne pose parmi les hypothèses | Temps |', '|---|---|---|---|---|---|---|---|');
  const R = [];
  for (let k = 0; k < S.length; k++) {
    const ref = LL.buildReferences(S, k), M = LL.prepareMap(ref.grid, ref.vmap.features);
    const pts = LL.uniformSubsample(LL.localPoints(S[k]), { max: 350 });
    const g = LL.globalLocalize(M, pts), e = LL.poseError(g.pose, S[k].pose);
    const inHyp = g.hyps.findIndex(h => { const f = LL.poseError(h, S[k].pose); return f.pos <= 0.05 && f.ang <= 2; });
    const ok = e.pos <= 0.05 && e.ang <= 2;
    R.push({ ok, conf: g.confidence, ms: g.ms, inHyp });
    md.push(`| ${S[k].name} | ${(e.pos * 100).toFixed(1)} cm | ${e.ang.toFixed(2)}° | ${pct(g.inliers)} | ${g.confidence} | ${g.ratio.toFixed(2)} | ${inHyp >= 0 ? 'oui (n° ' + (inHyp + 1) + ')' : 'non'} | ${g.ms.toFixed(0)} ms |`);
    process.stdout.write(`  ${S[k].name} ${ok ? 'ok' : 'échec'} ${g.confidence}\n`);
  }
  const sure = R.filter(r => r.conf === 'sûr');
  md.push('', `**Réussite : ${R.filter(r => r.ok).length}/${R.length}** · classés « sûr » : ${sure.length}, dont justes ${sure.filter(r => r.ok).length} · ` +
    `bonne pose parmi les hypothèses : ${R.filter(r => r.inHyp >= 0).length}/${R.length} · temps médian ${med(R.map(r => r.ms)).toFixed(0)} ms`, '');
}

// ---------- 2. suivi en simulation
const D = LL.demoScenario(seed), M = LL.prepareMap(D.refs.grid, D.refs.vmap.features), rng = LL.mulberry32(seed + 99);
const gauss = () => Math.sqrt(-2 * Math.log(rng() + 1e-12)) * Math.cos(2 * Math.PI * rng());
// objets absents de la carte (chaises déplacées, cartons, personnes) : 25 boîtes de 30 à 60 cm posées dans l'espace libre
const clutter = (() => {
  const r = LL.mulberry32(seed + 5), W = []; let n = 0;
  while (n < 25) {
    const x = 1 + r() * 11.5, y = 1 + r() * 6.5, w = 0.3 + r() * 0.3;
    if (!D.scene.free(x, y, 0.6) || D.path.some(p => Math.hypot(p.x - x, p.y - y) < 0.6)) continue;
    W.push(LC.seg(x, y, x + w, y), LC.seg(x + w, y, x + w, y + w), LC.seg(x + w, y + w, x, y + w), LC.seg(x, y + w, x, y)); n++;
  }
  return W;
})();
function run(label, opt) {
  const T = LL.createTracker(M), rows = []; let truthPrev = null;
  const kid = opt.kidnap ? Math.floor(D.path.length / 2) : -1;
  for (let k = 0; k < D.path.length; k++) {
    let truth = D.path[k];
    if (k >= kid && kid >= 0) truth = Object.assign({}, D.path[k], { x: D.path[k].x, y: D.path[k].y });
    let motion = null;
    if (truthPrev && opt.odom) {
      const m = LL.relPose(truthPrev, truth);                          // déplacement commandé = vrai déplacement bruité
      motion = { x: m.x * (1 + 0.1 * gauss()) + 0.01, y: m.y * (1 + 0.1 * gauss()), th: m.th * (1 + 0.1 * gauss()) + 0.5 };
    }
    if (k === kid) motion = null;                                      // déplacé à la main : aucune commande
    const scan = LL.simulateScan(opt.clutter ? D.scene.walls.concat(clutter) : D.scene.walls, truth, { tours: 1 }, D.rng);
    const pts = LL.uniformSubsample(LL.localPoints(scan), { max: 350 });
    const r = LL.trackerUpdate(T, pts, motion), e = T.pose ? LL.poseError(T.pose, truth) : { pos: Infinity, ang: Infinity };
    rows.push({ k, mode: r.mode, status: r.status, ms: r.ms, epos: e.pos, eang: e.ang, ok: e.pos <= 0.05 && e.ang <= 2 && r.status !== 'non localisé' });
    truthPrev = truth;
  }
  return rows;
}
// le kidnapping : à mi-parcours, le robot est « téléporté » à l'autre bout du trajet (la suite du trajet est décalée)
const half = Math.floor(D.path.length / 2), shift = D.path.slice(half).concat(D.path.slice(0, half));
md.push('## 2. Suivi en simulation (poses exactes connues)', '',
  `Salle simulée, carte de 9 scans, trajet de ${D.path.length} scans (pas de 15 cm, rotations de 15°). Odométrie : déplacement commandé avec 10 % d'erreur, plus un biais de 1 cm et 0,5° par pas.`, '',
  '| Scénario | Scans justes (≤ 5 cm, 2°) | Erreur médiane | Temps médian (suivi) | Recherches globales | Scans pour se relocaliser après le kidnapping |', '|---|---|---|---|---|---|');
for (const [label, opt, path] of [['Avec odométrie', { odom: true }, D.path], ['Sans odométrie (pose précédente seule)', { odom: false }, D.path],
                                  ['Salle encombrée : 25 objets absents de la carte', { odom: true, clutter: true }, D.path],
                                  ['Robot kidnappé à mi-parcours', { odom: true, kidnap: true }, null]]) {
  if (opt.kidnap) { D.path = D.path.slice(0, half).concat(shift.slice(half)); }
  const rows = run(label, opt);
  if (opt.kidnap) D.path = path || D.path;
  const okRows = rows.filter(r => r.ok), tr = rows.filter(r => r.mode === 'suivi');
  let rec = '–';
  if (opt.kidnap) { const j = rows.findIndex(r => r.k > half && r.ok); rec = j >= 0 ? `${rows[j].k - half}` : 'jamais'; }
  md.push(`| ${label} | ${okRows.length}/${rows.length} (${pct(okRows.length / rows.length)}) | ${(med(rows.filter(r => isFinite(r.epos)).map(r => r.epos)) * 100).toFixed(1)} cm, ${med(rows.filter(r => isFinite(r.eang)).map(r => r.eang)).toFixed(2)}° | ${med(tr.map(r => r.ms)).toFixed(0)} ms | ${rows.filter(r => r.mode !== 'suivi' && r.mode !== 'hypothèses').length} | ${rec} |`);
  console.log(label, okRows.length + '/' + rows.length, 'kidnap rec', rec, 'modes', rows.reduce((a, r) => (a[r.mode] = (a[r.mode] || 0) + 1, a), {}));
}
fs.writeFileSync(out + '.md', md.join('\n') + '\n');
console.log('\n' + md.join('\n'));
