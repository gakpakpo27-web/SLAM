// ============================================================================
//  Onglets « Grid map » et « Segments » (s'appuient sur LidarCore, voir lidar_core.js)
// ============================================================================
const LC = window.LidarCore;
let TAB = 'scans';
function setTab(t){
  TAB = t;
  document.querySelectorAll('.tabs button').forEach(b => b.classList.toggle('on', b.dataset.tab === t));
  document.querySelectorAll('.tab').forEach(d => d.hidden = d.id !== 'tab-' + t);
  $('tools').style.display = t === 'scans' ? '' : 'none';
  SEG.drawing = null;
  draw();
}
document.querySelectorAll('.tabs button').forEach(b => b.onclick = () => setTab(b.dataset.tab));
const numVal = id => parseFloat($(id).value);

// ---------------------------------------------------------------- GRID MAP
const GRID = { g: null, img: null, dirty: true, last: null, history: [] };
function gridParams(){
  return { res: numVal('gRes'), margin: numVal('gMargin'), model: $('gModel').value,
           free: numVal('gFree'), occ: numVal('gOcc'), occFree: numVal('gOccFree'), init: +$('gInit').value, fillBetween: $('gFill').checked };
}
function scanForGrid(s){
  const [X, Y] = toWorldPts(s), pts = new Array(X.length);
  for (let k = 0; k < X.length; k++) pts[k] = { x: X[k], y: Y[k] };
  return { robot: { x: s.pose.x, y: s.pose.y }, pts };
}
function gridSet(g, label, stats){
  if (GRID.g) { GRID.history.push(GRID.g); if (GRID.history.length > 10) GRID.history.shift(); }
  GRID.g = g; GRID.dirty = true; GRID.last = stats ? { label, ...stats } : { label };
  $('gShow').checked = true; renderGridInfo(); draw();
}
function needSel(){ const s = selScan(); if (!s){ toast('Sélectionne un scan (onglet Scans ou clic sur la carte)'); return null; } return s; }

// Outil 1 : scan + pose -> nouvelle carte d'occupation
function gridFromSelected(){
  const s = needSel(); if (!s) return;
  const p = gridParams(), { robot, pts } = scanForGrid(s);
  try {
    const t0 = performance.now(), { grid, stats } = LC.scanToGrid(robot, pts, p);
    gridSet(grid, `Nouvelle carte depuis « ${s.name} »`, { ...stats, ms: performance.now() - t0 });
  } catch (e){ toast(e.message); }
}
// Outil 2 : carte existante + scan + pose -> carte mise à jour
function gridUpdateSelected(){
  const s = needSel(); if (!s) return;
  if (!GRID.g) return toast('Pas encore de carte : crée-la ou charge-la d’abord');
  const p = gridParams(), { robot, pts } = scanForGrid(s);
  let g = { ...GRID.g, data: GRID.g.data.slice() };       // copie : l'ancienne carte reste annulable
  if ($('gExpand').checked){
    let x0 = robot.x, x1 = robot.x, y0 = robot.y, y1 = robot.y;
    for (const q of pts){ if (q.x < x0) x0 = q.x; if (q.x > x1) x1 = q.x; if (q.y < y0) y0 = q.y; if (q.y > y1) y1 = q.y; }
    g = LC.ensureCovers(g, x0, y0, x1, y1, p.margin);
  }
  const t0 = performance.now(), stats = LC.updateGrid(g, robot, pts, p);
  gridSet(g, `Mise à jour avec « ${s.name} »`, { ...stats, ms: performance.now() - t0 });
}
function gridFromAll(){
  const vis = S.scans.filter(s => s.visible);
  if (!vis.length) return toast('Aucun scan visible');
  const p = gridParams(), data = vis.map(scanForGrid);
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  data.forEach(({ robot, pts }) => [robot, ...pts].forEach(q => { if (q.x < x0) x0 = q.x; if (q.x > x1) x1 = q.x; if (q.y < y0) y0 = q.y; if (q.y > y1) y1 = q.y; }));
  try {
    const t0 = performance.now();
    const g = LC.createGrid({ xMin: x0 - p.margin, yMin: y0 - p.margin, xMax: x1 + p.margin, yMax: y1 + p.margin, res: p.res, init: p.init });
    let freed = 0, occupied = 0;
    data.forEach(({ robot, pts }) => { const st = LC.updateGrid(g, robot, pts, p); freed += st.freed; occupied += st.occupied; });
    gridSet(g, `Carte construite avec ${vis.length} scans`, { freed, occupied, outside: 0, ms: performance.now() - t0 });
  } catch (e){ toast(e.message); }
}
function renderGridInfo(){
  const g = GRID.g;
  if (!g){ $('gInfo').innerHTML = 'Aucune carte.'; return; }
  const st = LC.gridStats(g), L = GRID.last || {};
  const pc = v => (v / st.cells * 100).toFixed(1) + ' %';
  $('gInfo').innerHTML =
    `<b>${L.label || 'Carte'}</b><br>` +
    `${g.W} × ${g.H} cellules de ${(g.res * 100).toFixed(1)} cm = ${(g.W * g.res).toFixed(2)} × ${(g.H * g.res).toFixed(2)} m · ${(st.bytes / 1024).toFixed(0)} Ko (uint8)<br>` +
    `origine (${g.x0.toFixed(2)} ; ${g.y0.toFixed(2)}) m<br>` +
    `libres ${pc(st.free)} · occupées ${pc(st.occ)} · inconnues ${pc(st.unk)}` +
    (L.freed !== undefined ? `<br>dernière mise à jour : ${L.freed} cellules traversées, ${L.occupied} cellules d’impact` +
      (L.outside ? `, ${L.outside} points hors carte` : '') + (L.ms !== undefined ? ` · ${L.ms.toFixed(0)} ms` : '') : '');
}
function gridImage(){
  if (!GRID.dirty && GRID.img) return GRID.img;
  const g = GRID.g, c = document.createElement('canvas'); c.width = g.W; c.height = g.H;
  const cx = c.getContext('2d'), im = cx.createImageData(g.W, g.H), thr = $('gThresh').checked;
  for (let j = 0; j < g.H; j++){
    const row = (g.H - 1 - j) * g.W;                   // ligne j = 0 en bas
    for (let i = 0; i < g.W; i++){
      const raw = g.data[j * g.W + i], o = (row + i) * 4;
      const v = thr ? (raw <= LC.FREE_T ? 255 : raw >= LC.OCC_T ? 0 : 128) : 255 - raw;
      im.data[o] = v; im.data[o+1] = v; im.data[o+2] = v; im.data[o+3] = 255;
    }
  }
  cx.putImageData(im, 0, 0);
  GRID.img = c; GRID.dirty = false; return c;
}
function drawGridMap(){
  if (!GRID.g || !$('gShow').checked || TAB === 'proc') return;
  const g = GRID.g, [sx, sy] = w2s(g.x0, g.y0 + g.H * g.res);
  ctx.save(); ctx.imageSmoothingEnabled = false; ctx.globalAlpha = numVal('gAlpha');
  ctx.drawImage(gridImage(), sx, sy, g.W * g.res * S.view.s, g.H * g.res * S.view.s);
  ctx.restore();
  if (g.res * S.view.s >= 12){                          // lignes de cellules quand on zoome
    ctx.save(); ctx.strokeStyle = 'rgba(77,163,255,.18)'; ctx.lineWidth = 1; ctx.beginPath();
    const [ax, ay] = s2w(0, H), [bx, by] = s2w(W, 0);
    const i0 = Math.max(0, Math.floor((ax - g.x0) / g.res)), i1 = Math.min(g.W, Math.ceil((bx - g.x0) / g.res));
    const j0 = Math.max(0, Math.floor((ay - g.y0) / g.res)), j1 = Math.min(g.H, Math.ceil((by - g.y0) / g.res));
    for (let i = i0; i <= i1; i++){ const [px] = w2s(g.x0 + i * g.res, 0); ctx.moveTo(px + .5, Math.max(0, w2s(0, g.y0 + g.H * g.res)[1])); ctx.lineTo(px + .5, Math.min(H, w2s(0, g.y0)[1])); }
    for (let j = j0; j <= j1; j++){ const [, py] = w2s(0, g.y0 + j * g.res); ctx.moveTo(Math.max(0, w2s(g.x0, 0)[0]), py + .5); ctx.lineTo(Math.min(W, w2s(g.x0 + g.W * g.res, 0)[0]), py + .5); }
    ctx.stroke(); ctx.restore();
  }
}
function gridHud(wx, wy){
  if (!GRID.g || !$('gShow').checked) return '';
  const [i, j] = LC.worldToCell(GRID.g, wx, wy), v = LC.getCell(GRID.g, i, j);
  if (v === undefined) return '   ·   hors carte';
  const st = v <= LC.FREE_T ? 'libre' : v >= LC.OCC_T ? 'occupée' : 'inconnue';
  return `   ·   cellule (${i}, ${j}) = ${v} (${st})`;
}
$('gNew').onclick = gridFromSelected;
$('gUpd').onclick = gridUpdateSelected;
$('gAll').onclick = gridFromAll;
$('gUndo').onclick = () => { if (!GRID.history.length) return toast('Rien à annuler'); GRID.g = GRID.history.pop(); GRID.dirty = true; GRID.last = { label: 'Carte précédente restaurée' }; renderGridInfo(); draw(); };
$('gClear').onclick = () => { if (GRID.g) GRID.history.push(GRID.g); GRID.g = null; renderGridInfo(); draw(); };
$('gShow').onchange = draw; $('gAlpha').oninput = draw;
$('gThresh').onchange = () => { GRID.dirty = true; draw(); };
$('gPts').onchange = draw;
$('gModel').onchange = () => { const b = $('gModel').value === 'binary'; $('gFree').disabled = b; $('gOcc').disabled = b; $('gOccFree').disabled = b; };
$('gExpJson').onclick = () => { if (!GRID.g) return toast('Aucune carte'); download('grid_map.json', JSON.stringify(LC.gridToJSON(GRID.g)), 'application/json'); };
$('gExpPgm').onclick = () => {
  if (!GRID.g) return toast('Aucune carte');
  const { pgm, yaml } = LC.gridToPGM(GRID.g, 'grid_map');
  download('grid_map.pgm', pgm, 'image/x-portable-graymap');
  setTimeout(() => download('grid_map.yaml', yaml, 'text/yaml'), 300);
};
$('gExpPng').onclick = () => {
  if (!GRID.g) return toast('Aucune carte');
  const src = gridImage(), k = Math.max(1, Math.min(8, Math.floor(2000 / Math.max(GRID.g.W, GRID.g.H))));
  const c = document.createElement('canvas'); c.width = src.width * k; c.height = src.height * k;
  const x = c.getContext('2d'); x.imageSmoothingEnabled = false; x.drawImage(src, 0, 0, c.width, c.height);
  download('grid_map.png', c.toDataURL('image/png'));
};
$('gFile').onchange = async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  try { const g = LC.gridFromJSON(JSON.parse(await f.text())); gridSet(g, `Carte chargée : ${f.name}`); fitGrid(); }
  catch (err){ toast('Carte illisible : ' + err.message); }
};
function fitGrid(){
  const g = GRID.g; if (!g) return;
  S.view.x = g.x0 + g.W * g.res / 2; S.view.y = g.y0 + g.H * g.res / 2;
  S.view.s = Math.min(W / (g.W * g.res), H / (g.H * g.res)) * 0.9; draw();
}

// ---------------------------------------------------------------- SEGMENTS
const SEG = { list: [], A: -1, B: -1, tool: 'select', drawing: null, ghost: null, mark: null, history: [], nextId: 1 };
function segPush(){ SEG.history.push(JSON.stringify({ list: SEG.list, A: SEG.A, B: SEG.B })); if (SEG.history.length > 100) SEG.history.shift(); }
function segUndo(){
  const h = SEG.history.pop(); if (!h) return toast('Rien à annuler');
  const o = JSON.parse(h); SEG.list = o.list; SEG.A = o.A; SEG.B = o.B; SEG.ghost = null; SEG.mark = null; segRefresh();
}
function segAdd(s, src, color){ return Object.assign(s, { id: SEG.nextId++, src: src || 'manuel', color: color || '#e3e7ec' }); }
const segTol = () => ({ angTol: numVal('sAng'), distTol: numVal('sLat'), gapTol: numVal('sGap') });
const segExtractOpt = () => ({ gapBase: numVal('sGapBase'), gapK: numVal('sGapK'), splitTol: numVal('sSplit'),
                               minPts: Math.round(numVal('sMinPts')), minLen: numVal('sMinLen') });
function extractFrom(scans){
  segPush();
  let all = [];
  for (const s of scans){
    const [X, Y] = toWorldPts(s), pts = [];
    for (let k = 0; k < X.length; k++) pts.push({ x: X[k], y: Y[k] });
    LC.extractSegments(pts, { x: s.pose.x, y: s.pose.y }, segExtractOpt()).forEach(g => all.push(segAdd(g, s.name, s.color)));
  }
  const n0 = all.length; let merges = 0;
  if ($('sMergeAfter').checked && scans.length > 1){
    const r = LC.mergeAll(all, segTol()); merges = r.merges;
    all = r.segments.map(g => segAdd(g, 'fusion', '#e3e7ec'));
  }
  SEG.list = SEG.list.filter(g => !scans.some(s => s.name === g.src)).concat(all);
  SEG.A = SEG.B = -1; SEG.ghost = null; SEG.mark = null;
  segRefresh();
  toast(`${n0} segments extraits de ${scans.length} scan(s)` + (merges ? `, ${merges} fusions -> ${all.length} segments` : ''));
}
function segRefresh(){
  $('sCount').textContent = `${SEG.list.length} segment(s)`;
  const A = SEG.list[SEG.A], B = SEG.list[SEG.B];
  const desc = (s, n) => s ? `<b style="color:${n === 'A' ? 'var(--accent)' : '#ffb347'}">${n}</b> ${s.src} · L = ${LC.segLen(s).toFixed(3)} m · orientation ${(LC.segAngle(s) * 180 / Math.PI).toFixed(1)}°<br>` +
    `<span class="mono">(${s.x1.toFixed(3)} ; ${s.y1.toFixed(3)}) → (${s.x2.toFixed(3)} ; ${s.y2.toFixed(3)})</span>`
    : `<b style="color:${n === 'A' ? 'var(--accent)' : '#ffb347'}">${n}</b> <i>non sélectionné</i>`;
  let res = '';
  if (A && B && A !== B){
    const ang = LC.angleBetween(A, B), cl = LC.areClose(A, B, segTol()), it = LC.intersect(A, B);
    const yes = v => v ? '<span class="ok">oui</span>' : '<span class="ko">non</span>';
    res = `<div class="res">` +
      `<div><b>Angle</b> entre les droites : ${ang.lines.toFixed(2)}° (orienté A→B : ${ang.oriented.toFixed(2)}°)</div>` +
      `<div><b>Proches ?</b> ${yes(cl.close)} — angle ${cl.angle.toFixed(2)}° ${cl.angOk ? '≤' : '>'} ${segTol().angTol}°, ` +
      `écart latéral ${(cl.lateral * 100).toFixed(1)} cm ${cl.latOk ? '≤' : '>'} ${(segTol().distTol * 100).toFixed(0)} cm, ` +
      `trou ${(cl.gap * 100).toFixed(1)} cm ${cl.gapOk ? '≤' : '>'} ${(segTol().gapTol * 100).toFixed(0)} cm</div>` +
      `<div><b>Intersection</b> : ${
        it.type === 'point' ? `(${it.x.toFixed(3)} ; ${it.y.toFixed(3)}) sur les deux segments (t<sub>A</sub> = ${it.t.toFixed(3)}, t<sub>B</sub> = ${it.u.toFixed(3)})` :
        it.type === 'lines' ? `les droites se coupent en (${it.x.toFixed(3)} ; ${it.y.toFixed(3)}) mais hors ${!it.onS1 && !it.onS2 ? 'des deux segments' : !it.onS1 ? 'de A' : 'de B'}` :
        it.type === 'collinear' ? (it.overlap ? `colinéaires, partie commune de ${LC.segLen(it.overlap).toFixed(3)} m` : 'colinéaires, sans partie commune') :
        it.type === 'parallel' ? 'parallèles, pas d’intersection' : 'segment dégénéré'}</div></div>`;
    SEG.mark = (it.type === 'point' || it.type === 'lines') ? [it.x, it.y, it.type === 'point'] : null;
  } else SEG.mark = null;
  $('sAB').innerHTML = desc(A, 'A') + '<br>' + desc(B, 'B') + res;
  ['sMerge', 'sCutAB', 'sCutBA'].forEach(id => $(id).disabled = !(A && B && A !== B));
  ['sDel', 'sRev'].forEach(id => $(id).disabled = !A);
  draw();
}
function segHit(mx, my){
  let best = -1, bd = 8;
  SEG.list.forEach((s, k) => {
    const [x1, y1] = w2s(s.x1, s.y1), [x2, y2] = w2s(s.x2, s.y2);
    const d = LC.pointSegDist(mx, my, { x1, y1, x2, y2 });
    if (d < bd){ bd = d; best = k; }
  });
  return best;
}
function segOp(kind){
  const A = SEG.list[SEG.A], B = SEG.list[SEG.B];
  if (kind === 'merge'){
    const cl = LC.areClose(A, B, segTol());
    segPush();
    const m = segAdd(LC.mergeSegments(A, B), 'fusion', '#e3e7ec');
    const ia = SEG.A, ib = SEG.B;
    SEG.list = SEG.list.filter((_, k) => k !== ia && k !== ib); SEG.list.push(m);
    SEG.A = SEG.list.length - 1; SEG.B = -1; SEG.ghost = null;
    segRefresh();
    toast(cl.close ? 'A et B fusionnés' : 'Fusion faite, mais A et B ne sont pas « proches » selon les tolérances (Ctrl+Z pour annuler)');
    return;
  }
  const [ti, ci] = kind === 'cutAB' ? [SEG.A, SEG.B] : [SEG.B, SEG.A];
  const t = SEG.list[ti], c = SEG.list[ci], r = LC.cutSegment(t, c);
  if (!r.changed){ toast('Pas de découpe : ' + r.reason); SEG.ghost = null; draw(); return; }
  segPush();
  // partie supprimée affichée en pointillés rouges
  SEG.ghost = r.kept === 'premier' ? { x1: r.point[0], y1: r.point[1], x2: t.x2, y2: t.y2 } : { x1: t.x1, y1: t.y1, x2: r.point[0], y2: r.point[1] };
  Object.assign(SEG.list[ti], { x1: r.segment.x1, y1: r.segment.y1, x2: r.segment.x2, y2: r.segment.y2 });
  segRefresh();
  toast(`${kind === 'cutAB' ? 'A coupé par B' : 'B coupé par A'} : ${r.reason} (${r.lengths.map(v => v.toFixed(3)).join(' m / ')} m)`);
}
function drawSegments(){
  if (!SEG.list.length && !SEG.drawing) return;
  const dimOthers = SEG.A >= 0 || SEG.B >= 0;
  ctx.save(); ctx.lineCap = 'round';
  SEG.list.forEach((s, k) => {
    const sel = k === SEG.A ? 'A' : k === SEG.B ? 'B' : null;
    const [x1, y1] = w2s(s.x1, s.y1), [x2, y2] = w2s(s.x2, s.y2);
    ctx.globalAlpha = sel || !dimOthers ? 1 : 0.6;
    ctx.strokeStyle = sel === 'A' ? '#4da3ff' : sel === 'B' ? '#ffb347' : ($('sColor').checked ? s.color : '#e3e7ec');
    ctx.lineWidth = sel ? 4 : 2.2;
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    ctx.fillStyle = ctx.strokeStyle;                      // premier point : disque plein ; second : anneau
    ctx.beginPath(); ctx.arc(x1, y1, sel ? 4.5 : 2.5, 0, 2 * Math.PI); ctx.fill();
    if (sel){
      ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(x2, y2, 4.5, 0, 2 * Math.PI); ctx.stroke();
      ctx.font = '700 12px system-ui'; ctx.fillText(sel + '1', x1 + 7, y1 - 7); ctx.fillText(sel + '2', x2 + 7, y2 - 7);
    }
  });
  ctx.globalAlpha = 1;
  if (SEG.ghost){
    const [x1, y1] = w2s(SEG.ghost.x1, SEG.ghost.y1), [x2, y2] = w2s(SEG.ghost.x2, SEG.ghost.y2);
    ctx.strokeStyle = '#ff5c5c'; ctx.lineWidth = 2; ctx.setLineDash([6, 5]);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke(); ctx.setLineDash([]);
  }
  if (SEG.mark){
    const [x, y] = w2s(SEG.mark[0], SEG.mark[1]);
    ctx.strokeStyle = SEG.mark[2] ? '#ffe14d' : 'rgba(255,225,77,.5)'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x - 6, y - 6); ctx.lineTo(x + 6, y + 6); ctx.moveTo(x + 6, y - 6); ctx.lineTo(x - 6, y + 6); ctx.stroke();
    if (!SEG.mark[2]){ ctx.setLineDash([3, 4]); ctx.beginPath(); ctx.arc(x, y, 9, 0, 2 * Math.PI); ctx.stroke(); ctx.setLineDash([]); }
  }
  if (SEG.drawing){
    const d = SEG.drawing, [x1, y1] = w2s(d.x1, d.y1), [x2, y2] = w2s(d.x2, d.y2);
    ctx.strokeStyle = '#7dffb0'; ctx.lineWidth = 2.5; ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    ctx.font = '12px system-ui'; ctx.fillStyle = '#7dffb0'; ctx.fillText(`${Math.hypot(d.x2 - d.x1, d.y2 - d.y1).toFixed(2)} m`, x2 + 8, y2);
  }
  ctx.restore();
}
// interactions sur la carte quand l'onglet Segments est actif ; renvoie true si l'événement est consommé
function segPointerDown(e, mx, my){
  if (e.button !== 0) return false;
  if (SEG.tool === 'draw'){
    const [wx, wy] = s2w(mx, my);
    let [sx, sy] = [wx, wy];
    if ($('sSnap').checked){ const p = snapEnd(mx, my); if (p){ sx = p[0]; sy = p[1]; } }
    SEG.drawing = { x1: sx, y1: sy, x2: sx, y2: sy }; return true;
  }
  const h = segHit(mx, my);
  if (h < 0) return false;
  if (e.shiftKey) SEG.B = h; else SEG.A = h;
  if (SEG.A === SEG.B) SEG.B = -1;
  SEG.ghost = null; segRefresh(); return true;
}
function snapEnd(mx, my){                                 // aimantation sur une extrémité existante
  let best = null, bd = 10;
  for (const s of SEG.list) for (const [x, y] of [[s.x1, s.y1], [s.x2, s.y2]]){
    const [px, py] = w2s(x, y), d = Math.hypot(px - mx, py - my); if (d < bd){ bd = d; best = [x, y]; }
  }
  return best;
}
function segPointerMove(mx, my){
  if (!SEG.drawing) return false;
  let [wx, wy] = s2w(mx, my);
  if ($('sSnap').checked){ const p = snapEnd(mx, my); if (p){ wx = p[0]; wy = p[1]; } }
  SEG.drawing.x2 = wx; SEG.drawing.y2 = wy; draw(); return true;
}
function segPointerUp(){
  if (!SEG.drawing) return false;
  const d = SEG.drawing; SEG.drawing = null;
  if (Math.hypot(d.x2 - d.x1, d.y2 - d.y1) * S.view.s < 5){ draw(); return true; }
  segPush();
  SEG.list.push(segAdd(LC.seg(d.x1, d.y1, d.x2, d.y2), 'manuel', '#7dffb0'));
  // le segment tracé devient B si A existe déjà, sinon A
  if (SEG.A >= 0) SEG.B = SEG.list.length - 1; else SEG.A = SEG.list.length - 1;
  SEG.ghost = null; segRefresh(); return true;
}
function setSegTool(t){ SEG.tool = t; $('sToolSel').classList.toggle('on', t === 'select'); $('sToolDraw').classList.toggle('on', t === 'draw'); }
$('sToolSel').onclick = () => setSegTool('select');
$('sToolDraw').onclick = () => setSegTool('draw');
$('sExtractOne').onclick = () => { const s = needSel(); if (s) extractFrom([s]); };
$('sExtractAll').onclick = () => { const v = S.scans.filter(s => s.visible); if (!v.length) return toast('Aucun scan visible'); extractFrom(v); };
$('sMergeAll').onclick = () => {
  if (SEG.list.length < 2) return toast('Pas assez de segments');
  segPush(); const n0 = SEG.list.length, r = LC.mergeAll(SEG.list, segTol());
  SEG.list = r.segments.map(g => g.id ? g : segAdd(g, 'fusion', '#e3e7ec'));
  SEG.list.forEach(g => { if (!g.src) g.src = 'fusion'; });
  SEG.A = SEG.B = -1; segRefresh(); toast(`${r.merges} fusion(s) : ${n0} -> ${SEG.list.length} segments`);
};
$('sClean').onclick = () => {
  const L = numVal('sCleanLen'), n0 = SEG.list.length;
  const keep = SEG.list.filter(g => LC.segLen(g) >= L);
  if (keep.length === n0) return toast(`Aucun segment plus court que ${L} m`);
  segPush(); SEG.list = keep; SEG.A = SEG.B = -1; SEG.ghost = null; segRefresh();
  toast(`${n0 - keep.length} segment(s) de moins de ${L} m supprimé(s) (Ctrl+Z pour annuler)`);
};
$('sMerge').onclick = () => segOp('merge');
$('sCutAB').onclick = () => segOp('cutAB');
$('sCutBA').onclick = () => segOp('cutBA');
$('sSwap').onclick = () => { [SEG.A, SEG.B] = [SEG.B, SEG.A]; segRefresh(); };
$('sRev').onclick = () => { const s = SEG.list[SEG.A]; if (!s) return; segPush(); [s.x1, s.y1, s.x2, s.y2] = [s.x2, s.y2, s.x1, s.y1]; segRefresh(); };
$('sDel').onclick = () => {
  if (SEG.A < 0) return; segPush(); SEG.list.splice(SEG.A, 1);
  if (SEG.B > SEG.A) SEG.B--; else if (SEG.B === SEG.A) SEG.B = -1; SEG.A = -1; segRefresh();
};
$('sClear').onclick = () => { if (!SEG.list.length) return; segPush(); SEG.list = []; SEG.A = SEG.B = -1; SEG.ghost = null; segRefresh(); };
$('sUndo').onclick = segUndo;
$('sExp').onclick = () => { if (!SEG.list.length) return toast('Aucun segment'); download('carte_vectorielle.txt', LC.segmentsToText(SEG.list), 'text/plain'); };
$('sFile').onchange = async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  const L = LC.segmentsFromText(await f.text()); if (!L.length) return toast('Aucun segment lu');
  segPush(); L.forEach(g => SEG.list.push(segAdd(g, f.name.replace(/\.[^.]+$/, ''), '#c77dff'))); segRefresh(); toast(`${L.length} segments chargés`);
};
['sAng', 'sLat', 'sGap'].forEach(id => $(id).addEventListener('input', segRefresh));
$('sColor').onchange = draw; $('sHidePts').onchange = draw;
$('sTests').onclick = () => {
  const r = window.LidarTests.runTests(LC, window.LidarLoc, window.LidarMapTools), box = $('sTestOut');
  let html = `<b class="${r.passed === r.total ? 'ok' : 'ko'}">${r.passed}/${r.total} tests réussis</b>`, grp = '';
  for (const t of r.results){
    if (t.group !== grp){ grp = t.group; html += `<div class="tg">${grp}</div>`; }
    html += `<div class="${t.ok ? 'ok' : 'ko'}">${t.ok ? '✓' : '✗'} ${t.name}${t.ok ? '' : ' — ' + t.msg}</div>`;
  }
  box.innerHTML = html; box.hidden = false;
};
renderGridInfo(); segRefresh();

// ---------------------------------------------------------------- CARTE VECTORIELLE
const VEC = { map: LC.createVectorMap(), last: null, history: [], queue: [], qi: 0, sel: -1 };
function vecOpt(){
  return {
    angTol: numVal('vAng'), distTol: numVal('vLat'), gapTol: numVal('vGap'),
    confInit: 1, confHit: numVal('vHit'), confMiss: numVal('vMiss'), confMax: numVal('vMax'),
    absence: $('vAbs').checked, absMinRays: Math.round(numVal('vAbsRays')), absMargin: numVal('vAbsMargin'),
    maxAge: Math.round(numVal('vMaxAge')), minLen: numVal('vMinLen'), mergeRedundant: $('vMergeRed').checked,
    cornerAngle: numVal('vCornerAng'), cornerDist: 0.30,
    extract: segExtractOpt()                              // mêmes paramètres d'extraction que l'onglet Segments
  };
}
const vecSnapshot = () => JSON.stringify(VEC.map);
function vecPush(){ VEC.history.push({ map: vecSnapshot(), last: VEC.last, qi: VEC.qi }); if (VEC.history.length > 50) VEC.history.shift(); }
function vecUndo(){
  const h = VEC.history.pop(); if (!h) return toast('Rien à annuler');
  VEC.map = JSON.parse(h.map); VEC.last = h.last; VEC.qi = h.qi; VEC.sel = -1; vecRefresh();
}
function vecIntegrate(s){
  const [X, Y] = toWorldPts(s), pts = [];
  for (let k = 0; k < X.length; k++) pts.push({ x: X[k], y: Y[k] });
  const t0 = performance.now(), st = LC.updateVectorMap(VEC.map, { x: s.pose.x, y: s.pose.y }, pts, vecOpt());
  st.name = s.name; st.ms = performance.now() - t0;
  VEC.last = st; VEC.sel = -1;
  return st;
}
function vecRefresh(){
  const M = VEC.map, F = M.features, st = VEC.last;
  const meanConf = F.length ? F.reduce((a, f) => a + f.conf, 0) / F.length : 0;
  const total = F.reduce((a, f) => a + LC.segLen(f), 0);
  const mc = numVal('vMinConf'), shown = F.filter(f => f.conf >= mc).length;
  let html = `<b>${F.length} segments</b> (${total.toFixed(1)} m au total)${mc > 1 ? ` dont ${shown} de confiance ≥ ${mc}` : ''} · <b>${M.corners.length} coins</b> · ${M.scans} scan(s) intégré(s)` +
             (F.length ? `<br>confiance moyenne ${meanConf.toFixed(1)} / ${numVal('vMax')}` : '');
  if (st){
    const L = (lbl, a, col) => a && a.length !== undefined ? (a.length ? `<span style="color:${col}">${lbl} ${a.length}</span>` : '') : '';
    html += `<div class="res"><b>Scan ${st.scan}${st.name ? ' · « ' + st.name + ' »' : ''}</b> (${(st.ms || 0).toFixed(0)} ms)<br>` +
      `${st.extracted} segments extraits → ` + [
        L('ajoutés', st.added, '#4cd97b'), L('fusionnés', st.merged, '#4da3ff'),
        L('raccourcis', st.shortened, '#ffb347'), L('scindés', st.split, '#ffb347'),
        L('confiance −', st.weakened, '#ff8a65'), L('supprimés', st.removed, '#ff6b6b'), L('expirés', st.expired, '#ff6b6b'),
        st.fused ? `redondants fusionnés ${st.fused}` : ''].filter(Boolean).join(' · ') + '</div>';
  }
  const f = F[VEC.sel];
  if (f){
    html += `<div class="res"><b>Segment n°${f.id}</b> · L = ${LC.segLen(f).toFixed(3)} m · orientation ${(LC.segAngle(f) * 180 / Math.PI).toFixed(1)}°<br>` +
      `<span class="mono">(${f.x1.toFixed(3)} ; ${f.y1.toFixed(3)}) → (${f.x2.toFixed(3)} ; ${f.y2.toFixed(3)})</span><br>` +
      `confiance ${f.conf} · vu ${f.nObs} fois · scans ${f.firstSeen} → ${f.lastSeen} · ${f.n || 0} points</div>`;
  }
  $('vInfo').innerHTML = html;
  const n = VEC.queue.length;
  $('vStep').textContent = n ? (VEC.qi < n ? `Intégrer le scan suivant ▶ (${VEC.qi + 1}/${n} : ${VEC.queue[VEC.qi].name})` : `Terminé (${n}/${n})`) : 'Intégrer le scan suivant ▶';
  $('vStep').disabled = !n || VEC.qi >= n;
  draw();
}
function vecConfColor(c){
  const t = Math.max(0, Math.min(1, (c - 1) / Math.max(1, numVal('vMax') - 1)));
  const lerp = (a, b, u) => Math.round(a + (b - a) * u);
  // rouge-orangé (peu sûr) -> jaune -> vert (sûr)
  const [r, g, b] = t < 0.5 ? [lerp(255, 255, t * 2), lerp(122, 214, t * 2), lerp(77, 77, t * 2)]
                            : [lerp(255, 76, (t - 0.5) * 2), lerp(214, 217, (t - 0.5) * 2), lerp(77, 123, (t - 0.5) * 2)];
  return `rgb(${r},${g},${b})`;
}
function drawVectorMap(){
  if (TAB !== 'vec') return;
  const M = VEC.map, st = VEC.last, hl = $('vHl').checked && st;
  ctx.save(); ctx.lineCap = 'round';
  if (hl && st.ghosts) for (const g of st.ghosts){
    const [x1, y1] = w2s(g.x1, g.y1), [x2, y2] = w2s(g.x2, g.y2);
    ctx.strokeStyle = '#ff5c5c'; ctx.lineWidth = 2.5; ctx.setLineDash([6, 5]);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke(); ctx.setLineDash([]);
  }
  const maxC = numVal('vMax');
  const minC = numVal('vMinConf');
  M.features.forEach((f, k) => {
    if (f.conf < minC && k !== VEC.sel) return;          // filtre d'affichage (la carte garde tout)
    const [x1, y1] = w2s(f.x1, f.y1), [x2, y2] = w2s(f.x2, f.y2);
    if (hl){
      const glow = st.added.includes(f.id) ? 'rgba(76,217,123,.45)' : st.merged.includes(f.id) ? 'rgba(77,163,255,.40)'
                 : (st.shortened.includes(f.id) || st.split.includes(f.id)) ? 'rgba(255,179,71,.55)' : null;
      if (glow){ ctx.strokeStyle = glow; ctx.lineWidth = 10; ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke(); }
    }
    ctx.strokeStyle = k === VEC.sel ? '#4da3ff' : $('vConfCol').checked ? vecConfColor(f.conf) : '#e3e7ec';
    ctx.lineWidth = k === VEC.sel ? 4.5 : 1.8 + 2 * Math.min(1, f.conf / maxC);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
    if ($('vLabels').checked){
      ctx.fillStyle = ctx.strokeStyle; ctx.font = '11px system-ui';
      ctx.fillText(`${f.conf}`, (x1 + x2) / 2 + 4, (y1 + y2) / 2 - 4);
    }
  });
  if ($('vCorners').checked) for (const c of M.corners){
    if (c.conf < minC) continue;
    const [x, y] = w2s(c.x, c.y), th = c.th * Math.PI / 180;
    ctx.strokeStyle = '#ffe14d'; ctx.fillStyle = 'rgba(255,225,77,.25)'; ctx.lineWidth = 1.8;
    ctx.beginPath(); ctx.rect(x - 5, y - 5, 10, 10); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + 16 * Math.cos(th), y - 16 * Math.sin(th)); ctx.stroke();
  }
  ctx.restore();
}
function vecPointerDown(e, mx, my){
  if (e.button !== 0) return false;
  let best = -1, bd = 8;
  VEC.map.features.forEach((f, k) => {
    const [x1, y1] = w2s(f.x1, f.y1), [x2, y2] = w2s(f.x2, f.y2);
    const d = LC.pointSegDist(mx, my, { x1, y1, x2, y2 }); if (d < bd){ bd = d; best = k; }
  });
  if (best < 0) return false;
  VEC.sel = best; vecRefresh(); return true;
}
$('vNew').onclick = () => { vecPush(); VEC.map = LC.createVectorMap(); VEC.last = null; VEC.queue = []; VEC.qi = 0; VEC.sel = -1; vecRefresh(); toast('Carte vectorielle vide'); };
$('vOne').onclick = () => {
  const s = needSel(); if (!s) return;
  vecPush(); const st = vecIntegrate(s); vecRefresh();
  toast(`« ${s.name} » intégré : ${st.added.length} ajoutés, ${st.merged.length} fusionnés`);
};
$('vStepStart').onclick = () => {
  const vis = S.scans.filter(s => s.visible); if (!vis.length) return toast('Aucun scan visible');
  vecPush(); VEC.map = LC.createVectorMap(); VEC.last = null; VEC.sel = -1;
  VEC.queue = vis; VEC.qi = 0; vecRefresh();
  toast(`Pas à pas : ${vis.length} scans à intégrer, dans l’ordre de la liste`);
};
$('vStep').onclick = () => {
  if (VEC.qi >= VEC.queue.length) return;
  vecPush(); const s = VEC.queue[VEC.qi++]; vecIntegrate(s); vecRefresh();
};
$('vAll').onclick = () => {
  const vis = S.scans.filter(s => s.visible); if (!vis.length) return toast('Aucun scan visible');
  vecPush(); VEC.map = LC.createVectorMap(); VEC.queue = []; VEC.qi = 0;
  const t0 = performance.now(); let added = 0;
  vis.forEach(s => { added += vecIntegrate(s).added.length; });
  vecRefresh();
  toast(`${vis.length} scans intégrés en ${(performance.now() - t0).toFixed(0)} ms : ${VEC.map.features.length} segments, ${VEC.map.corners.length} coins`);
};
$('vUndo').onclick = vecUndo;
$('vExp').onclick = () => { if (!VEC.map.features.length) return toast('Carte vide'); download('carte_vectorielle_proprietes.txt', LC.vectorMapToText(VEC.map), 'text/plain'); };
$('vFile').onchange = async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  const M = LC.vectorMapFromText(await f.text(), vecOpt());
  if (!M.features.length) return toast('Aucun segment lu');
  vecPush(); VEC.map = M; VEC.last = null; VEC.sel = -1; VEC.queue = []; VEC.qi = 0; vecRefresh();
  toast(`Carte chargée : ${M.features.length} segments (servira de carte initiale)`);
};
$('vToSeg').onclick = () => {
  if (!VEC.map.features.length) return toast('Carte vide');
  segPush();
  SEG.list = VEC.map.features.map(f => segAdd(LC.seg(f.x1, f.y1, f.x2, f.y2), 'carte n°' + f.id, '#e3e7ec'));
  SEG.A = SEG.B = -1; segRefresh(); setTab('seg'); toast(`${SEG.list.length} segments copiés dans l’onglet Segments`);
};
['vConfCol', 'vCorners', 'vHl', 'vLabels', 'vHidePts'].forEach(id => $(id).onchange = draw);
$('vMinConf').oninput = () => { $('vMinConfV').textContent = $('vMinConf').value; vecRefresh(); };
vecRefresh();

// ---------------------------------------------------------------- LOCALISATION (Jour 2)
const LL = window.LidarLoc;
const LOC = { refPoses: new Map(), cache: null, cacheKey: '', bench: null, stop: false, busy: false };
// poses de référence : celles des scans au moment où on les mémorise (après superposition)
function locMemorize(quiet){
  LOC.refPoses = new Map(S.scans.map(s => [s.id, { ...s.pose }]));
  if (!quiet) toast(`Poses de référence mémorisées pour ${S.scans.length} scans`);
  locRefresh();
}
function rawScan(s){ return { name: s.name, a: s.a, d: s.d, i: s.i, pose: { ...s.pose } }; }
function locMethodsFor(ref){ return Object.entries(LL.METHODS[ref]).map(([k, [label]]) => [k, label]); }
function locFillMethods(){
  const ref = $('locRef').value, sel = $('locMethod'), prev = sel.value;
  sel.innerHTML = locMethodsFor(ref).map(([k, l]) => `<option value="${k}">${l}</option>`).join('');
  if ([...sel.options].some(o => o.value === prev)) sel.value = prev;
  $('locScanRefRow').style.display = ref === 'scan' ? '' : 'none';
  const sc = $('locScanRef'), pv = sc.value;
  sc.innerHTML = S.scans.map(s => `<option value="${s.id}">${s.name}</option>`).join('');
  if ([...sc.options].some(o => o.value === pv)) sc.value = pv;
}
function locRefresh(){
  locFillMethods();
  const s = selScan(), rp = s && LOC.refPoses.get(s.id);
  $('locTarget').textContent = s ? `« ${s.name} » · pose actuelle (${s.pose.x.toFixed(2)} ; ${s.pose.y.toFixed(2)} ; ${s.pose.th.toFixed(1)}°)` +
    (rp ? ` · référence (${rp.x.toFixed(2)} ; ${rp.y.toFixed(2)} ; ${rp.th.toFixed(1)}°)` : ' · pas de pose de référence mémorisée') : 'aucun scan sélectionné';
}
async function locRun(){
  const s = selScan(); if (!s) return toast('Sélectionne le scan à localiser');
  if (!LOC.refPoses.size) locMemorize(true);
  const ref = $('locRef').value, m = $('locMethod').value, [label, fn] = LL.METHODS[ref][m];
  const pts = LL.uniformSubsample(LL.localPoints(rawScan(s)), { max: 350 });
  let R, init, toWorld = p => p;
  if (ref === 'scan'){
    const rs = S.scans.find(o => String(o.id) === $('locScanRef').value);
    if (!rs || rs === s) return toast('Choisis un scan de référence différent du scan à localiser');
    const all = LL.localPoints(rawScan(rs));
    R = { pts: LL.uniformSubsample(all, { cell: 0.03, minR: 0.3, max: 3000 }), ptsAll: all };
    init = LL.relPose(rs.pose, s.pose); toWorld = p => LL.compose(rs.pose, p);
  } else {
    // carte construite avec les scans visibles SAUF celui à localiser (sinon il se retrouverait lui-même)
    const others = S.scans.filter(o => o.visible && o !== s);
    if (!others.length) return toast('Il faut d’autres scans visibles pour construire la carte de référence');
    const key = others.map(o => `${o.id}:${o.pose.x.toFixed(4)},${o.pose.y.toFixed(4)},${o.pose.th.toFixed(3)}`).join('|');
    if (LOC.cacheKey !== key){ setStatusLoc('Construction de la carte de référence…'); await sleep(20); LOC.cache = LL.buildReferences(others.map(rawScan), -1); LOC.cacheKey = key; }
    R = LOC.cache; init = { ...s.pose };
  }
  setStatusLoc(`${label}…`); await sleep(20);
  const t0 = performance.now(); let r;
  try { r = fn(R, pts, init, { rxy: numVal('locRxy'), rth: numVal('locRth') }); } catch (e){ setStatusLoc(''); return toast('Erreur : ' + e.message); }
  const ms = performance.now() - t0;
  if (r.failed){ setStatusLoc(''); return toast(`${label} : aucun résultat (pas assez de coins visibles)`); }
  const w = toWorld(r), rp = LOC.refPoses.get(s.id);
  pushUndo(s); s.pose = { x: w.x, y: w.y, th: normDeg(w.th) }; updateInputs(s); draw();
  const e = rp ? LL.poseError(s.pose, rp) : null;
  $('locResult').innerHTML = `<b>${label}</b> · ${ms.toFixed(0)} ms · ${r.iters ?? '—'} itérations${r.hypotheses !== undefined ? ` · ${r.hypotheses} hypothèses` : ''}<br>` +
    `pose trouvée (${s.pose.x.toFixed(3)} ; ${s.pose.y.toFixed(3)} ; ${s.pose.th.toFixed(2)}°)` +
    (e ? `<br>écart à la référence : <b class="${e.pos <= numVal('benchPos') && e.ang <= numVal('benchAng') ? 'ok' : 'ko'}">${(e.pos * 100).toFixed(1)} cm, ${e.ang.toFixed(2)}°</b>` : '');
  setStatusLoc('');
}
function setStatusLoc(t){ $('locStatus').textContent = t; }
function locPerturb(){
  const s = selScan(); if (!s) return toast('Sélectionne un scan');
  if (!LOC.refPoses.has(s.id)) locMemorize(true);
  const p = LL.perturb(LOC.refPoses.get(s.id), numVal('locPd'), numVal('locPa'), Math.random);
  pushUndo(s); s.pose = p; updateInputs(s); draw(); locRefresh();
  toast(`« ${s.name} » déplacé de ${numVal('locPd')} m et ${numVal('locPa')}° : à toi de le relocaliser`);
}
function locRestore(){
  const s = selScan(), rp = s && LOC.refPoses.get(s.id); if (!rp) return;
  pushUndo(s); s.pose = { ...rp }; updateInputs(s); draw(); locRefresh();
}

// ---- banc d'évaluation
function benchCfg(){
  const levels = $('benchLevels').value.split(/[;\n]/).map(t => t.trim()).filter(Boolean).map(t => t.split(/[\/, ]+/).map(Number)).filter(v => v.length >= 2 && v.every(isFinite));
  const methods = {}, refs = [];
  document.querySelectorAll('#benchMethods input:checked').forEach(c => { const [r, m] = c.value.split('.'); (methods[r] ||= []).push(m); if (!refs.includes(r)) refs.push(r); });
  return { levels, methods, refs: ['grid', 'vector', 'scan'].filter(r => refs.includes(r)), draws: Math.max(1, Math.round(numVal('benchDraws'))), seed: Math.round(numVal('benchSeed')),
           success: { pos: numVal('benchPos') / 100, ang: numVal('benchAng') }, coarse: { pos: 0.2, ang: 5 }, window: { rxy: numVal('locRxy'), rth: numVal('locRth') },
           sim: { nTest: Math.max(1, Math.round(numVal('benchNSim'))) } };
}
async function benchRun(){
  if (LOC.busy) return;
  const cfg = benchCfg(); if (!cfg.refs.length) return toast('Coche au moins une méthode');
  if (!cfg.levels.length) return toast('Niveaux de perturbation invalides');
  const doSim = $('benchSim').checked, doReal = $('benchReal').checked;
  if (!doSim && !doReal) return toast('Choisis simulation et/ou scans réels');
  const real = S.scans.filter(s => s.visible);
  if (doReal && real.length < 3) return toast('Il faut au moins 3 scans visibles et superposés pour le banc réel');
  LOC.busy = true; LOC.stop = false; $('benchGo').disabled = true; $('autoOverlay').style.display = 'flex';
  $('aoStop').onclick = () => { LOC.stop = true; };
  const results = [];
  const prog = async (label) => async (f, t) => {
    if (LOC.stop) throw new Error('arrêt');
    $('aoText').textContent = `${label} · ${t}`; $('aoBar').style.width = Math.round(f * 100) + '%'; $('aoSub').textContent = `${Math.round(f * 100)} %`;
    await sleep(0);
  };
  document.querySelector('#autoOverlay .ao-title').textContent = 'Banc de localisation en cours';
  try {
    if (doSim) results.push(['Simulation', await LL.runLocBench({ ...cfg, scenario: 'sim' }, await prog('Simulation'))]);
    if (doReal){
      if (!LOC.refPoses.size) locMemorize(true);
      const scans = real.map(s => ({ ...rawScan(s), pose: { ...(LOC.refPoses.get(s.id) || s.pose) } }));
      results.push([`Scans réels (${scans.length})`, await LL.runLocBench({ ...cfg, scenario: 'real', real: { scans } }, await prog('Scans réels'))]);
    }
    LOC.bench = results; benchShow();
  } catch (e){ toast(e.message === 'arrêt' ? 'Banc arrêté' : 'Erreur : ' + e.message); }
  finally {
    LOC.busy = false; $('benchGo').disabled = false; $('autoOverlay').style.display = 'none';
    document.querySelector('#autoOverlay .ao-title').textContent = 'Superposition automatique en cours';
    $('aoStop').onclick = () => { autoCancel = true; setAutoStatus('Arrêt après le scan en cours…'); };
  }
}
const BENCH_COLORS = ['#4da3ff', '#4cd97b', '#ffb347', '#c77dff', '#ff6b6b', '#3ee6e6'];
function benchShow(){
  if (!LOC.bench) return toast('Lance d’abord le banc');
  let html = '';
  LOC.bench.forEach(([title, res], bi) => {
    const C = res.cfg;
    html += `<h3>${title}</h3><div class="help">Réussite : ≤ ${C.success.pos * 100} cm et ≤ ${C.success.ang}° · ${C.draws} tirages par niveau · fenêtre des méthodes globales ±${C.window.rxy} m / ±${C.window.rth}°. ` +
            `Chaque case : taux de réussite, puis erreur médiane (position / angle) et temps moyen.</div>`;
    for (const ref of C.refs){
      const G = res.summary.filter(g => g.ref === ref), meths = [...new Set(G.map(g => g.method))];
      html += `<h4>${LL.REF_LABEL[ref]}</h4><div class="bench-wrap"><table class="bench"><tr><th>Méthode \\ perturbation</th>` +
              C.levels.map(([dp, da]) => `<th>${dp} m / ${da}°</th>`).join('') + '</tr>';
      for (const m of meths){
        html += `<tr><td>${G.find(g => g.method === m).label}</td>` + C.levels.map((_, l) => {
          const g = G.find(x => x.method === m && x.level === l); if (!g) return '<td>—</td>';
          const col = g.success >= 0.9 ? '#4cd97b' : g.success >= 0.5 ? '#ffb347' : '#ff6b6b';
          return `<td><b style="color:${col}">${Math.round(g.success * 100)} %</b><br><span class="mono">${isFinite(g.medPos) ? (g.medPos * 100).toFixed(1) + ' cm' : '—'} / ${isFinite(g.medAng) ? g.medAng.toFixed(2) + '°' : '—'}<br>${g.meanMs.toFixed(0)} ms</span></td>`;
        }).join('') + '</tr>';
      }
      html += `</table><canvas class="bench-chart" data-b="${bi}" data-ref="${ref}" width="1120" height="400"></canvas></div>`;
    }
  });
  $('benchBody').innerHTML = html; $('benchModal').style.display = 'flex';
  document.querySelectorAll('.bench-chart').forEach(cv2 => benchChart(cv2, LOC.bench[+cv2.dataset.b][1], cv2.dataset.ref));
}
// courbe de sensibilité : taux de réussite en fonction de la perturbation initiale
function benchChart(c, res, ref){
  const x = c.getContext('2d'); x.setTransform(2, 0, 0, 2, 0, 0);          // dessin en 560 × 200, rendu net (×2)
  const W2 = 560, H2 = 200, L = res.cfg.levels, G = res.summary.filter(g => g.ref === ref);
  const meths = [...new Set(G.map(g => g.method))], left = 44, right = W2 - 190;          // légende à droite
  const px = l => left + l * (right - left) / Math.max(1, L.length - 1), py = v => H2 - 28 - v * (H2 - 48);
  x.fillStyle = '#0f1216'; x.fillRect(0, 0, W2, H2);
  x.strokeStyle = '#2a313b'; x.fillStyle = '#8b95a3'; x.font = '10px system-ui'; x.lineWidth = 1;
  for (const v of [0, 0.25, 0.5, 0.75, 1]){ x.beginPath(); x.moveTo(left, py(v)); x.lineTo(right, py(v)); x.stroke(); x.fillText(Math.round(v * 100) + ' %', 6, py(v) + 3); }
  L.forEach(([dp, da], l) => { const t = `${dp} m / ${da}°`; x.fillText(t, px(l) - x.measureText(t).width / 2, H2 - 10); });
  x.fillText('taux de réussite selon la perturbation initiale', left, 12);
  meths.forEach((m, k) => {
    const col = BENCH_COLORS[k % BENCH_COLORS.length];
    x.strokeStyle = x.fillStyle = col; x.lineWidth = 2; x.beginPath();
    L.forEach((_, l) => { const g = G.find(q => q.method === m && q.level === l); if (!g) return; l ? x.lineTo(px(l), py(g.success)) : x.moveTo(px(l), py(g.success)); });
    x.stroke();
    L.forEach((_, l) => { const g = G.find(q => q.method === m && q.level === l); if (g){ x.beginPath(); x.arc(px(l), py(g.success), 3, 0, 7); x.fill(); } });
    x.fillRect(right + 14, 30 + k * 18, 12, 3);
    x.fillStyle = '#e3e7ec'; x.font = '10.5px system-ui';
    const lab = G.find(g => g.method === m).label; x.fillText(lab.length > 30 ? lab.slice(0, 29) + '…' : lab, right + 32, 34 + k * 18);
  });
}
$('locRef').onchange = locRefresh;
$('locGo').onclick = locRun;
$('locMem').onclick = () => locMemorize(false);
$('locPert').onclick = locPerturb;
$('locBack').onclick = locRestore;
$('benchGo').onclick = benchRun;
$('benchView').onclick = benchShow;
$('benchClose').onclick = () => { $('benchModal').style.display = 'none'; };
$('benchCsv').onclick = () => { if (!LOC.bench) return; download('resultats_localisation.csv', LOC.bench.map(([, r], k) => { const t = LL.benchToCSV(r); return k ? t.split('\n').slice(1).join('\n') : t; }).join(''), 'text/csv'); };
$('benchMd').onclick = () => { if (!LOC.bench) return; download('resultats_localisation.md', '# Résultats du banc de localisation\n\n' + LOC.bench.map(([t, r]) => LL.benchToMarkdown(r, t)).join('\n'), 'text/markdown'); };
$('benchPng').onclick = () => { const c = document.querySelector('.bench-chart'); if (c) download('sensibilite_localisation.png', c.toDataURL('image/png')); };
(() => {   // cases à cocher des méthodes
  const box = $('benchMethods');
  box.innerHTML = Object.entries(LL.METHODS).map(([ref, ms]) => `<div class="help" style="margin-top:4px"><b>${LL.REF_LABEL[ref]}</b></div>` +
    Object.entries(ms).map(([k, [l]]) => `<label class="chk"><input type="checkbox" value="${ref}.${k}" checked> ${l}</label>`).join('')).join('');
})();
locRefresh();

// ---------------------------------------------------------------- TRAITEMENT (cartes et obstacles)
const LM = window.LidarMapTools;
const PROC = { res: null, imgs: {}, scans: [], busy: false, stop: false, planMode: false, plan: { start: null, goal: null, result: null } };
function procOpt(){
  const age = Math.round(numVal('pAge'));
  return {
    refine: $('pRefine').checked,
    pre: { dMin: numVal('pDMin'), dMax: numVal('pDMax'), iMin: numVal('pIMin'), isolated: $('pIso').checked, mixed: $('pMixed').checked },
    res: numVal('pRes'),
    mobile: $('pMobile').checked, mobileOpt: { maxPersist: numVal('pPersist') / 100 },
    speckles: $('pSpeck').checked, smooth: $('pSmooth').checked, close: $('pClose').checked,
    obsolete: $('pObsolete').checked, obsoleteOpt: { age }, decay: $('pDecay').checked, decayOpt: { age, step: 16 },
    features: { method: $('pMethod').value, smallMax: numVal('pSmall') },
    obstacleOpt: { smallMax: numVal('pSmall') },
    vmOpt: { extract: Object.assign(segExtractOpt(), { minLen: 0.3 }) },
    sharpen: $('pSharp').checked, occFree: numVal('gOccFree') || 15,
    plan: $('pPlan').checked, planOpt: planOpt(),
    regularize: $('pReg').checked, regOpt: { snapTol: numVal('pSnap'), orthogonal: numVal('pSnap') > 0 },
    filters: filterOpt(),
    cost: { robotRadius: numVal('pRobotR'), inflation: numVal('pInfl') }
  };
}
function planOpt(){ return { objMax: numVal('pObjMax') || 1, leak: numVal('pLeak') || 0, notch: numVal('pNotch') || 0 }; }
async function procRun(){
  if (PROC.busy) return;
  const vis = S.scans.filter(s => s.visible);
  if (!vis.length) return toast('Aucun scan visible');
  const o = procOpt();
  PROC.busy = true; PROC.stop = false; $('pRun').disabled = true;
  const overlay = o.refine && vis.length > 1;
  if (overlay){
    $('autoOverlay').style.display = 'flex';
    document.querySelector('#autoOverlay .ao-title').textContent = 'Traitement de la carte';
    $('aoStop').onclick = () => { PROC.stop = true; };
  }
  const prog = async (f, t) => {
    if (PROC.stop) throw new Error('arrêt');
    $('pStatus').textContent = `${Math.round(f * 100)} % · ${t}`;
    if (overlay){ $('aoText').textContent = t; $('aoBar').style.width = Math.round(f * 100) + '%'; $('aoSub').textContent = `${Math.round(f * 100)} %`; }
    await sleep(0);
  };
  try {
    PROC.scans = vis.map(s => ({ id: s.id, name: s.name, a: s.a, d: s.d, i: s.i, scale: s.scale || 1, pose: { ...s.pose } }));
    const t0 = performance.now();
    PROC.res = await LM.processMap(PROC.scans, o, prog);
    PROC.res.ms = performance.now() - t0;
    PROC.imgs = {}; PROC.plan = { start: null, goal: null, result: null }; PROC.undo = [];
    $('pStatus').textContent = `terminé en ${(PROC.res.ms / 1000).toFixed(1)} s`;
    if (PROC.res.plan) $('pLayer').value = 'plan'; else if ($('pLayer').value === 'plan') $('pLayer').value = 'clean';
    procInfo(); procPlanInfo(); if (typeof procEInfo === 'function') procEInfo();
  } catch (e){ $('pStatus').textContent = ''; toast(e.message === 'arrêt' ? 'Traitement arrêté' : 'Erreur : ' + e.message); console.error(e); }
  finally {
    PROC.busy = false; $('pRun').disabled = false;
    if (overlay){
      $('autoOverlay').style.display = 'none';
      document.querySelector('#autoOverlay .ao-title').textContent = 'Superposition automatique en cours';
      $('aoStop').onclick = () => { autoCancel = true; setAutoStatus('Arrêt après le scan en cours…'); };
    }
    draw();
  }
}
function procInfo(){
  const R = PROC.res; if (!R){ $('pInfo').innerHTML = 'Aucun traitement.'; return; }
  const q = R.quality, pc = v => (v * 100).toFixed(1) + ' %';
  const row = (l, a, b) => `<tr><td>${l}</td><td>${a}</td><td>${b}</td></tr>`;
  let h = `<b>${PROC.scans.length} scans traités</b> en ${(R.ms / 1000).toFixed(1)} s<table class="bench" style="margin-top:6px"><tr><th>Étape</th><th>Résultat</th><th>ms</th></tr>`;
  for (const st of R.steps) h += `<tr><td>${st.step}</td><td style="text-align:left">${st.detail}</td><td>${st.ms.toFixed(0)}</td></tr>`;
  h += `</table><table class="bench" style="margin-top:6px"><tr><th>Qualité de la grid map</th><th>brute</th><th>traitée</th></tr>` +
    row('Points des scans sur un obstacle', pc(q.raw.pointsOnObstacle), pc(q.clean.pointsOnObstacle)) +
    row('Distance moyenne point → obstacle', (q.raw.meanDist * 100).toFixed(1) + ' cm', (q.clean.meanDist * 100).toFixed(1) + ' cm') +
    row('Cellules occupées', q.raw.occCells, q.clean.occCells) +
    row('Surface libre', q.raw.freeArea.toFixed(1) + ' m²', q.clean.freeArea.toFixed(1) + ' m²') +
    row('Netteté (cellules tranchées)', pc(q.raw.sharpness), pc(q.clean.sharpness)) +
    row('Cellules en conflit', q.raw.conflicts, q.clean.conflicts) + '</table>';
  if (R.refineLog){
    const last = new Map(); R.refineLog.forEach(l => last.set(l.name, l));
    const moved = R.poses.map((p, k) => ({ n: PROC.scans[k].name, d: Math.hypot(p.x - PROC.scans[k].pose.x, p.y - PROC.scans[k].pose.y), a: Math.abs(((p.th - PROC.scans[k].pose.th + 540) % 360) - 180) }));
    const mx = moved.reduce((a, m) => m.d > a.d ? m : a, { d: 0 });
    h += `<div class="res">Recalage : correction médiane ${(median(moved.map(m => m.d)) * 100).toFixed(1)} cm, max ${(mx.d * 100).toFixed(1)} cm (${mx.n || '–'}). « Appliquer les poses recalées » pour les garder.</div>`;
  }
  $('pInfo').innerHTML = h;
}
function median(a){ const b = a.slice().sort((x, y) => x - y); return b.length ? b[Math.floor(b.length / 2)] : 0; }

// --- images des couches (1 pixel par cellule, mises en cache)
function procLayerImage(kind){
  const R = PROC.res; if (!R || kind === 'none') return null;
  if (PROC.imgs[kind]) return PROC.imgs[kind];
  const g = R.grid, Wg = g.W, Hg = g.H, c = document.createElement('canvas'); c.width = Wg; c.height = Hg;
  const cx = c.getContext('2d'), im = cx.createImageData(Wg, Hg), D = im.data, L = R.layers;
  const CLS = { structure: [227, 231, 236], objet: [255, 179, 71], petit: [255, 92, 138] };
  const clsOf = R.obstacles.map(o => CLS[o.cls]);
  for (let j = 0; j < Hg; j++) for (let i = 0; i < Wg; i++){
    const k = j * Wg + i, o = ((Hg - 1 - j) * Wg + i) * 4;
    let r, gg, b, a = 255;
    const gray = v => { const x = 255 - v; return [x, x, x]; };
    if (kind === 'plan') [r, gg, b] = R.plan ? (R.plan.grid.data[k] === 0 ? [246, 247, 249] : R.plan.grid.data[k] === 255 ? [30, 34, 40] : [120, 126, 134]) : [120, 126, 134];
    else if (kind === 'clean') [r, gg, b] = gray(g.data[k]);
    else if (kind === 'raw') [r, gg, b] = gray(R.gridRaw.data[k]);
    else if (kind === 'diff'){
      const before = R.gridRaw.data[k] >= LC.OCC_T, after = g.data[k] >= LC.OCC_T;
      if (before && !after) [r, gg, b] = [255, 70, 70];
      else if (!before && after) [r, gg, b] = [70, 230, 110];
      else if (R.mobile[k]) [r, gg, b] = [190, 90, 255];
      else { const x = 255 - g.data[k]; [r, gg, b] = [x * 0.55, x * 0.55, x * 0.55]; }
    } else if (kind === 'persist'){
      if (!L.seen[k]) [r, gg, b] = [20, 24, 30];
      else if (!L.hits[k]) [r, gg, b] = [45, 52, 64];
      else { const p = L.hits[k] / L.seen[k]; [r, gg, b] = [Math.round(80 + 175 * p), Math.round(60 + 170 * p), Math.round(200 * (1 - p))]; }
    } else if (kind === 'obst'){
      const id = R.labels[k];
      if (id >= 0) [r, gg, b] = clsOf[id];
      else { const v = g.data[k]; [r, gg, b] = v <= LC.FREE_T ? [70, 76, 86] : [26, 30, 36]; }
    } else if (kind === 'cost'){
      const v = R.cost.cost[k];
      if (v === 254) [r, gg, b] = [0, 0, 0];
      else if (v === 253) [r, gg, b] = [120, 40, 160];
      else if (v === 255) [r, gg, b] = [40, 48, 60];
      else if (v === 0) [r, gg, b] = [235, 240, 235];
      else { const t = v / 252; [r, gg, b] = [255, Math.round(230 - 180 * t), Math.round(120 - 110 * t)]; }
    }
    D[o] = r; D[o + 1] = gg; D[o + 2] = b; D[o + 3] = a;
  }
  cx.putImageData(im, 0, 0);
  return PROC.imgs[kind] = c;
}
function drawProcUnder(){
  const R = PROC.res; if (!R) return;
  const img = procLayerImage($('pLayer').value); if (!img) return;
  const g = R.grid, [sx, sy] = w2s(g.x0, g.y0 + g.H * g.res);
  ctx.save(); ctx.imageSmoothingEnabled = false; ctx.globalAlpha = numVal('pAlpha');
  ctx.drawImage(img, sx, sy, g.W * g.res * S.view.s, g.H * g.res * S.view.s);
  ctx.restore();
}
function drawProcOver(){
  const R = PROC.res;
  ctx.save(); ctx.lineCap = 'round';
  const line = (f) => { const [x1, y1] = w2s(f.x1, f.y1), [x2, y2] = w2s(f.x2, f.y2); ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke(); };
  const planLayer = R && R.plan && $('pLayer').value === 'plan';
  if (planLayer){
    const poly = (pts, col) => { if (!pts.length) return; ctx.strokeStyle = col; ctx.lineWidth = 3.5; ctx.lineJoin = 'miter'; ctx.beginPath();
      pts.forEach((p, k) => { const [x, y] = w2s(p[0], p[1]); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.closePath(); ctx.stroke();
      ctx.fillStyle = '#ffe14d'; for (const p of pts){ const [x, y] = w2s(p[0], p[1]); ctx.fillRect(x - 3, y - 3, 6, 6); } };
    poly(R.plan.outer, '#4da3ff'); R.plan.islands.forEach(I => poly(I, '#ffb347'));
    ctx.strokeStyle = '#ffb347'; ctx.lineWidth = 3.5; ctx.lineCap = 'round';
    for (const [a, b] of R.plan.innerWalls || []){ const p = w2s(a[0], a[1]), q = w2s(b[0], b[1]); ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke(); }
  }
  if (R && $('pShowVecRaw').checked && !planLayer){
    ctx.strokeStyle = 'rgba(255,214,77,.9)'; ctx.lineWidth = 1.5; ctx.setLineDash([5, 4]);
    R.vmapRaw.features.forEach(line); ctx.setLineDash([]);
  }
  if (R && $('pShowVec').checked && !planLayer){
    ctx.strokeStyle = '#4da3ff'; ctx.lineWidth = 3; R.vmap.features.forEach(line);
    for (const c of R.vmap.corners){
      const [x, y] = w2s(c.x, c.y); ctx.strokeStyle = '#ffe14d'; ctx.fillStyle = 'rgba(255,225,77,.3)'; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.rect(x - 4, y - 4, 8, 8); ctx.fill(); ctx.stroke();
    }
  }
  if (R && $('pShowCirc').checked && !planLayer){
    const mc = numVal('pMinConf');
    for (const it of R.omap.items){
      if (it.conf < mc && !it.mobile) continue;
      const [x, y] = w2s(it.x, it.y), rr = Math.max(3, it.r * S.view.s);
      ctx.strokeStyle = it.mobile ? '#ff4d4d' : '#ff9b3d'; ctx.lineWidth = it.kind === 'cercle' ? 2.2 : 1.6;
      ctx.beginPath(); ctx.arc(x, y, rr, 0, 2 * Math.PI); ctx.stroke();
    }
  }
  // trajet
  const P = PROC.plan, rob = numVal('pRobotR') * S.view.s;
  const mark = (p, col) => { const [x, y] = w2s(p.x, p.y); ctx.fillStyle = col + '55'; ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(x, y, Math.max(5, rob), 0, 2 * Math.PI); ctx.fill(); ctx.stroke(); };
  if (P.result && P.result.ok){
    ctx.strokeStyle = 'rgba(62,230,230,.35)'; ctx.lineWidth = Math.max(2, 2 * rob); ctx.lineJoin = 'round';
    ctx.beginPath(); P.result.path.forEach((p, k) => { const [x, y] = w2s(p.x, p.y); k ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke();
    ctx.strokeStyle = '#3ee6e6'; ctx.lineWidth = 2.5; ctx.stroke();
    ctx.fillStyle = '#3ee6e6'; for (const p of P.result.path){ const [x, y] = w2s(p.x, p.y); ctx.beginPath(); ctx.arc(x, y, 3.5, 0, 2 * Math.PI); ctx.fill(); }
  }
  if (P.start) mark(P.start, '#4cd97b');
  if (P.goal) mark(P.goal, '#ff6b6b');
  ctx.restore();
}
function procHud(wx, wy){
  const R = PROC.res; if (!R) return '';
  const g = R.grid, [i, j] = LC.worldToCell(g, wx, wy); if (!LC.inGrid(g, i, j)) return '   ·   hors carte';
  const k = j * g.W + i, L = R.layers, c = R.cost.cost[k];
  const cs = c === 254 ? 'obstacle' : c === 253 ? 'trop près (inscrit)' : c === 255 ? 'inconnu' : c ? `coût ${c}` : 'libre';
  const ob = R.labels[k] >= 0 ? ` · obstacle n°${R.labels[k]} (${R.obstacles[R.labels[k]].cls})` : '';
  return `   ·   cellule ${g.data[k]} (brute ${R.gridRaw.data[k]}) · impacts ${L.hits[k]}/${L.seen[k]} obs.${ob} · ${cs}`;
}
function procPointerDown(e, mx, my){
  if (!PROC.planMode || e.button !== 0) return false;
  if (!PROC.res) { toast('Lance d’abord le traitement'); return true; }
  const [x, y] = s2w(mx, my), P = PROC.plan;
  if (!P.start || P.goal){ P.start = { x, y }; P.goal = null; P.result = null; }
  else { P.goal = { x, y }; procPlan(); }
  procPlanInfo(); draw(); return true;
}
function procPlan(){
  const P = PROC.plan; if (!P.start || !P.goal || !PROC.res) return;
  const t0 = performance.now();
  P.result = LM.planPath(PROC.res.cost, P.start, P.goal, { allowUnknown: $('pPlanUnk').checked });
  P.result.ms = performance.now() - t0;
}
function procPlanInfo(){
  const P = PROC.plan, el = $('pPlanInfo');
  $('pPlanMode').classList.toggle('on', PROC.planMode);
  if (!PROC.res){ el.textContent = 'Lance d’abord le traitement.'; return; }
  if (!P.start){ el.textContent = PROC.planMode ? 'Clique le départ.' : 'Active « Placer départ / arrivée » puis clique le départ et l’arrivée.'; return; }
  if (!P.goal){ el.textContent = `Départ (${P.start.x.toFixed(2)} ; ${P.start.y.toFixed(2)}) m · clique l’arrivée.`; return; }
  const r = P.result;
  if (!r.ok){ el.innerHTML = `<span style="color:#ff6b6b">Pas de trajet : ${r.reason}.</span>`; return; }
  const direct = Math.hypot(P.goal.x - P.start.x, P.goal.y - P.start.y);
  el.innerHTML = `<b>Trajet : ${r.length.toFixed(2)} m</b> (ligne droite ${direct.toFixed(2)} m), ${r.path.length - 1} tronçon(s) droits, ${r.raw.length} cellules · ${r.expanded} cellules explorées en ${r.ms.toFixed(0)} ms` +
    `<br><span class="mono">${r.path.map(p => `(${p.x.toFixed(2)} ; ${p.y.toFixed(2)})`).join(' → ')}</span>`;
}
$('pRun').onclick = procRun;
['pLayer'].forEach(id => $(id).onchange = draw);
['pShowVec', 'pShowVecRaw', 'pShowCirc', 'pShowPts'].forEach(id => $(id).onchange = draw);
$('pAlpha').oninput = draw;
$('pMinConf').oninput = () => { $('pMinConfV').textContent = $('pMinConf').value; draw(); };
$('pPlanMode').onclick = () => { PROC.planMode = !PROC.planMode; procPlanInfo(); };
$('pPlanClear').onclick = () => { PROC.plan = { start: null, goal: null, result: null }; procPlanInfo(); draw(); };
$('pPlanUnk').onchange = () => { procPlan(); procPlanInfo(); draw(); };
['pRobotR', 'pInfl'].forEach(id => $(id).onchange = () => {
  if (!PROC.res) return;
  PROC.res.cost = LM.costmap(PROC.res.grid, { robotRadius: numVal('pRobotR'), inflation: numVal('pInfl') });
  delete PROC.imgs.cost; procPlan(); procPlanInfo(); draw();
});
const needProc = () => { if (!PROC.res){ toast('Lance d’abord le traitement'); return false; } return true; };
$('pApply').onclick = () => {
  if (!needProc()) return;
  if (!PROC.res.refineLog) return toast('Coche « Recaler toutes les poses » puis relance le traitement');
  let n = 0;
  PROC.scans.forEach((ps, k) => {
    const s = S.scans.find(x => x.id === ps.id); if (!s) return;
    const p = PROC.res.poses[k];
    if (Math.hypot(p.x - s.pose.x, p.y - s.pose.y) < 1e-6 && Math.abs(p.th - s.pose.th) < 1e-6) return;
    pushUndo(s); s.pose = { x: p.x, y: p.y, th: p.th }; n++;
  });
  if (typeof renderList === 'function') renderList();
  toast(`${n} pose(s) mise(s) à jour`); draw();
};
$('pToGrid').onclick = () => { if (!needProc()) return; gridSet(LM.cloneGrid(PROC.res.grid), 'Grid map traitée (onglet Traitement)'); toast('Carte envoyée vers l’onglet Grid map'); };
$('pToVec').onclick = () => {
  if (!needProc()) return;
  vecPush(); VEC.map = JSON.parse(JSON.stringify(PROC.res.vmap)); VEC.last = null; VEC.sel = -1; vecRefresh();
  toast('Carte envoyée vers l’onglet Vectorielle');
};
$('pExpPgm').onclick = () => {
  if (!needProc()) return;
  const { pgm, yaml } = LC.gridToPGM(PROC.res.grid, 'carte_traitee');
  download('carte_traitee.pgm', pgm, 'image/x-portable-graymap'); setTimeout(() => download('carte_traitee.yaml', yaml, 'text/yaml'), 300);
};
$('pExpPng').onclick = () => {
  if (!needProc()) return;
  const src = procLayerImage($('pLayer').value === 'none' ? 'clean' : $('pLayer').value), k = Math.max(1, Math.min(8, Math.floor(2400 / Math.max(src.width, src.height))));
  const c = document.createElement('canvas'); c.width = src.width * k; c.height = src.height * k;
  const x = c.getContext('2d'); x.imageSmoothingEnabled = false; x.drawImage(src, 0, 0, c.width, c.height);
  // superposition de la carte vectorielle et des obstacles circulaires
  const g = PROC.res.grid, tw = (X, Y) => [(X - g.x0) / g.res * k, (g.y0 + g.H * g.res - Y) / g.res * k];
  if ($('pLayer').value === 'plan' && PROC.res.plan){
    const poly = (pts, col) => { x.strokeStyle = col; x.lineWidth = Math.max(2.5, k); x.beginPath(); pts.forEach((p, n) => { const [a, b2] = tw(p[0], p[1]); n ? x.lineTo(a, b2) : x.moveTo(a, b2); }); x.closePath(); x.stroke(); };
    poly(PROC.res.plan.outer, '#1e6fff'); PROC.res.plan.islands.forEach(I => poly(I, '#ff8a00'));
    for (const [a, b] of PROC.res.plan.innerWalls || []){ const [p1, p2] = tw(a[0], a[1]), [q1, q2] = tw(b[0], b[1]); x.strokeStyle = '#ff8a00'; x.beginPath(); x.moveTo(p1, p2); x.lineTo(q1, q2); x.stroke(); }
    download('plan_salle.png', c.toDataURL('image/png')); return;
  }
  if ($('pShowVec').checked){ x.strokeStyle = '#4da3ff'; x.lineWidth = Math.max(2, k * 0.8); x.lineCap = 'round';
    for (const f of PROC.res.vmap.features){ const [a, b] = tw(f.x1, f.y1), [c2, d] = tw(f.x2, f.y2); x.beginPath(); x.moveTo(a, b); x.lineTo(c2, d); x.stroke(); } }
  if ($('pShowCirc').checked){ for (const it of PROC.res.omap.items){ if (it.conf < numVal('pMinConf') && !it.mobile) continue;
    const [a, b] = tw(it.x, it.y); x.strokeStyle = it.mobile ? '#ff4d4d' : '#ff9b3d'; x.lineWidth = Math.max(1.5, k * 0.4); x.beginPath(); x.arc(a, b, Math.max(3, it.r / g.res * k), 0, 2 * Math.PI); x.stroke(); } }
  download(`carte_${$('pLayer').value}.png`, c.toDataURL('image/png'));
};
$('pExpVec').onclick = () => { if (!needProc()) return; download('carte_vectorielle_reguliere.txt', LC.vectorMapToText(PROC.res.vmap), 'text/plain'); };
$('pExpObs').onclick = () => {
  if (!needProc()) return;
  let s = '# Obstacles circulaires suivis (features cercle)\n' + LM.obstaclesToCSV(PROC.res.omap);
  s += '\n# Obstacles de la grid map (composantes connexes)\nid,classe,cellules,surface_m2,cx_m,cy_m,longueur_m,largeur_m,angle_deg,impacts_max\n';
  for (const o of PROC.res.obstacles) s += [o.id, o.cls, o.cells, o.area.toFixed(4), o.cx.toFixed(3), o.cy.toFixed(3), o.len.toFixed(3), o.wid.toFixed(3), o.angle.toFixed(1), o.support].join(',') + '\n';
  download('obstacles.csv', s, 'text/csv');
};
$('pExpCost').onclick = () => { if (!needProc()) return; download('carte_de_cout.pgm', LM.costmapToPGM(PROC.res.cost), 'image/x-portable-graymap'); };
$('pExpRep').onclick = () => { if (!needProc()) return; download('rapport_traitement.md', LM.reportToMarkdown(PROC.res), 'text/markdown'); };
procPlanInfo();

// ---------------------------------------------------------------- TRAITEMENT : gomme et filtres
PROC.eraser = false; PROC.erasing = false; PROC.cursor = null; PROC.undo = []; PROC.lastErase = null;
function procSnapshot(){
  const R = PROC.res;
  PROC.undo.push({ grid: R.grid.data.slice(), vmap: JSON.stringify(R.vmap), omap: JSON.stringify(R.omap), manual: { ...R.manual } });
  if (PROC.undo.length > 40) PROC.undo.shift();
}
function procRefresh(){
  LM.refreshDerived(PROC.res, { cost: { robotRadius: numVal('pRobotR'), inflation: numVal('pInfl') } });
  PROC.imgs = {}; procPlan(); procPlanInfo(); procInfo(); procEInfo(); draw();
}
function procUndo(){
  const u = PROC.undo.pop(); if (!u || !PROC.res) return toast('Rien à annuler');
  const R = PROC.res; R.grid.data.set(u.grid); R.vmap = JSON.parse(u.vmap); R.omap = JSON.parse(u.omap); R.manual = u.manual;
  procRefresh(); toast('Retouche annulée');
}
function procEInfo(){
  const R = PROC.res, m = R && R.manual;
  $('pEraser').classList.toggle('on', PROC.eraser);
  if (!m){ $('pEInfo').textContent = 'Lance d’abord le traitement.'; return; }
  $('pEInfo').innerHTML = (m.cells || m.segCut || m.segRemoved || m.circles)
    ? `Retouches : <b>${m.cells}</b> cellules effacées, <b>${m.segCut}</b> segments coupés, <b>${m.segRemoved}</b> supprimés, <b>${m.circles}</b> cercles supprimés · ${PROC.undo.length} étape(s) annulable(s)`
    : 'Aucune retouche.';
}
function eraseOpt(){ return { grid: $('pEGrid').checked, segments: $('pESeg').checked, circles: $('pECirc').checked, fill: $('pFill').value }; }
function eraseAt(x, y){
  const r = numVal('pBrush'), o = eraseOpt(), R = PROC.res;
  // on interpole depuis le point précédent pour ne pas laisser de trous quand la souris va vite
  const prev = PROC.lastErase, pts = [];
  if (prev){ const d = Math.hypot(x - prev.x, y - prev.y), n = Math.ceil(d / (r * 0.5)); for (let k = 1; k <= n; k++) pts.push([prev.x + (x - prev.x) * k / n, prev.y + (y - prev.y) * k / n]); }
  else pts.push([x, y]);
  for (const [a, b] of pts){
    const st = LM.eraseDisc(R, a, b, r, o);
    R.manual.cells += st.cells; R.manual.circles += st.circles; R.manual.segRemoved += st.segRemoved;
    for (const id of st.cutIds) if (!PROC.strokeIds.has(id)){ PROC.strokeIds.add(id); R.manual.segCut++; }   // un segment compte une fois par coup de gomme
  }
  PROC.lastErase = { x, y };
  delete PROC.imgs.clean; delete PROC.imgs.diff; delete PROC.imgs.obst; delete PROC.imgs.cost;   // couches à redessiner
}
// remplace le gestionnaire de clic : gomme prioritaire, puis trajet
const procPointerDownPlan = procPointerDown;
procPointerDown = function(e, mx, my){
  if (PROC.eraser && e.button === 0){
    if (!PROC.res){ toast('Lance d’abord le traitement'); return true; }
    procSnapshot(); PROC.erasing = true; PROC.lastErase = null; PROC.strokeIds = new Set();
    const [x, y] = s2w(mx, my); eraseAt(x, y); draw(); return true;
  }
  return procPointerDownPlan(e, mx, my);
};
function procPointerMove(mx, my){
  if (!PROC.eraser) return false;
  PROC.cursor = [mx, my];
  if (PROC.erasing){ const [x, y] = s2w(mx, my); eraseAt(x, y); }
  draw();
  return PROC.erasing;
}
function procPointerUp(){
  if (!PROC.erasing) return;
  PROC.erasing = false; PROC.lastErase = null; procRefresh();
}
// curseur de la gomme (dessiné par-dessus le reste)
const drawProcOverBase = drawProcOver;
drawProcOver = function(){
  drawProcOverBase();
  if (!PROC.eraser || !PROC.cursor) return;
  const [mx, my] = PROC.cursor, r = numVal('pBrush') * S.view.s;
  ctx.save(); ctx.strokeStyle = '#ff6bd5'; ctx.fillStyle = 'rgba(255,107,213,.12)'; ctx.lineWidth = 1.5; ctx.setLineDash([4, 3]);
  ctx.beginPath(); ctx.arc(mx, my, Math.max(2, r), 0, 2 * Math.PI); ctx.fill(); ctx.stroke(); ctx.restore();
};
$('pEraser').onclick = () => {
  PROC.eraser = !PROC.eraser;
  if (PROC.eraser && PROC.planMode){ PROC.planMode = false; procPlanInfo(); }
  if (!PROC.eraser) PROC.cursor = null;
  procEInfo(); draw();
};
$('pPlanMode').addEventListener('click', () => { if (PROC.planMode && PROC.eraser){ PROC.eraser = false; PROC.cursor = null; procEInfo(); draw(); } });
$('pEUndo').onclick = procUndo;
$('pBrush').oninput = draw;
cv.addEventListener('pointerleave', () => { if (PROC.cursor){ PROC.cursor = null; if (TAB === 'proc') draw(); } });
function filterOpt(){ return { minSegLen: numVal('pFSeg') || 0, minCircleD: numVal('pFCirc') || 0, minBlob: numVal('pFBlob') || 0, nearPoses: numVal('pFNear') || 0, fill: $('pFill').value }; }
$('pFApply').onclick = () => {
  if (!needProc()) return;
  const F = filterOpt(), R = PROC.res;
  if (!(F.minSegLen || F.minCircleD || F.minBlob || F.nearPoses)) return toast('Tous les filtres sont à 0');
  procSnapshot();
  const a = LM.filterSmall(R, F), b = F.nearPoses > 0 ? LM.filterNearPoses(R, R.poses, F.nearPoses, { fill: F.fill }) : { cells: 0, segCut: 0, segRemoved: 0, circles: 0 };
  R.manual.cells += a.cells + b.cells; R.manual.segRemoved += a.segments + b.segRemoved; R.manual.segCut += b.segCut; R.manual.circles += a.circles + b.circles;
  procRefresh();
  toast(`Supprimés : ${a.segments + b.segRemoved} segments (${b.segCut} coupés), ${a.circles + b.circles} cercles, ${a.blobs} taches, ${a.cells + b.cells} cellules`);
};
procEInfo();

// ---------------------------------------------------------------- ROBOT EN DIRECT (localisation sur la carte du Jour 1, vue GPS)
const R2D = 180 / Math.PI;
const LIVE = { map: null, mapImg: null, mapKey: '', tracker: null, running: false, feed: 'robot', busy: false, pending: null,
               view: 'head', zoom: 60, free: null, drag: null, hint: false, hintDrag: null, motion: null, cur: null, trail: [],
               replay: { i: 0, list: [] }, sim: null, simIdx: 0, timer: null, looCache: new Map(), stats: { n: 0, global: 0, reloc: 0 } };
const lvSetInfo = h => { $('lvMapInfo').innerHTML = h; };
function lvRaw(s){                         // scan brut avec la convention d'angle de l'affichage (miroir, décalage)
  const o = S.opt, sg = o.mirror ? -1 : 1, a = s.a.map(v => ((sg * v + (o.aOff || 0)) % 360 + 360) % 360);
  return { name: s.name, a, d: s.d, i: s.i, pose: s.pose ? { ...s.pose } : null, id: s.id };
}
// --- carte de référence
function livePrepare(silent){
  const src = $('lvSrc').value, t0 = performance.now();
  let grid = null, segs = null, label = '';
  try {
    if (src === 'scans'){
      const vis = S.scans.filter(s => s.visible); if (!vis.length) throw new Error('aucun scan visible');
      const R = LL.buildReferences(vis.map(lvRaw), -1); grid = R.grid; segs = R.vmap.features; label = `${vis.length} scans visibles`;
    } else if (src === 'plan'){
      if (!PROC.res || !PROC.res.plan) throw new Error('construis d’abord le plan (onglet Traitement)');
      grid = PROC.res.plan.grid; segs = PROC.res.plan.segments; label = 'plan de la salle (sans obstacles)';
    } else if (src === 'proc'){
      if (!PROC.res) throw new Error('lance d’abord le traitement (onglet Traitement)');
      grid = PROC.res.grid; segs = PROC.res.vmap.features; label = 'grid map traitée + carte vectorielle régularisée';
    } else if (src === 'file'){
      if (!LIVE.fileGrid) throw new Error('charge d’abord la grid map (.json)');
      grid = LIVE.fileGrid; segs = LIVE.fileVec ? LIVE.fileVec.features : null; label = 'fichiers' + (segs ? '' : ' (sans carte vectorielle)');
    } else {
      LIVE.sim = LL.demoScenario(3); grid = LIVE.sim.refs.grid; segs = LIVE.sim.refs.vmap.features; label = 'salle simulée (9 scans)';
    }
    LIVE.map = LL.prepareMap(grid, segs); LIVE.map.label = label; LIVE.mapImg = null; LIVE.looCache.clear();
    liveCorrReset(true);
    liveReset(false);
    const st = LC.gridStats(grid);
    lvSetInfo(`<b>Carte prête</b> : ${label}<br>${grid.W} × ${grid.H} cellules de ${(grid.res * 100).toFixed(0)} cm · ${(st.free * grid.res * grid.res).toFixed(0)} m² libres · ` +
      `${segs ? segs.length + ' segments' : 'pas de segments'} · ${LIVE.map.cands.length} positions candidates · ${(performance.now() - t0).toFixed(0)} ms`);
    if (!LIVE.pose) liveCenterOnMap();
    draw(); return true;
  } catch (e){ if (!silent) toast('Carte : ' + e.message); lvSetInfo('Aucune carte : ' + e.message); return false; }
}
function liveReset(clearTrail = true){
  LIVE.tracker = LIVE.map ? LL.createTracker(LIVE.map) : null; LIVE.cur = null; LIVE.motion = null;
  if (clearTrail){ LIVE.trail = []; LIVE.stats = { n: 0, global: 0, reloc: 0 }; }
  liveState();
}
function liveCenterOnMap(){
  const g = LIVE.map && LIVE.map.grid; if (!g) return;
  LIVE.free = { x: g.x0 + g.W * g.res / 2, y: g.y0 + g.H * g.res / 2, phi: 0 };
  LIVE.zoom = Math.min(W / (g.W * g.res), H / (g.H * g.res)) * 0.9;
  if (!LIVE.cur) LIVE.view = 'free', liveViewButtons();
}
// --- traitement d'un scan
function liveFeedScan(b, truth){
  if (LIVE.rec && LIVE.feed !== 'rec') LIVE.rec.scans.push({ t: Date.now() - LIVE.rec.t0, a: Array.from(b.a), d: Array.from(b.d), i: b.i ? Array.from(b.i) : null, truth: truth || null });
  if (!LIVE.map) return;
  if (LIVE.busy){ LIVE.pending = [b, truth]; return; }      // on garde seulement le plus récent
  LIVE.busy = true;
  try {
    const raw = b.pose !== undefined && b.name ? b : lvRaw(b);
    // relecture honnête : carte construite sans ce scan
    if (LIVE.feed === 'replay' && $('lvLOO').checked && $('lvSrc').value === 'scans' && raw.id !== undefined){
      let M = LIVE.looCache.get(raw.id);
      if (!M){
        const vis = S.scans.filter(s => s.visible && s.id !== raw.id).map(lvRaw);
        const R = LL.buildReferences(vis, -1); M = LL.prepareMap(R.grid, R.vmap.features); LIVE.looCache.set(raw.id, M);
      }
      LIVE.tracker.M = M;
    } else if (LIVE.tracker) LIVE.tracker.M = LIVE.map;
    const local = LL.localPoints(raw), pts = LL.uniformSubsample(local, { max: 350 });
    const motion = $('lvOdom').checked ? LIVE.motion : null; LIVE.motion = null;
    const r = LL.trackerUpdate(LIVE.tracker, pts, motion, local);
    LIVE.stats.n++; if (r.mode === 'global') LIVE.stats.global++; if (r.mode === 'relocalisation') LIVE.stats.reloc++;
    const T = LIVE.tracker;
    LIVE.cur = { local, res: r, truth: truth || null, t: Date.now() };
    if ($('lvLearn').checked && LIVE.corr && T.pose && (r.status === 'localisé' || r.status === 'relocalisé') && !(T.hyps && T.hyps.length > 1)){
      LM.correctorAdd(LIVE.corr, T.pose, local);
      if (LIVE.corr.n % 3 === 1 || LIVE.corr.n < 4) liveCorrUpdate();
    }
    if (T.pose){
      LIVE.trail.push({ x: T.pose.x, y: T.pose.y, th: T.pose.th, inliers: r.inliers, mode: r.mode, status: r.status, ms: r.ms, t: Date.now(),
                        err: truth ? LL.poseError(T.pose, truth) : null });
      if (LIVE.trail.length > 5000) LIVE.trail.shift();
      if ($('lvSendPose').checked && SER.wifi && (r.status === 'localisé' || r.status === 'relocalisé')){
        const mm = v => Math.max(-32768, Math.min(32767, Math.round(v * 1000)));
        robotCmd(`/pose?x=${mm(T.pose.x)}&y=${mm(T.pose.y)}&z=${Math.round(((T.pose.th % 360) + 360) % 360 * 100) % 36000}`);
      }
    }
    liveState(); draw();
  } catch (e){ console.error(e); toast('Localisation : ' + e.message); }
  finally {
    LIVE.busy = false;
    if (LIVE.onceStop){ LIVE.onceStop = false; LIVE.running = false; }
    if (LIVE.pending){ const [pb, pt] = LIVE.pending; LIVE.pending = null; setTimeout(() => liveFeedScan(pb, pt), 0); }
  }
}
// déplacement commandé (pilotage WiFi) -> prédiction pour le prochain scan, dans le repère du robot
function liveOnDrive(mv, ms){
  if (LIVE.rec) LIVE.rec.cmds.push({ t: Date.now() - LIVE.rec.t0, mv, ms });
  liveOnDriveMotion(mv, ms);
}
function liveOnDriveMotion(mv, ms){
  if (!LIVE.running) return;
  const v = numVal('lvV') * ms / 1000, w = numVal('lvW') * ms / 1000;
  const d = { forward: [v, 0, 0], backward: [-v, 0, 0], left: [0, v, 0], right: [0, -v, 0], rotateleft: [0, 0, w], rotateright: [0, 0, -w] }[mv];
  if (!d) return;
  const m = { x: d[0], y: d[1], th: d[2] };
  LIVE.motion = LIVE.motion ? LL.compose(LIVE.motion, m) : m;
}
function liveState(){
  const T = LIVE.tracker, r = LIVE.cur && LIVE.cur.res, el = $('lvState');
  if (!LIVE.map){ el.textContent = 'Prépare d’abord la carte.'; return; }
  if (!T || !r){ el.innerHTML = 'Pas encore de scan. <b>Démarrer</b> pour commencer.'; return; }
  const col = { 'localisé': '#4cd97b', 'relocalisé': '#4cd97b', 'ambigu': '#ffb347', 'incertain': '#ffb347', 'perdu': '#ff6b6b', 'non localisé': '#ff6b6b' }[r.status] || '#ccc';
  const p = T.pose, tr = LIVE.cur.truth;
  let h = `<span style="color:${col};font-weight:700;font-size:15px">● ${r.status}</span> · ${r.mode} · ${r.ms.toFixed(0)} ms`;
  if (p) h += `<br>x <b>${p.x.toFixed(2)}</b> m · y <b>${p.y.toFixed(2)}</b> m · cap <b>${p.th.toFixed(1)}°</b>`;
  h += `<br>points sur la carte : <b>${(r.inliers * 100).toFixed(0)} %</b>` + (r.nScans > 1 ? ` · carte locale de ${r.nScans} scans` : '');
  if (r.mode === 'relocalisation à confirmer') h += `<br><span style="color:#ffb347">Une autre position est possible : le robot ne saute pas, il garde les deux et laisse son déplacement trancher.</span>`;
  if (r.status === 'ambigu' && T.hyps) h += `<br><span style="color:#ffb347">${T.hyps.length} positions possibles : déplacer le robot pour lever le doute, ou l’indiquer à la main.</span>`;
  if (tr && p){ const e = LL.poseError(p, tr); h += `<br>écart à la vraie pose : <b style="color:${e.pos <= 0.05 && e.ang <= 2 ? '#4cd97b' : '#ff6b6b'}">${(e.pos * 100).toFixed(1)} cm, ${e.ang.toFixed(2)}°</b>`; }
  const okN = LIVE.trail.filter(t => t.err).length ? LIVE.trail.filter(t => t.err && t.err.pos <= 0.05 && t.err.ang <= 2).length : null;
  h += `<br><span class="help">${LIVE.stats.n} scans · ${LIVE.stats.global} recherches globales · ${LIVE.stats.reloc} relocalisations` +
       (okN !== null ? ` · justes ${okN}/${LIVE.trail.filter(t => t.err).length}` : '') + '</span>';
  el.innerHTML = h;
}
// --- sources de scans
function liveStart(){
  const feed = LIVE.feed = $('lvFeed').value;
  // le robot simulé roule dans la salle simulée : c'est sa carte
  if (feed === 'sim' && ($('lvSrc').value !== 'sim' || !LIVE.sim || !LIVE.map)){ $('lvSrc').value = 'sim'; $('lvSrc').onchange(); if (!livePrepare()) return; toast('Carte : salle simulée'); }
  if (!LIVE.map && !livePrepare()) return;
  liveStop(true); liveReset(true); LIVE.running = true; liveViewButtons();
  if (feed === 'robot'){
    if (!isConnected()){ LIVE.running = false; return toast('Connecte d’abord le robot (onglet Scans, USB ou WiFi)'); }
    if (!SER.live){ $('live').checked = true; $('live').onchange({ target: $('live') }); }
    toast('Localisation en direct : en attente des scans du robot');
  } else if (feed === 'replay'){
    LIVE.replay = { i: 0, list: S.scans.filter(s => s.visible) };
    if (!LIVE.replay.list.length){ LIVE.running = false; return toast('Aucun scan visible'); }
    liveTick();
  } else if (feed === 'rec'){
    if (!LIVE.recPlay){ LIVE.running = false; return toast('Charge d’abord un enregistrement (.json)'); }
    LIVE.recPlay.i = 0; liveTick();
  } else { LIVE.simIdx = 0; LIVE.simPrev = null; liveTick(); }
  if (LIVE.view === 'free') LIVE.zoom = Math.max(LIVE.zoom, 70);          // vue GPS : assez près du robot
  liveSetView(LIVE.view === 'free' ? 'head' : LIVE.view);
}
function liveStop(quiet){
  LIVE.running = false; clearTimeout(LIVE.timer);
  if (!quiet) toast('Arrêté');
}
function liveStep(){                       // un scan de la source relecture / simulation
  if (LIVE.feed === 'replay'){
    const L = LIVE.replay.list; if (!L.length) return false;
    const s = L[LIVE.replay.i % L.length]; LIVE.replay.i++;
    liveFeedScan(lvRaw(s), { ...s.pose });
    return true;
  }
  if (LIVE.feed === 'rec'){
    const R = LIVE.recPlay; if (!R || R.i >= R.data.scans.length) return false;
    const sc = R.data.scans[R.i], prevT = R.i ? R.data.scans[R.i - 1].t : -Infinity; R.i++;
    // commandes envoyées entre le scan précédent et celui-ci -> prédiction (comme en direct)
    for (const c of R.data.cmds) if (c.t > prevT && c.t <= sc.t) { const run = LIVE.running; LIVE.running = true; liveOnDriveMotion(c.mv, c.ms); LIVE.running = run; }
    liveFeedScan({ a: sc.a, d: sc.d, i: sc.i || sc.a.map(() => 200) }, sc.truth || undefined);
    $('lvRecInfo').textContent = `scan ${R.i}/${R.data.scans.length}`;
    return true;
  }
  if (LIVE.feed === 'sim'){
    const D = LIVE.sim; if (!D) return false;
    const truth = D.path[LIVE.simIdx % D.path.length]; LIVE.simIdx++;
    if (LIVE.simPrev && !LIVE.kidnapped){
      const m = LL.relPose(LIVE.simPrev, truth), g = () => (D.rng() - 0.5) * 0.2;
      LIVE.motion = { x: m.x * (1 + g()) + 0.01, y: m.y * (1 + g()), th: m.th * (1 + g()) + 0.5 };   // odométrie bruitée
    }
    LIVE.kidnapped = false; LIVE.simPrev = truth;
    liveFeedScan(Object.assign(LL.simulateScan($('lvSimChange').checked ? liveChangedScene(D) : D.scene.walls, truth, { tours: 1 }, D.rng), { name: 'sim', pose: null }), truth);
    return true;
  }
  return false;
}
function liveTick(){
  if (!LIVE.running) return;
  liveStep();
  LIVE.timer = setTimeout(liveTick, Math.max(50, numVal('lvPeriod')));
}
// --- vue « GPS » : transformation monde -> écran (rotation, zoom, centrage)
function liveXf(){
  const T = LIVE.tracker, p = T && T.pose;
  if (LIVE.view !== 'free' && p){
    return { x: p.x, y: p.y, phi: LIVE.view === 'head' ? (90 - p.th) * D2R : 0, cx: W / 2, cy: LIVE.view === 'head' ? H * 0.62 : H / 2, k: LIVE.zoom };
  }
  const f = LIVE.free || { x: 0, y: 0, phi: 0 };
  return { x: f.x, y: f.y, phi: f.phi, cx: W / 2, cy: H / 2, k: LIVE.zoom };
}
const lvW2S = (X, x, y) => { const dx = x - X.x, dy = y - X.y, c = Math.cos(X.phi), s = Math.sin(X.phi);
  return [X.cx + (c * dx - s * dy) * X.k, X.cy - (s * dx + c * dy) * X.k]; };
const lvS2W = (X, sx, sy) => { const u = (sx - X.cx) / X.k, v = -(sy - X.cy) / X.k, c = Math.cos(-X.phi), s = Math.sin(-X.phi);
  return [X.x + c * u - s * v, X.y + s * u + c * v]; };
function liveMapImage(){
  const g = LIVE.showProposal && LIVE.proposal ? LIVE.proposal : LIVE.map.grid;
  if (LIVE.mapImg && LIVE.mapImg.g === g) return LIVE.mapImg.c;
  const c = document.createElement('canvas'); c.width = g.W; c.height = g.H;
  const x = c.getContext('2d'), im = x.createImageData(g.W, g.H);
  for (let j = 0; j < g.H; j++) for (let i = 0; i < g.W; i++){
    const v = g.data[j * g.W + i], o = ((g.H - 1 - j) * g.W + i) * 4;
    const [r, gg, b, a] = v >= LC.OCC_T ? [214, 222, 232, 255] : v <= LC.FREE_T ? [36, 44, 56, 255] : [0, 0, 0, 0];
    im.data[o] = r; im.data[o + 1] = gg; im.data[o + 2] = b; im.data[o + 3] = a;
  }
  x.putImageData(im, 0, 0); LIVE.mapImg = { g, c }; return c;
}
const LV_COL = { 'localisé': '#4cd97b', 'relocalisé': '#4cd97b', 'ambigu': '#ffb347', 'incertain': '#ffb347', 'perdu': '#ff6b6b', 'non localisé': '#ff6b6b' };
function drawLiveView(){
  ctx.fillStyle = '#0b0e12'; ctx.fillRect(0, 0, W, H);
  if (!LIVE.map){
    ctx.fillStyle = '#9aa3ad'; ctx.font = '15px system-ui'; ctx.textAlign = 'center';
    ctx.fillText('Robot en direct : prépare la carte (étape 1), puis Démarrer.', W / 2, H / 2); ctx.textAlign = 'left'; return;
  }
  const X = liveXf(), g = LIVE.map.grid, P = (x, y) => lvW2S(X, x, y);
  // carte (image affine : tourne avec la vue)
  const ytop = g.y0 + g.H * g.res, o = P(g.x0, ytop), di = P(g.x0 + g.res, ytop), dj = P(g.x0, ytop - g.res);
  ctx.save(); ctx.imageSmoothingEnabled = false;
  ctx.setTransform(dpr * (di[0] - o[0]), dpr * (di[1] - o[1]), dpr * (dj[0] - o[0]), dpr * (dj[1] - o[1]), dpr * o[0], dpr * o[1]);
  ctx.drawImage(liveMapImage(), 0, 0);
  if ($('lvShowCorr').checked && LIVE.corrDiff && !LIVE.showProposal){ const ci = liveCorrImage(); if (ci) ctx.drawImage(ci, 0, 0); }
  ctx.restore();
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  // grille métrique légère (repère : tous les mètres)
  ctx.save(); ctx.lineCap = 'round';
  if ($('lvShowVec').checked && LIVE.map.segs.length){
    ctx.strokeStyle = 'rgba(77,163,255,.85)'; ctx.lineWidth = 2;
    ctx.beginPath(); for (const f of LIVE.map.segs){ const a = P(f.x1, f.y1), b = P(f.x2, f.y2); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); } ctx.stroke();
  }
  // trajectoire
  if ($('lvShowTrail').checked && LIVE.trail.length > 1){
    ctx.lineWidth = 3; ctx.lineJoin = 'round';
    for (let k = 1; k < LIVE.trail.length; k++){
      const a = LIVE.trail[k - 1], b = LIVE.trail[k];
      if (Math.hypot(a.x - b.x, a.y - b.y) > 1.5) continue;            // saut (relocalisation) : pas de trait
      const u = k / LIVE.trail.length;
      ctx.strokeStyle = `rgba(62,230,230,${0.15 + 0.7 * u})`;
      const pa = P(a.x, a.y), pb = P(b.x, b.y); ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke();
    }
  }
  const T = LIVE.tracker, cur = LIVE.cur, r = cur && cur.res;
  const robotR = Math.max(9, (numVal('pRobotR') || 0.15) * X.k);
  const drawRobot = (p, fill, stroke, alpha, dashed) => {
    const [x, y] = P(p.x, p.y), a = p.th * D2R + X.phi;               // cap à l'écran
    const ux = Math.cos(a), uy = -Math.sin(a);
    ctx.save(); ctx.globalAlpha = alpha;
    if (dashed) ctx.setLineDash([5, 4]);
    ctx.fillStyle = fill; ctx.strokeStyle = stroke; ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.arc(x, y, robotR, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.setLineDash([]);
    const L = robotR * 1.9;
    ctx.beginPath(); ctx.moveTo(x + ux * L, y + uy * L); ctx.lineTo(x - uy * robotR * 0.7 + ux * robotR * 0.2, y + ux * robotR * 0.7 + uy * robotR * 0.2);
    ctx.lineTo(x + uy * robotR * 0.7 + ux * robotR * 0.2, y - ux * robotR * 0.7 + uy * robotR * 0.2); ctx.closePath();
    ctx.fillStyle = stroke; ctx.fill(); ctx.restore();
  };
  // hypothèses (ambiguïté)
  if ($('lvShowHyps').checked && T && T.hyps && T.hyps.length > 1)
    for (const h of T.hyps.slice(1)) drawRobot(h, 'rgba(255,179,71,.18)', '#ffb347', 0.8, true);
  // scan actuel, posé à la pose trouvée : vert s'il colle à la carte, rouge sinon (obstacle nouveau)
  if ($('lvShowScan').checked && cur && T && T.pose){
    const p = T.pose, c = Math.cos(p.th * D2R), s = Math.sin(p.th * D2R), F = LIVE.tracker.M.field;
    for (const q of cur.local){
      const wx = p.x + c * q.x - s * q.y, wy = p.y + s * q.x + c * q.y, d = LL.fieldSample(F, wx, wy)[0];
      const [sx, sy] = P(wx, wy); ctx.fillStyle = d <= 0.1 ? '#4cd97b' : '#ff5c5c';
      ctx.fillRect(sx - 1.6, sy - 1.6, 3.2, 3.2);
    }
  }
  if (cur && cur.truth) drawRobot(cur.truth, 'rgba(255,255,255,0)', 'rgba(255,255,255,.75)', 0.9, true);
  if (T && T.pose){
    const col = LV_COL[T.status] || '#4da3ff';
    drawRobot(T.pose, col + '44', col, 1, false);
  }
  // main : indication de la pose
  if (LIVE.hintDrag){
    const { a, b } = LIVE.hintDrag, pa = P(a[0], a[1]), pb = P(b[0], b[1]);
    ctx.strokeStyle = '#ff6bd5'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(pa[0], pa[1]); ctx.lineTo(pb[0], pb[1]); ctx.stroke();
    ctx.beginPath(); ctx.arc(pa[0], pa[1], robotR, 0, 2 * Math.PI); ctx.stroke();
  }
  ctx.restore();
  // boussole : direction de l'axe +y de la carte (« nord ») et +x
  const cx0 = W - 46, cy0 = 46;
  ctx.save(); ctx.translate(cx0, cy0);
  ctx.fillStyle = 'rgba(15,18,22,.85)'; ctx.strokeStyle = '#2a323c'; ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(0, 0, 32, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
  const ny = [-Math.sin(X.phi), -Math.cos(X.phi)], ex = [Math.cos(X.phi), -Math.sin(X.phi)];
  ctx.strokeStyle = '#ff5c5c'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(ny[0] * 24, ny[1] * 24); ctx.stroke();
  ctx.strokeStyle = '#9aa3ad'; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(ex[0] * 18, ex[1] * 18); ctx.stroke();
  ctx.fillStyle = '#ff5c5c'; ctx.font = '700 11px system-ui'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  ctx.fillText('y', ny[0] * 30 * 0.92 + 0, ny[1] * 30 * 0.92); ctx.fillStyle = '#9aa3ad'; ctx.fillText('x', ex[0] * 26, ex[1] * 26);
  ctx.restore();
  // échelle
  const m = [0.5, 1, 2, 5, 10].find(v => v * X.k >= 60) || 10;
  ctx.save(); ctx.strokeStyle = '#d6dee8'; ctx.fillStyle = '#d6dee8'; ctx.lineWidth = 2; ctx.font = '12px system-ui';
  ctx.beginPath(); ctx.moveTo(16, H - 22); ctx.lineTo(16 + m * X.k, H - 22); ctx.moveTo(16, H - 27); ctx.lineTo(16, H - 17); ctx.moveTo(16 + m * X.k, H - 27); ctx.lineTo(16 + m * X.k, H - 17); ctx.stroke();
  ctx.fillText(m + ' m', 20 + m * X.k, H - 18); ctx.restore();
  // pas encore localisé : le dernier scan reçu est montré tel quel autour du centre de l'écran (on voit que ça scanne)
  if ((!T || !T.pose) && cur && $('lvShowScan').checked){
    ctx.save(); ctx.fillStyle = '#e3e7ec';
    for (const q of cur.local) ctx.fillRect(W / 2 + q.x * X.k - 1.4, H / 2 - q.y * X.k - 1.4, 2.8, 2.8);
    ctx.fillStyle = '#9aa3ad'; ctx.font = '12px system-ui'; ctx.textAlign = 'center';
    ctx.fillText('scan brut (robot pas encore localisé)', W / 2, H / 2 + 18); ctx.textAlign = 'left'; ctx.restore();
  }
  // bandeau d'état
  if (r || LIVE.running){
    const st = T ? T.status : '…', col = LV_COL[st] || '#9aa3ad';
    const line1 = r ? `${st}` : 'en attente du premier scan…';
    let line2 = r ? `${(r.inliers * 100).toFixed(0)} % du scan sur la carte · ${r.mode} · ${r.ms.toFixed(0)} ms` : '';
    if (LIVE.feed === 'robot' && LIVE.running){
      const age = SER.lastScanAt ? (performance.now() - SER.lastScanAt) / 1000 : Infinity;
      line2 += (line2 ? ' · ' : '') + (isFinite(age) ? `dernier scan il y a ${age.toFixed(1)} s` : 'aucun scan reçu');
      if (age > 2){ ctx.save(); ctx.fillStyle = '#ff6b6b'; ctx.font = '700 13px system-ui';
        ctx.fillText(isConnected() ? (SER.live ? 'le robot n’envoie plus de scans : batterie, WiFi ou lidar ?' : 'aperçu direct arrêté (case « Aperçu direct » décochée)') : 'robot déconnecté', 26, 84); ctx.restore(); }
    }
    ctx.save(); ctx.fillStyle = 'rgba(15,18,22,.88)'; ctx.strokeStyle = col; ctx.lineWidth = 2;
    const w = 330, hh = r ? 52 : 34; ctx.beginPath(); ctx.roundRect ? ctx.roundRect(14, 14, w, hh, 8) : ctx.rect(14, 14, w, hh); ctx.fill(); ctx.stroke();
    ctx.fillStyle = col; ctx.font = '700 15px system-ui'; ctx.fillText('● ' + line1, 26, 36);
    ctx.fillStyle = '#c7cfd8'; ctx.font = '12px system-ui'; ctx.fillText(line2, 26, 55); ctx.restore();
  }
}
// --- interactions
function liveSetView(v){
  if (v === 'free'){ const X = liveXf(); LIVE.free = { x: X.x, y: X.y, phi: X.phi }; }
  LIVE.view = v; liveViewButtons(); draw();
}
function liveViewButtons(){ $('lvViewHead').classList.toggle('on', LIVE.view === 'head'); $('lvViewNorth').classList.toggle('on', LIVE.view === 'north'); $('lvViewFree').classList.toggle('on', LIVE.view === 'free'); $('lvHint').classList.toggle('on', LIVE.hint); }
function liveWheel(e){
  const f = (e.deltaY || e.deltaX) < 0 ? 1.15 : 1 / 1.15;
  LIVE.zoom = Math.max(4, Math.min(600, LIVE.zoom * f)); draw();
}
function livePointerDown(e, mx, my){
  if (!LIVE.map) return;
  const X = liveXf(), w = lvS2W(X, mx, my);
  if (LIVE.hint && e.button === 0){ LIVE.hintDrag = { a: w, b: w }; draw(); return; }
  if (LIVE.view !== 'free') liveSetView('free');
  LIVE.drag = { m0: [mx, my], f0: { ...LIVE.free } };
}
function livePointerMove(mx, my){
  if (!LIVE.map) return true;
  const X = liveXf(), [wx, wy] = lvS2W(X, mx, my);
  $('hud').textContent = `x ${wx.toFixed(2)} m   y ${wy.toFixed(2)} m   ·   ${LIVE.view === 'head' ? 'cap en haut' : LIVE.view === 'north' ? 'nord en haut' : 'vue libre'}   ·   zoom ${(100 / LIVE.zoom * 100).toFixed(0)} cm / 100 px`;
  if (LIVE.hintDrag){ LIVE.hintDrag.b = [wx, wy]; draw(); return true; }
  if (LIVE.drag){
    const d = LIVE.drag, F = d.f0, c = Math.cos(-F.phi), s = Math.sin(-F.phi);
    const u = -(mx - d.m0[0]) / LIVE.zoom, v = (my - d.m0[1]) / LIVE.zoom;
    LIVE.free = { x: F.x + c * u - s * v, y: F.y + s * u + c * v, phi: F.phi }; draw();
  }
  return true;
}
function livePointerUp(){
  if (LIVE.hintDrag){
    const { a, b } = LIVE.hintDrag; LIVE.hintDrag = null; LIVE.hint = false; liveViewButtons();
    const th = Math.hypot(b[0] - a[0], b[1] - a[1]) > 0.05 ? Math.atan2(b[1] - a[1], b[0] - a[0]) * R2D : 0;
    if (!LIVE.tracker) liveReset(false);
    // pose donnée à la main : le prochain scan part de là (recherche locale élargie, comme le « 2D Pose Estimate » de ROS)
    Object.assign(LIVE.tracker, { pose: { x: a[0], y: a[1], th }, hyps: null, lost: 0, status: 'incertain' });
    LIVE.tracker.o.track = { rxy: 0.6, rth: 35 };
    LIVE.handHint = true;
    toast(`Position indiquée : (${a[0].toFixed(2)} ; ${a[1].toFixed(2)}) m, ${th.toFixed(0)}° — le prochain scan s’y recale`);
    if (!LIVE.running && LIVE.cur) liveRelocalizeCurrent();
    liveSetView(LIVE.view === 'free' ? 'head' : LIVE.view);
  }
  LIVE.drag = null;
}
// relance la localisation sur le dernier scan reçu (après une indication à la main)
function liveRelocalizeCurrent(){
  const cur = LIVE.cur; if (!cur) return;
  const pts = LL.uniformSubsample(cur.local, { max: 350 });
  const r = LL.trackerUpdate(LIVE.tracker, pts, null), T = LIVE.tracker; LIVE.cur.res = r;
  T.o.track = undefined;
  if (T.pose) LIVE.trail.push({ x: T.pose.x, y: T.pose.y, th: T.pose.th, inliers: r.inliers, mode: r.mode, status: r.status, ms: r.ms, t: Date.now(),
                                err: cur.truth ? LL.poseError(T.pose, cur.truth) : null });
  liveState(); draw();
}
// --- boutons
$('lvSrc').onchange = () => { $('lvFileRow').style.display = $('lvSrc').value === 'file' ? '' : 'none'; };
$('lvFeed').onchange = () => { $('lvSimChangeRow').style.display = $('lvFeed').value === 'sim' ? '' : 'none'; $('lvLOORow').style.display = $('lvFeed').value === 'replay' ? '' : 'none'; $('lvKidnap').style.display = $('lvFeed').value === 'sim' ? '' : 'none'; $('lvRecRow').style.display = $('lvFeed').value === 'rec' ? '' : 'none'; };
$('lvFeed').onchange();
$('lvPrep').onclick = () => { liveStop(true); LIVE.trail = []; livePrepare(); };
$('lvGridFile').onchange = async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  try { LIVE.fileGrid = LC.gridFromJSON(JSON.parse(await f.text())); toast('Grid map chargée : ' + f.name); } catch (err){ toast('Grid map illisible : ' + err.message); }
};
$('lvVecFile').onchange = async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  try { LIVE.fileVec = LC.vectorMapFromText(await f.text()); toast(`Carte vectorielle chargée : ${LIVE.fileVec.features.length} segments`); } catch (err){ toast('Carte vectorielle illisible : ' + err.message); }
};
$('lvStart').onclick = liveStart;
$('lvStop').onclick = () => liveStop();
$('lvOnce').onclick = () => {
  if (!LIVE.map && !livePrepare()) return;
  LIVE.feed = $('lvFeed').value;
  if (LIVE.feed === 'robot'){ if (!isConnected()) return toast('Robot non connecté'); LIVE.running = true; LIVE.onceStop = true; if (!SER.live){ $('live').checked = true; $('live').onchange({ target: $('live') }); } return; }
  if (LIVE.feed === 'sim' && !LIVE.sim){ $('lvSrc').value = 'sim'; livePrepare(); LIVE.simIdx = 0; }
  if (LIVE.feed === 'replay' && (!LIVE.replay.list || !LIVE.replay.list.length)) LIVE.replay = { i: 0, list: S.scans.filter(s => s.visible) };
  liveStep(); if (LIVE.view === 'free' && LIVE.tracker && LIVE.tracker.pose) liveSetView('head');
};
$('lvKidnap').onclick = () => {
  if (!LIVE.sim) return toast('Démarre d’abord le robot simulé');
  LIVE.simIdx += Math.floor(LIVE.sim.path.length / 2); LIVE.kidnapped = true;
  toast('Robot déplacé à la main (sans commande) : il doit se rendre compte qu’il est perdu et se relocaliser');
};
$('lvGlobal').onclick = () => { if (!LIVE.map) return toast('Prépare d’abord la carte'); Object.assign(LIVE.tracker, { pose: null, hyps: null, status: 'non localisé' }); if (!LIVE.running && LIVE.cur) liveRelocalizeCurrent(); else liveState(); toast('Recherche sur toute la carte au prochain scan'); };
$('lvBase').onclick = () => {
  if (!LIVE.map && !livePrepare()) return;
  if (!LIVE.tracker) liveReset(false);
  // départ connu : la base = origine de la cartographie automatique (premier scan) ; recherche locale élargie au 1er scan
  Object.assign(LIVE.tracker, { pose: { x: 0, y: 0, th: 0 }, hyps: null, lost: 0, status: 'incertain' });
  LIVE.tracker.o.track = { rxy: 0.6, rth: 35 }; LIVE.handHint = true;
  if (!LIVE.running && LIVE.cur) liveRelocalizeCurrent();
  liveSetView(LIVE.view === 'free' ? 'head' : LIVE.view);
  toast('Départ sur la base (0 ; 0, cap 0°) : le prochain scan s’y recale, puis le robot se suit');
};
$('lvHint').onclick = () => { if (!LIVE.map) return toast('Prépare d’abord la carte'); LIVE.hint = !LIVE.hint; liveViewButtons(); };
$('lvViewHead').onclick = () => liveSetView('head');
$('lvViewNorth').onclick = () => liveSetView('north');
$('lvViewFree').onclick = () => liveSetView('free');
['lvShowScan', 'lvShowTrail', 'lvShowHyps', 'lvShowVec'].forEach(id => $(id).onchange = draw);
$('lvClear').onclick = () => { LIVE.trail = []; draw(); };
$('lvExport').onclick = () => {
  if (!LIVE.trail.length) return toast('Trajectoire vide');
  let s = 'n,temps_ms,x_m,y_m,cap_deg,points_sur_carte,mode,etat,calcul_ms,erreur_pos_m,erreur_ang_deg\n';
  const t0 = LIVE.trail[0].t;
  LIVE.trail.forEach((t, k) => s += [k + 1, t.t - t0, t.x.toFixed(4), t.y.toFixed(4), t.th.toFixed(2), t.inliers.toFixed(3), t.mode, t.status, t.ms.toFixed(1),
    t.err ? t.err.pos.toFixed(4) : '', t.err ? t.err.ang.toFixed(3) : ''].join(',') + '\n');
  download('trajectoire_robot.csv', s, 'text/csv');
};
document.querySelectorAll('#lvPad [data-mv]').forEach(b => b.onclick = () => { if (!SER.wifi) return toast('Pilotage : robot connecté en WiFi'); drive(b.dataset.mv); });
$('lvPadStop').onclick = robotStop;
liveViewButtons();

setInterval(() => { if (TAB === 'live' && LIVE.running && LIVE.feed === 'robot') draw(); }, 500);   // âge du dernier scan

// --- plan de la salle
function planRecompute(){
  const R = PROC.res; if (!R) return toast('Lance d’abord le traitement');
  const t0 = performance.now(), th = R.regReport && R.regReport.theta0 !== undefined ? R.regReport.theta0 : LM.dominantDirection(R.vmap.features);
  R.plan = LM.roomPlan(R.grid, R.poses, Object.assign({ theta0: th, segs: R.vmap.features }, planOpt()));
  R.plan.cost = LM.costmap(R.plan.grid, { robotRadius: numVal('pRobotR'), inflation: numVal('pInfl') });
  delete PROC.imgs.plan; $('pLayer').value = 'plan';
  const ps = R.plan.stats;
  toast(`Plan : ${ps.corners} coins, ${ps.roomArea.toFixed(1)} m², ${ps.erased} obstacles effacés, ${ps.islands} meuble(s), ${ps.innerWalls} cloison(s) (${(performance.now() - t0).toFixed(0)} ms)`);
  draw();
}
$('pPlanRedo').onclick = planRecompute;
['pObjMax', 'pLeak', 'pNotch'].forEach(id => $(id).onchange = () => { if (PROC.res && $('pPlan').checked) planRecompute(); });
$('pPlanSend').onclick = () => {
  const R = PROC.res; if (!R || !R.plan) return toast('Pas de plan : coche « Construire le plan de la salle » et lance le traitement');
  gridSet(LM.cloneGrid(R.plan.grid), 'Plan de la salle (onglet Traitement)');
  vecPush(); VEC.map = { features: R.plan.segments.map(s => ({ ...s })), corners: LC.computeCorners(R.plan.segments, LC.VM_DEFAULTS), scans: 0, nextId: R.plan.segments.length + 1 };
  VEC.last = null; VEC.sel = -1; vecRefresh();
  toast('Plan envoyé vers les onglets Grid map et Vectorielle');
};
$('pExpPlan').onclick = () => {
  const R = PROC.res; if (!R || !R.plan) return toast('Pas de plan');
  const { pgm, yaml } = LC.gridToPGM(R.plan.grid, 'plan_salle');
  download('plan_salle.pgm', pgm, 'image/x-portable-graymap'); setTimeout(() => download('plan_salle.yaml', yaml, 'text/yaml'), 300);
};
$('pExpPlanTxt').onclick = () => {
  const R = PROC.res; if (!R || !R.plan) return toast('Pas de plan');
  let s = '# Plan de la salle : contour (C), meubles (M), cloisons (W) ; sommets x,y en metres, dans l ordre\n';
  s += 'C,' + R.plan.outer.map(p => p[0].toFixed(3) + ' ' + p[1].toFixed(3)).join(',') + '\n';
  R.plan.islands.forEach(I => s += 'M,' + I.map(p => p[0].toFixed(3) + ' ' + p[1].toFixed(3)).join(',') + '\n');
  (R.plan.innerWalls || []).forEach(([a, b]) => s += 'W,' + [a, b].map(p => p[0].toFixed(3) + ' ' + p[1].toFixed(3)).join(',') + '\n');
  s += '# Segments x1,y1,x2,y2\n' + R.plan.segments.map(f => [f.x1, f.y1, f.x2, f.y2].map(v => v.toFixed(4)).join(',')).join('\n') + '\n';
  download('plan_salle.txt', s, 'text/plain');
};

// --- enregistrement d'une session (scans + commandes) et relecture à l'identique
function liveRecState(){
  $('lvRec').textContent = LIVE.rec ? '■ Arrêter l’enregistrement' : '● Enregistrer';
  $('lvRec').classList.toggle('on', !!LIVE.rec);
  const R = LIVE.rec || LIVE.recLast;
  $('lvRecState').textContent = R ? `${R.scans.length} scans, ${R.cmds.length} commandes${LIVE.rec ? ' (en cours)' : ''}` : '';
}
$('lvRec').onclick = () => {
  if (LIVE.rec){ LIVE.recLast = LIVE.rec; LIVE.rec = null; toast(`Enregistrement arrêté : ${LIVE.recLast.scans.length} scans`); }
  else { LIVE.rec = { format: 'ld06-session-direct', version: 1, t0: Date.now(), date: new Date().toISOString(), scans: [], cmds: [],
                      speeds: { v: numVal('lvV'), w: numVal('lvW') }, mirror: !!S.opt.mirror }; toast('Enregistrement : chaque scan reçu et chaque commande sont gardés'); }
  liveRecState();
};
setInterval(() => { if (LIVE.rec) liveRecState(); }, 1000);
$('lvRecSave').onclick = () => {
  const R = LIVE.rec || LIVE.recLast; if (!R || !R.scans.length) return toast('Rien à télécharger : lance l’enregistrement pendant que le robot scanne');
  download(`session_direct_${R.date.slice(0, 19).replace(/[:T]/g, '-')}.json`, JSON.stringify(R), 'application/json');
};
$('lvRecFile').onchange = async e => {
  const f = e.target.files[0]; e.target.value = ''; if (!f) return;
  try {
    const d = JSON.parse(await f.text()); if (!d.scans || !d.scans.length) throw new Error('aucun scan');
    LIVE.recPlay = { data: { scans: d.scans, cmds: d.cmds || [] }, i: 0 };
    if (d.speeds){ $('lvV').value = d.speeds.v; $('lvW').value = d.speeds.w; }
    $('lvFeed').value = 'rec'; $('lvFeed').onchange();
    $('lvRecInfo').textContent = `${d.scans.length} scans, ${(d.cmds || []).length} commandes`;
    toast('Enregistrement chargé : prépare la carte puis Démarrer');
  } catch (err){ toast('Enregistrement illisible : ' + err.message); }
};
liveRecState();

// --- correction de la carte
function liveCorrReset(quiet){
  LIVE.corr = LIVE.map ? LM.createMapCorrector(LIVE.map.grid) : null;
  LIVE.corrDiff = null; LIVE.proposal = null; LIVE.corrImg = null; LIVE.showProposal = false;
  $('lvCorrView').classList.remove('on'); liveCorrInfo();
  if (!quiet) toast('Corrections oubliées');
}
function liveCorrUpdate(){
  if (!LIVE.corr) return;
  LIVE.corrDiff = LM.correctorDiff(LIVE.corr);
  LIVE.proposal = LM.correctorProposal(LIVE.corr, LIVE.corrDiff);
  LIVE.corrImg = null; if (LIVE.showProposal) LIVE.mapImg = null;
  liveCorrInfo();
}
function liveCorrInfo(){
  const D = LIVE.corrDiff, C = LIVE.corr, el = $('lvCorrInfo');
  if (!C || !C.n){ el.textContent = 'Aucune correction pour l’instant : démarre le suivi (les scans pris à une position sûre sont utilisés).'; return; }
  const r = LIVE.map.grid.res * LIVE.map.grid.res;
  el.innerHTML = `<b>${C.n}</b> scans comparés à la carte · <span style="color:#4cd97b"><b>${D ? D.nAdd : 0}</b> cases à ajouter</span> (${D ? (D.nAdd * r).toFixed(2) : 0} m²) · ` +
    `<span style="color:#ff6b6b"><b>${D ? D.nRem : 0}</b> à retirer</span> (${D ? (D.nRem * r).toFixed(2) : 0} m²)` +
    (C.n < 10 ? '<br><span class="help">Encore peu de scans : les corrections se confirment au fil du trajet (une case doit être vue plusieurs fois).</span>' : '');
}
function liveCorrImage(){
  if (LIVE.corrImg) return LIVE.corrImg;
  const D = LIVE.corrDiff, g = LIVE.map.grid; if (!D) return null;
  const c = document.createElement('canvas'); c.width = g.W; c.height = g.H;
  const x = c.getContext('2d'), im = x.createImageData(g.W, g.H);
  for (let j = 0; j < g.H; j++) for (let i = 0; i < g.W; i++){
    const k = j * g.W + i, o = ((g.H - 1 - j) * g.W + i) * 4;
    if (D.add[k]){ im.data[o] = 76; im.data[o + 1] = 217; im.data[o + 2] = 123; im.data[o + 3] = 255; }
    else if (D.rem[k]){ im.data[o] = 255; im.data[o + 1] = 92; im.data[o + 2] = 92; im.data[o + 3] = 255; }
  }
  x.putImageData(im, 0, 0); return LIVE.corrImg = c;
}
const needCorr = () => { if (!LIVE.corrDiff || !(LIVE.corrDiff.nAdd || LIVE.corrDiff.nRem)){ toast('Pas encore de correction : fais rouler le robot (suivi sûr) pour comparer ses scans à la carte'); return false; } return true; };
$('lvCorrView').onclick = () => {
  if (!needCorr()) return;
  LIVE.showProposal = !LIVE.showProposal; LIVE.mapImg = null;
  $('lvCorrView').classList.toggle('on', LIVE.showProposal);
  $('lvCorrView').textContent = LIVE.showProposal ? 'Revoir la carte actuelle' : 'Voir la carte proposée';
  draw();
};
$('lvCorrAdopt').onclick = () => {
  if (!needCorr()) return;
  const P = LIVE.proposal, label = LIVE.map.label, D = LIVE.corrDiff;
  // segments qui ne reposent plus sur un obstacle de la carte corrigée (meuble retiré) : supprimés
  const onObst = f => { const L = LC.segLen(f), n = Math.max(2, Math.ceil(L / 0.05)); let ok = 0;
    for (let q = 0; q <= n; q++){ const x = f.x1 + (f.x2 - f.x1) * q / n, y = f.y1 + (f.y2 - f.y1) * q / n, [i, j] = LC.worldToCell(P, x, y);
      let hit = false; for (let dj = -1; dj <= 1 && !hit; dj++) for (let di = -1; di <= 1; di++){ const v = LC.getCell(P, i + di, j + dj); if (v !== undefined && v >= LC.OCC_T){ hit = true; break; } }
      if (hit) ok++; }
    return ok / (n + 1) >= 0.5; };
  const segs = LIVE.map.segs.filter(onObst), dropped = LIVE.map.segs.length - segs.length;
  LIVE.map = LL.prepareMap(P, segs); LIVE.map.label = label + ' (corrigée)'; LIVE.mapImg = null;
  if (LIVE.tracker) LIVE.tracker.M = LIVE.map;
  LIVE.looCache.clear();
  liveCorrReset(true);
  $('lvCorrView').textContent = 'Voir la carte proposée';
  const st = LC.gridStats(P);
  lvSetInfo(`<b>Carte corrigée adoptée</b> : ${D.nAdd} cases ajoutées, ${D.nRem} retirées, ${dropped} segment(s) retiré(s)<br>${P.W} × ${P.H} cellules · ${(st.free * P.res * P.res).toFixed(0)} m² libres · ${LIVE.map.cands.length} positions candidates`);
  toast('Carte corrigée adoptée : le suivi continue sur la nouvelle carte'); draw();
};
$('lvCorrSend').onclick = () => {
  const P = LIVE.corrDiff && (LIVE.corrDiff.nAdd || LIVE.corrDiff.nRem) ? LIVE.proposal : LIVE.map && LIVE.map.grid;
  if (!P) return toast('Pas de carte');
  gridSet(LM.cloneGrid(P), LIVE.proposal ? 'Carte corrigée (Robot en direct)' : 'Carte du robot en direct'); toast('Carte envoyée vers l’onglet Grid map');
};
$('lvCorrExport').onclick = () => {
  const P = LIVE.corrDiff && (LIVE.corrDiff.nAdd || LIVE.corrDiff.nRem) ? LIVE.proposal : LIVE.map && LIVE.map.grid;
  if (!P) return toast('Pas de carte');
  const { pgm, yaml } = LC.gridToPGM(P, 'carte_corrigee');
  download('carte_corrigee.pgm', pgm, 'image/x-portable-graymap'); setTimeout(() => download('carte_corrigee.yaml', yaml, 'text/yaml'), 300);
};
$('lvCorrReset').onclick = () => { liveCorrReset(false); LIVE.mapImg = null; draw(); };
$('lvShowCorr').onchange = draw;
liveCorrInfo();

// salle simulée modifiée : armoire retirée, 6 cartons et 10 chaises ajoutés (la carte ne le sait pas)
function liveChangedScene(D){
  if (D.changed) return D.changed;
  const S = LC.seg, rng = LL.mulberry32(1234), W = D.scene.walls.filter(w => !(Math.max(w.x1, w.x2) <= 0.61 && Math.min(w.y1, w.y2) >= 2.99 && Math.max(w.y1, w.y2) <= 5.01));
  const box = (x, y, w) => W.push(S(x, y, x + w, y), S(x + w, y, x + w, y + w), S(x + w, y + w, x, y + w), S(x, y + w, x, y));
  const far = (x, y) => D.path.every(p => Math.hypot(p.x - x, p.y - y) > 0.7);
  for (let n = 0, k = 0; n < 6 && k < 500; k++){ const x = 1 + rng() * 11.5, y = 1 + rng() * 6.5; if (D.scene.free(x, y, 0.6) && far(x, y)){ box(x, y, 0.5); n++; } }
  for (let n = 0, k = 0; n < 10 && k < 500; k++){ const x = 1 + rng() * 11.5, y = 1 + rng() * 6.5; if (D.scene.free(x, y, 0.5) && far(x, y)){ for (const [a, b] of [[0, 0], [0.4, 0], [0, 0.4], [0.4, 0.4]]) box(x + a, y + b, 0.03); n++; } }
  return D.changed = W;
}

// ---------------------------------------------------------------- RETROUVER LA POSITION (scan + grid map) ET VALIDATION
const FIND = { fileGrid: null, pgm: null, yaml: null, res: null, map: null, img: null, imgGrid: null, batch: null };
function fdFillScans(){
  const sel = $('fdScan'), cur = sel.value;
  sel.innerHTML = S.scans.map(s => `<option value="${s.id}">${s.name}</option>`).join('') + '<option value="live">Dernier scan reçu du robot (aperçu direct)</option>';
  if ([...sel.options].some(o => o.value === cur)) sel.value = cur;
}
function fdRefPose(s){ return (typeof LOC !== 'undefined' && LOC.refPoses.get(s.id)) || s.pose; }
// carte choisie -> { grid, segs, label } ; except = id du scan testé (carte des autres scans)
function fdMapFor(except){
  const src = $('fdMap').value;
  if (src === 'others'){
    const vis = S.scans.filter(s => s.visible && s.id !== except);
    if (vis.length < 2) throw new Error('il faut au moins 2 autres scans visibles');
    const R = LL.buildReferences(vis.map(s => ({ ...rawScan(s), pose: { ...fdRefPose(s) } })), -1);
    return { grid: R.grid, segs: R.vmap.features, label: `carte des ${vis.length} autres scans` };
  }
  if (src === 'grid'){ if (!GRID.g) throw new Error('pas de carte dans l’onglet Grid map'); return { grid: GRID.g, segs: null, label: 'carte de l’onglet Grid map' }; }
  if (src === 'proc'){ if (!PROC.res) throw new Error('lance d’abord le traitement'); return { grid: PROC.res.grid, segs: PROC.res.vmap.features, label: 'grid map traitée' }; }
  if (src === 'plan'){ if (!PROC.res || !PROC.res.plan) throw new Error('pas de plan (onglet Traitement)'); return { grid: PROC.res.plan.grid, segs: PROC.res.plan.segments, label: 'plan de la salle' }; }
  if (!FIND.fileGrid) throw new Error('charge d’abord la carte (.json, ou .pgm + .yaml)');
  return { grid: FIND.fileGrid, segs: null, label: 'fichier ' + FIND.fileName };
}
function fdScanData(){
  const v = $('fdScan').value;
  if (v === 'live'){ const L = SER.liveScan; if (!L) throw new Error('aucun scan reçu : connecte le robot et coche « Aperçu direct »'); return { name: 'dernier scan du robot', raw: lvRaw({ a: L.a, d: L.d, i: L.i }), scan: null }; }
  const s = S.scans.find(x => String(x.id) === v); if (!s) throw new Error('scan introuvable');
  return { name: s.name, raw: lvRaw(s), scan: s };
}
function fdTruth(sd){
  if ($('fdUseRef').checked) return sd.scan ? { ...fdRefPose(sd.scan) } : null;
  const x = parseFloat($('fdTx').value), y = parseFloat($('fdTy').value), th = parseFloat($('fdTt').value);
  return isFinite(x) && isFinite(y) ? { x, y, th: isFinite(th) ? th : NaN } : null;
}
function fdRun(){
  try {
    const sd = fdScanData(), mp = fdMapFor($('fdMap').value === 'others' && sd.scan ? sd.scan.id : undefined);
    const t0 = performance.now(), M = LL.prepareMap(mp.grid, mp.segs);
    const local = LL.localPoints(sd.raw), r = LL.locateScan(M, local);
    FIND.res = { ...r, local, name: sd.name, scanId: sd.scan && sd.scan.id, truth: fdTruth(sd), mapLabel: mp.label, msAll: performance.now() - t0 };
    FIND.map = mp.grid; FIND.img = null;
    fdShow(); fdFit(); draw();
  } catch (e){ toast('Retrouver la position : ' + e.message); }
}
function fdShow(){
  const r = FIND.res, el = $('fdResult'); if (!r){ el.textContent = 'Choisis une carte et un scan.'; return; }
  const col = { 'sûr': '#4cd97b', 'ambigu': '#ffb347', 'incertain': '#ff6b6b' }[r.confidence];
  let h = `<b>${r.name}</b> sur ${r.mapLabel}<br><span style="color:${col};font-weight:700">● ${r.confidence}</span> · ${(r.inliers * 100).toFixed(0)} % des points sur la carte · ${r.msAll.toFixed(0)} ms` +
    `<br>position trouvée : x <b>${r.pose.x.toFixed(3)}</b> m · y <b>${r.pose.y.toFixed(3)}</b> m · θ <b>${r.pose.th.toFixed(1)}°</b>` +
    `<br>dispersion due au bruit des points (2σ) : ± <b>${(2 * r.sigma.xy * 100).toFixed(1)} cm</b> · ± <b>${(2 * r.sigma.th).toFixed(2)}°</b> <span class="help">(borne basse : ne compte pas les défauts de la carte ; erreur réelle mesurée sur tes scans ≈ 2 cm)</span>`;
  if (r.hyps.length > 1) h += `<br><span class="help">autres positions possibles : ${r.hyps.slice(1, 4).map(q => `(${q.x.toFixed(2)} ; ${q.y.toFixed(2)} ; ${q.th.toFixed(0)}°) ${(q.inliers * 100).toFixed(0)} %`).join(' · ')}</span>`;
  if (r.truth){
    const e = LL.poseError(r.pose, isFinite(r.truth.th) ? r.truth : { ...r.truth, th: r.pose.th }), ok = e.pos <= 0.05 && (!isFinite(r.truth.th) || e.ang <= 2);
    h += `<div class="res">position réelle (${r.truth.x.toFixed(3)} ; ${r.truth.y.toFixed(3)}${isFinite(r.truth.th) ? ' ; ' + r.truth.th.toFixed(1) + '°' : ''})<br>` +
      `erreur : <b style="color:${ok ? '#4cd97b' : '#ff6b6b'}">${(e.pos * 100).toFixed(1)} cm${isFinite(r.truth.th) ? ', ' + e.ang.toFixed(2) + '°' : ''}</b> → ${ok ? 'réussite' : 'échec'} (seuil 5 cm / 2°)` +
      `<br><span class="help">rappel : les poses de référence réelles sont connues à ± 3 cm / ± 0,6° environ</span></div>`;
  }
  el.innerHTML = h;
}
function fdFit(){
  const g = FIND.map; if (!g) return;
  S.view.x = g.x0 + g.W * g.res / 2; S.view.y = g.y0 + g.H * g.res / 2; S.view.s = Math.min(W / (g.W * g.res), H / (g.H * g.res)) * 0.9;
}
function drawFindUnder(){
  const g = FIND.map; if (!g || !FIND.res) return;
  if (!FIND.img || FIND.imgGrid !== g){
    const c = document.createElement('canvas'); c.width = g.W; c.height = g.H; const x = c.getContext('2d'), im = x.createImageData(g.W, g.H);
    for (let j = 0; j < g.H; j++) for (let i = 0; i < g.W; i++){ const v = g.data[j * g.W + i], o = ((g.H - 1 - j) * g.W + i) * 4, [r, gg, b, a] = v >= LC.OCC_T ? [214, 222, 232, 255] : v <= LC.FREE_T ? [36, 44, 56, 255] : [0, 0, 0, 0]; im.data[o] = r; im.data[o + 1] = gg; im.data[o + 2] = b; im.data[o + 3] = a; }
    x.putImageData(im, 0, 0); FIND.img = c; FIND.imgGrid = g;
  }
  const [sx, sy] = w2s(g.x0, g.y0 + g.H * g.res);
  ctx.save(); ctx.imageSmoothingEnabled = false; ctx.globalAlpha = 0.9; ctx.drawImage(FIND.img, sx, sy, g.W * g.res * S.view.s, g.H * g.res * S.view.s); ctx.restore();
}
function drawFindOver(){
  const r = FIND.res; if (!r) return;
  const robot = (p, col, dashed, label) => {
    const [x, y] = w2s(p.x, p.y), a = (p.th || 0) * D2R, R = Math.max(8, 0.15 * S.view.s);
    ctx.save(); ctx.strokeStyle = col; ctx.fillStyle = col + '33'; ctx.lineWidth = 2.5; if (dashed) ctx.setLineDash([5, 4]);
    ctx.beginPath(); ctx.arc(x, y, R, 0, 2 * Math.PI); ctx.fill(); ctx.stroke(); ctx.setLineDash([]);
    if (isFinite(p.th)){ ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + Math.cos(a) * R * 2, y - Math.sin(a) * R * 2); ctx.stroke(); }
    if (label){ ctx.fillStyle = col; ctx.font = '600 12px system-ui'; ctx.fillText(label, x + R + 4, label === 'trouvée' ? y + R + 12 : y - R); }
    ctx.restore();
  };
  for (const q of r.hyps.slice(1, 4)) robot(q, '#ffb347', true, `${(q.inliers * 100).toFixed(0)} %`);
  const p = r.pose, c = Math.cos(p.th * D2R), s = Math.sin(p.th * D2R), F = LL.prepareMap ? null : null;
  ctx.save();
  for (const q of r.local){ const wx = p.x + c * q.x - s * q.y, wy = p.y + s * q.x + c * q.y, [sx, sy] = w2s(wx, wy); ctx.fillStyle = '#3ee6e6'; ctx.fillRect(sx - 1.3, sy - 1.3, 2.6, 2.6); }
  ctx.restore();
  // ellipse d'incertitude (2σ, agrandie si invisible)
  const [ex, ey] = w2s(p.x, p.y); ctx.save(); ctx.strokeStyle = '#ffe14d'; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(ex, ey, Math.max(4, 2 * r.sigma.xy * S.view.s), 0, 2 * Math.PI); ctx.stroke(); ctx.restore();
  if (r.truth) robot(r.truth, '#ffffff', true, 'réelle');
  robot(p, '#4cd97b', false, 'trouvée');
}
async function fdBatch(){
  const vis = S.scans.filter(s => s.visible); if (vis.length < 3) return toast('Il faut au moins 3 scans visibles, superposés');
  if (typeof LOC !== 'undefined' && !LOC.refPoses.size) locMemorize(true);
  $('fdBatch').disabled = true; const rows = [], out = $('fdBatchOut');
  try {
    let fixed = null;
    if ($('fdMap').value !== 'others'){ const mp = fdMapFor(); fixed = { M: LL.prepareMap(mp.grid, mp.segs), label: mp.label }; }
    for (let k = 0; k < vis.length; k++){
      const s = vis[k]; out.textContent = `${k + 1}/${vis.length} · ${s.name}…`; await sleep(0);
      let M = fixed && fixed.M, label = fixed && fixed.label;
      if (!fixed){ const mp = fdMapFor(s.id); M = LL.prepareMap(mp.grid, mp.segs); label = mp.label; }
      const t0 = performance.now(), r = LL.locateScan(M, LL.localPoints(lvRaw(s))), truth = fdRefPose(s), e = LL.poseError(r.pose, truth);
      rows.push({ name: s.name, map: label, x: r.pose.x, y: r.pose.y, th: r.pose.th, tx: truth.x, ty: truth.y, tth: truth.th, epos: e.pos, eang: e.ang,
                  ok: e.pos <= 0.05 && e.ang <= 2, conf: r.confidence, inl: r.inliers, s2xy: 2 * r.sigma.xy, s2th: 2 * r.sigma.th, ms: performance.now() - t0 });
    }
    FIND.batch = rows;
    const med = a => { const b = a.slice().sort((x, y) => x - y); return b[Math.floor(b.length / 2)]; }, p90 = a => { const b = a.slice().sort((x, y) => x - y); return b[Math.min(b.length - 1, Math.floor(b.length * 0.9))]; };
    const okR = rows.filter(r => r.ok), sure = rows.filter(r => r.conf === 'sûr'), cover = rows.filter(r => r.epos <= Math.max(r.s2xy, 0.03) + 0.03).length;
    let h = `<b>${okR.length}/${rows.length} réussites</b> (≤ 5 cm et ≤ 2°) · « sûr » : ${sure.length}, dont faux <b>${sure.filter(r => !r.ok).length}</b>` +
      `<br>erreur (réussites) : médiane ${(med(okR.map(r => r.epos)) * 100 || 0).toFixed(1)} cm / ${(med(okR.map(r => r.eang)) || 0).toFixed(2)}°, 90e centile ${(p90(okR.map(r => r.epos)) * 100 || 0).toFixed(1)} cm / ${(p90(okR.map(r => r.eang)) || 0).toFixed(2)}° · temps médian ${med(rows.map(r => r.ms)).toFixed(0)} ms` +
      `<br><span class="help">erreur ≤ dispersion 2σ + 3 cm (incertitude des poses de référence) : ${cover}/${rows.length}</span>`;
    h += '<table class="bench" style="margin-top:6px"><tr><th>Scan</th><th>Erreur</th><th>Confiance</th><th>Points</th><th>Dispersion 2σ</th><th>ms</th></tr>' +
      rows.map(r => `<tr><td>${r.name}</td><td style="color:${r.ok ? '#4cd97b' : '#ff6b6b'}">${(r.epos * 100).toFixed(1)} cm, ${r.eang.toFixed(2)}°</td><td>${r.conf}</td><td>${(r.inl * 100).toFixed(0)} %</td><td>${(r.s2xy * 100).toFixed(1)} cm, ${r.s2th.toFixed(2)}°</td><td>${r.ms.toFixed(0)}</td></tr>`).join('') + '</table>';
    out.innerHTML = h;
  } catch (e){ out.textContent = 'Erreur : ' + e.message; }
  finally { $('fdBatch').disabled = false; }
}
$('fdMap').onchange = () => { $('fdFileRow').style.display = $('fdMap').value === 'file' ? '' : 'none'; };
$('fdUseRef').onchange = () => { $('fdTruthRow').style.display = $('fdUseRef').checked ? 'none' : ''; };
$('fdFile').onchange = async e => {
  const files = [...e.target.files]; e.target.value = '';
  try {
    for (const f of files){
      if (/\.json$/i.test(f.name)){ FIND.fileGrid = LC.gridFromJSON(JSON.parse(await f.text())); FIND.fileName = f.name; }
      else if (/\.pgm$/i.test(f.name)){ FIND.pgm = new Uint8Array(await f.arrayBuffer()); FIND.pgmName = f.name; }
      else if (/\.ya?ml$/i.test(f.name)) FIND.yaml = await f.text();
    }
    if (FIND.pgm){ FIND.fileGrid = LC.gridFromPGM(FIND.pgm, FIND.yaml || ''); FIND.fileName = FIND.pgmName + (FIND.yaml ? ' + yaml' : ' (sans yaml : résolution 5 cm, origine 0)'); }
    if (FIND.fileGrid) $('fdFileInfo').textContent = `${FIND.fileName} : ${FIND.fileGrid.W} × ${FIND.fileGrid.H}, ${(FIND.fileGrid.res * 100).toFixed(0)} cm`;
  } catch (err){ toast('Carte illisible : ' + err.message); }
};
$('fdGo').onclick = fdRun;
$('fdClear').onclick = () => { FIND.res = null; FIND.map = null; fdShow(); draw(); };
$('fdPlace').onclick = () => {
  const r = FIND.res; if (!r || r.scanId === undefined || r.scanId === null) return toast('Retrouve d’abord la position d’un scan de la liste');
  const s = S.scans.find(x => x.id === r.scanId); if (!s) return;
  pushUndo(s); s.pose = { x: r.pose.x, y: r.pose.y, th: r.pose.th }; if (typeof renderList === 'function') renderList(); draw(); toast(`« ${s.name} » placé à la position trouvée`);
};
$('fdBatch').onclick = fdBatch;
$('fdBatchCsv').onclick = () => {
  if (!FIND.batch) return toast('Lance d’abord la validation');
  let s = 'scan,carte,x_m,y_m,theta_deg,x_ref_m,y_ref_m,theta_ref_deg,erreur_pos_m,erreur_ang_deg,reussite,confiance,points_sur_carte,incertitude_2sigma_m,incertitude_2sigma_deg,temps_ms\n';
  for (const r of FIND.batch) s += [r.name, '"' + r.map + '"', r.x.toFixed(4), r.y.toFixed(4), r.th.toFixed(2), r.tx.toFixed(4), r.ty.toFixed(4), r.tth.toFixed(2), r.epos.toFixed(4), r.eang.toFixed(3), r.ok ? 1 : 0, r.conf, r.inl.toFixed(3), r.s2xy.toFixed(4), r.s2th.toFixed(3), r.ms.toFixed(0)].join(',') + '\n';
  download('validation_localisation.csv', s, 'text/csv');
};
// la liste des scans change souvent : on la remplit quand on ouvre l'onglet
document.querySelector('.tabs button[data-tab="loc"]').addEventListener('click', fdFillScans);
fdFillScans();

// ---------------------------------------------------------------- CARTOGRAPHIE AUTOMATIQUE (onglet Scans)
const AM = { running: false, src: 'robot', mp: null, busy: false, pending: null, motion: null, cur: null, added: 0, prefix: 'auto', sim: null, simIdx: 0, simPrev: null, timer: null, t0: 0, n: 0 };
function amUndoMirror(a){ const o = S.opt, sg = o.mirror ? -1 : 1; return a.map(v => (((sg * (v - (o.aOff || 0))) % 360) + 360) % 360); }
function amStart(){
  if (AM.running) return;
  AM.src = $('amSrc').value;
  if (AM.src === 'robot' && !isConnected()) return toast('Connecte d’abord le robot (USB ou WiFi), ou choisis « Robot simulé »');
  AM.mp = LL.createMapper({ kfDist: numVal('amDist'), kfRot: numVal('amRot'), tours: Math.max(1, Math.round(numVal('amTours'))) });
  let k = 1; while (S.scans.some(s => s.name.startsWith(`auto${k}_`))) k++;
  AM.prefix = `auto${k}_`; AM.added = 0; AM.n = 0; AM.t0 = performance.now(); AM.running = true; AM.motion = null; AM.cur = null;
  if (S.scans.length) toast('Les nouvelles positions ont leur propre repère (la première est à l’origine) : masque les anciens scans pour ne voir que la nouvelle carte');
  $('amStart').disabled = true; $('amSrc').disabled = true;
  if (AM.src === 'robot'){ if (!SER.live){ $('live').checked = true; $('live').onchange({ target: $('live') }); } }
  else { AM.sim = AM.sim || LL.demoScenario(5); AM.simIdx = 0; AM.simPrev = null; amSimTick(); }
  amInfo();
}
function amStop(){
  if (!AM.running) return;
  AM.running = false; clearTimeout(AM.timer);
  if (AM.mp && AM.mp.pending){ const kf = LL.mapperCommit(AM.mp); if (kf) amAddKeyframe(kf); }
  $('amStart').disabled = false; $('amSrc').disabled = false; AM.cur = null;
  amInfo(true); draw();
}
function amAddKeyframe(kf){
  AM.added++;
  const s = addScan(`${AM.prefix}${String(AM.added).padStart(2, '0')}`, amUndoMirror(kf.a), kf.d, kf.i, kf.pose);
  s.locked = true;
  if (typeof renderList === 'function') renderList();
}
function amFeed(b, truth){
  if (!AM.running) return;
  if (AM.busy){ AM.pending = [b, truth]; return; }
  AM.busy = true;
  try {
    const raw = lvRaw(b), motion = AM.motion; AM.motion = null; AM.n++;
    const r = LL.mapperUpdate(AM.mp, raw, motion);
    if (r.keyframe) amAddKeyframe(r.keyframe);
    AM.cur = { local: LL.localPoints(raw), r, truth };
    if (AM.n % 2 === 0 || r.keyframe) amInfo();
    draw();
  } catch (e){ console.error(e); toast('Cartographie : ' + e.message); }
  finally {
    AM.busy = false;
    if (AM.pending){ const [pb, pt] = AM.pending; AM.pending = null; setTimeout(() => amFeed(pb, pt), 0); }
  }
}
function amOnDrive(mv, ms){
  if (!AM.running) return;
  const v = numVal('lvV') * ms / 1000, w = numVal('lvW') * ms / 1000;
  const d = { forward: [v, 0, 0], backward: [-v, 0, 0], left: [0, v, 0], right: [0, -v, 0], rotateleft: [0, 0, w], rotateright: [0, 0, -w] }[mv];
  if (!d) return; const m = { x: d[0], y: d[1], th: d[2] };
  AM.motion = AM.motion ? LL.compose(AM.motion, m) : m;
}
// robot simulé : trajet dans la salle, arrêt de 3 scans tous les 3 pas, odométrie bruitée de 10 %
function amSimTick(){
  if (!AM.running) return;
  const D = AM.sim, k = AM.simIdx++;
  const seq = D.seq || (D.seq = (() => { const q = []; D.path.forEach((p, j) => { q.push(p); if (j % 3 === 0) q.push(p, p); }); return q; })());
  if (k >= seq.length){ amStop(); toast('Robot simulé : trajet terminé'); return; }
  const truth = seq[k];
  if (AM.simPrev){ const m = LL.relPose(AM.simPrev, truth), g = () => (D.rng() - 0.5) * 0.2; const mo = { x: m.x * (1 + g()), y: m.y * (1 + g()), th: m.th * (1 + g()) }; AM.motion = AM.motion ? LL.compose(AM.motion, mo) : mo; }
  AM.simPrev = truth;
  amFeed(LL.simulateScan(D.scene.walls, truth, { tours: 1 }, D.rng), LL.relPose(seq[0], truth));
  AM.timer = setTimeout(amSimTick, 70);
}
function amInfo(done){
  const el = $('amInfo'), Mp = AM.mp;
  if (!Mp){ el.textContent = 'Prêt.'; return; }
  const r = AM.cur && AM.cur.r, col = r && r.status === 'perdu' ? '#ff6b6b' : '#4cd97b';
  const dur = (performance.now() - AM.t0) / 1000;
  let h = `${done ? '<b>Terminé</b>' : `<span style="color:${col};font-weight:700">● ${r ? r.status : 'en attente du premier scan'}</span>`} · <b>${AM.added}</b> positions enregistrées · ${AM.n} scans reçus en ${dur.toFixed(0)} s`;
  if (r && !done) h += `<br>robot : x ${r.pose.x.toFixed(2)} m · y ${r.pose.y.toFixed(2)} m · cap ${r.pose.th.toFixed(0)}° · ${(r.inliers * 100).toFixed(0)} % des points sur la carte · ${r.ms.toFixed(0)} ms`;
  if (r && r.status === 'perdu' && !done) h += '<br><span style="color:#ff6b6b">Le scan ne colle plus à la carte : reviens doucement vers une zone déjà cartographiée (aucune position n’est enregistrée tant qu’il est perdu).</span>';
  if (AM.cur && AM.cur.truth && !done){ const e = LL.poseError(r.pose, AM.cur.truth); h += `<br>écart à la vraie pose (simulation) : ${(e.pos * 100).toFixed(1)} cm, ${e.ang.toFixed(2)}°`; }
  if (done && AM.added) h += '<br>Ensuite : <b>Construire la grid map</b>, puis l’onglet Traitement (plan, murs nets) et Robot en direct.';
  el.innerHTML = h;
}
function drawAutoMap(){
  if (!AM.running || !AM.cur) return;
  const r = AM.cur.r, p = r.pose, c = Math.cos(p.th * D2R), s = Math.sin(p.th * D2R);
  ctx.save(); ctx.fillStyle = r.status === 'perdu' ? '#ff6b6b' : '#ffffff';
  for (const q of AM.cur.local){ const [x, y] = w2s(p.x + c * q.x - s * q.y, p.y + s * q.x + c * q.y); ctx.fillRect(x - 1.2, y - 1.2, 2.4, 2.4); }
  const [x, y] = w2s(p.x, p.y), R = Math.max(7, 0.15 * S.view.s);
  ctx.strokeStyle = r.status === 'perdu' ? '#ff6b6b' : '#4cd97b'; ctx.fillStyle = ctx.strokeStyle + '44'; ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.arc(x, y, R, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + c * R * 2, y - s * R * 2); ctx.stroke();
  ctx.restore();
}
$('amStart').onclick = amStart;
$('amStop').onclick = amStop;
$('amGrid').onclick = () => { if (!S.scans.some(s => s.visible)) return toast('Aucun scan visible'); setTab('grid'); gridFromAll(); fitGrid(); };
