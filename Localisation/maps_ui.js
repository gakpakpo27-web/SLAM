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
           free: numVal('gFree'), occ: numVal('gOcc'), init: +$('gInit').value, fillBetween: $('gFill').checked };
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
  if (!GRID.g || !$('gShow').checked) return;
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
$('gModel').onchange = () => { const b = $('gModel').value === 'binary'; $('gFree').disabled = b; $('gOcc').disabled = b; };
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
  const r = window.LidarTests.runTests(LC, window.LidarLoc), box = $('sTestOut');
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
