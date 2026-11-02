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
  const r = window.LidarTests.runTests(LC), box = $('sTestOut');
  let html = `<b class="${r.passed === r.total ? 'ok' : 'ko'}">${r.passed}/${r.total} tests réussis</b>`, grp = '';
  for (const t of r.results){
    if (t.group !== grp){ grp = t.group; html += `<div class="tg">${grp}</div>`; }
    html += `<div class="${t.ok ? 'ok' : 'ko'}">${t.ok ? '✓' : '✗'} ${t.name}${t.ok ? '' : ' — ' + t.msg}</div>`;
  }
  box.innerHTML = html; box.hidden = false;
};
renderGridInfo(); segRefresh();
