// ============================================================================
//  ESP32 LD06 – WiFi (firmware du cours ESP32RobotControl, version avec interface web,
//  verrou moteurs et trajectoires) + USB (capture CSV / Teleplot pour lidar_viewer.html)
//
//  WiFi : point d'accès « RobotESP32 » / « 12345678 », http://192.168.4.1
//    /            interface web (pilotage, trajectoires, tracé)
//    /scan        scan binaire (lidar_viewer.html, mode WiFi)        /pose ?x=mm&y=mm&z=centidegrés
//    /forward /backward /left /right /rotateleft /rotateright ?time=ms   (403 LOCKED si verrouillé)
//    /stop  /arm  /disarm  /status  /traj?seq=F:1000,TR:450&repeat=1  /traj/stop  /ping
//  Moteurs : arrêtés dès le démarrage (Init + Stop), VERROUILLÉS tant qu'on n'a pas appelé /arm,
//            reverrouillés après 2 min sans mouvement, ordre d'arrêt rappelé toutes les 300 ms au repos.
//
//  USB (ajout pour le TP, comme ESP32_LD06_Capture.ino) :
//    s = 1 scan | 1..9 = N scans | c = flux CSV | t = Teleplot | x = stop | ? = infos
//    Format : #SCAN,<id>,<nbPoints>,<vitesse> / angle_deg,distance_mm,intensite / #END
//
//  Bibliothèques : LD06 (ld06.h), MecanumCar_v2, AsyncTCP, ESPAsyncWebServer — Carte : ESP32 Dev Module
// ============================================================================

#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ld06.h"
#include "MecanumCar_v2.h"

// ============================================================
// INTERFACE WEB (page HTML servie sur http://192.168.4.1/)
// ============================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Robot ESP32</title>
<style>
:root{--bg:#0f1216;--card:#181d24;--line:#2a313b;--txt:#e6e9ee;--mut:#8b95a3;--acc:#3fa7ff;--ok:#3ecf8e;--bad:#ff5d5d;--warn:#f5b942}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--txt)}
header{position:sticky;top:0;z-index:5;display:flex;justify-content:space-between;align-items:center;padding:10px 14px;background:var(--bg);border-bottom:1px solid var(--line)}
h1{font-size:17px;margin:0}
h2{font-size:15px;margin:0 0 10px}
#conn{font-size:13px;color:var(--mut);display:flex;align-items:center;gap:6px}
#conn i{width:9px;height:9px;border-radius:50%;background:var(--bad);display:inline-block}
#conn.ok i{background:var(--ok)}
nav{display:flex;gap:6px;padding:10px 14px 0;max-width:760px;margin:auto}
nav button{flex:1}
nav button.on{background:var(--acc);color:#04121f;border-color:var(--acc)}
main{max-width:760px;margin:auto;padding:12px 14px 90px}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}
.hide{display:none}
button,select,input{font:inherit;color:var(--txt);background:#232a33;border:1px solid var(--line);border-radius:9px;padding:8px 11px}
button{cursor:pointer;user-select:none}
button:active{filter:brightness(1.3)}
input[type=number]{width:90px}
input[type=range]{padding:0;flex:1}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}
.mut{color:var(--mut);font-size:13px}
.go{background:var(--ok);color:#04140c;border-color:var(--ok);font-weight:600}
.danger{background:var(--bad);color:#fff;border-color:var(--bad);font-weight:600}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:320px;margin:6px auto;touch-action:none;user-select:none}
.pad button{aspect-ratio:1;font-size:30px;border-radius:14px;touch-action:none}
.pad button.active{background:var(--acc);color:#04121f}
.pad .empty{visibility:hidden}
#estop{position:fixed;left:50%;bottom:14px;transform:translateX(-50%);z-index:9;padding:14px 34px;font-size:17px;border-radius:30px;box-shadow:0 4px 18px #0008}
table{width:100%;border-collapse:collapse;font-size:14px}
td{padding:5px 4px;border-bottom:1px solid var(--line)}
td:last-child{text-align:right;white-space:nowrap}
tr.cur td{background:#1d3a55}
tr.done td{opacity:.45}
td input{width:80px;padding:4px 6px}
td button{padding:3px 8px;font-size:13px}
canvas{width:100%;aspect-ratio:1;background:#0b0e12;border:1px solid var(--line);border-radius:10px;touch-action:none;cursor:crosshair}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
label{font-size:13px;color:var(--mut);display:flex;flex-direction:column;gap:4px}
kbd{background:#232a33;border:1px solid var(--line);border-radius:4px;padding:0 5px;font-size:12px}
#lockBar{display:flex;align-items:center;gap:10px;max-width:760px;margin:10px auto 0;padding:0 14px}
#armBtn{flex:1;padding:12px;font-size:16px;font-weight:600;border-radius:12px;background:#3a2a12;border-color:var(--warn);color:var(--warn)}
#armBtn.armed{background:#0f2e20;border-color:var(--ok);color:var(--ok)}
body.locked .pad button[data-c]:not([data-c=STOP]){opacity:.35}
#lockMsg{color:var(--warn);font-size:13px;text-align:center;min-height:18px;margin:6px 0 0}
</style>
</head>
<body class="locked">
<header>
  <h1>🤖 Robot ESP32</h1>
  <div id="conn"><i></i><span>hors ligne</span></div>
</header>

<div id="lockBar">
  <button id="armBtn">🔒 Moteurs verrouillés — appuyer pour autoriser</button>
</div>
<p id="lockMsg"></p>

<nav>
  <button data-tab="pilot" class="on">Pilotage</button>
  <button data-tab="traj">Trajectoire</button>
  <button data-tab="draw">Tracé</button>
</nav>

<main>
<!-- ================= PILOTAGE ================= -->
<section id="pilot">
  <div class="card">
    <h2>Commande manuelle</h2>
    <div class="pad" id="pad">
      <button data-c="TL" title="Rotation gauche">↺</button>
      <button data-c="F"  title="Avancer">▲</button>
      <button data-c="TR" title="Rotation droite">↻</button>
      <button data-c="L"  title="Gauche (crabe)">◀</button>
      <button data-c="STOP" class="danger" title="Stop">■</button>
      <button data-c="R"  title="Droite (crabe)">▶</button>
      <button class="empty"></button>
      <button data-c="B"  title="Reculer">▼</button>
      <button class="empty"></button>
    </div>
    <div class="row">
      <label style="flex-direction:row;align-items:center;gap:6px;color:var(--txt)">
        <input type="checkbox" id="holdMode" checked> Maintenir appuyé pour rouler
      </label>
    </div>
    <div class="row" id="pulseRow">
      <span class="mut">Durée d'un appui :</span>
      <input type="range" id="pulse" min="100" max="2000" step="50" value="500">
      <span id="pulseV" class="mut">500 ms</span>
    </div>
    <p class="mut">Clavier : <kbd>Z</kbd><kbd>Q</kbd><kbd>S</kbd><kbd>D</kbd> ou flèches = déplacement, <kbd>A</kbd>/<kbd>E</kbd> = rotation, <kbd>Espace</kbd> = stop.</p>
  </div>
</section>

<!-- ================= TRAJECTOIRE ================= -->
<section id="traj" class="hide">
  <div class="card">
    <h2>Ajouter une étape</h2>
    <div class="row">
      <select id="stepCmd"></select>
      <input type="number" id="stepMs" value="1000" min="50" max="20000" step="50"> <span class="mut">ms</span>
      <button id="addStep">+ Ajouter</button>
    </div>
    <div class="row">
      <span class="mut">Modèles :</span>
      <select id="preset">
        <option value="">— choisir —</option>
        <option value="square">Carré</option>
        <option value="square_crab">Carré en crabe</option>
        <option value="back_forth">Aller-retour</option>
        <option value="slalom">Slalom</option>
        <option value="spin">Tour sur place</option>
      </select>
    </div>
  </div>

  <div class="card">
    <h2>Étapes <span class="mut" id="trajInfo"></span></h2>
    <table id="stepTable"></table>
    <p class="mut" id="emptyMsg">Aucune étape. Ajoutez-en ou dessinez un parcours dans l'onglet « Tracé ».</p>
    <div class="row">
      <span class="mut">Répétitions :</span>
      <input type="number" id="repeat" value="1" min="0" max="1000"> <span class="mut">(0 = en boucle)</span>
    </div>
    <div class="row">
      <button class="go" id="runTraj">▶ Lancer</button>
      <button class="danger" id="stopTraj">■ Arrêter</button>
      <button id="clearTraj">Vider</button>
    </div>
    <div class="row">
      <input id="saveName" placeholder="Nom du parcours" style="flex:1;min-width:120px">
      <button id="saveTraj">Enregistrer</button>
      <select id="savedList"><option value="">Parcours enregistrés…</option></select>
      <button id="delSaved" title="Supprimer le parcours sélectionné">🗑</button>
    </div>
    <p class="mut" id="trajMsg"></p>
  </div>
</section>

<!-- ================= TRACE ================= -->
<section id="draw" class="hide">
  <div class="card">
    <h2>Dessiner un parcours</h2>
    <p class="mut">Le robot part du centre, orienté vers le haut. Touchez la grille pour poser les points de passage.</p>
    <canvas id="cv"></canvas>
    <div class="row">
      <button id="undoPt">↶ Annuler</button>
      <button id="clearPts">Effacer</button>
      <label style="flex-direction:row;align-items:center;gap:6px;color:var(--txt)"><input type="checkbox" id="snap" checked> Aimanter</label>
      <span class="mut" id="ptsInfo"></span>
    </div>
    <div class="grid2">
      <label>Taille de la zone (m)<input type="number" id="area" value="4" min="1" max="20" step="0.5"></label>
      <label>Mode de déplacement
        <select id="drawMode">
          <option value="rot">Rotation puis avance</option>
          <option value="crab">Crabe (sans rotation)</option>
        </select>
      </label>
    </div>
    <div class="row">
      <button class="go" id="toSteps">Convertir en étapes →</button>
      <button id="runDraw">▶ Convertir et lancer</button>
    </div>
  </div>

  <div class="card">
    <h2>Calibration</h2>
    <p class="mut">Sans encodeurs, le robot avance « au temps » : mesurez ces vitesses sur votre sol pour que le tracé soit fidèle.</p>
    <div class="grid2">
      <label>Vitesse avance (mm/s)<input type="number" id="vLin" value="300" min="10" max="3000"></label>
      <label>Vitesse crabe (mm/s)<input type="number" id="vSide" value="250" min="10" max="3000"></label>
      <label>Vitesse rotation (°/s)<input type="number" id="vRot" value="180" min="5" max="1440"></label>
    </div>
    <div class="row">
      <button id="testLin">Test avance 1 s</button>
      <button id="testSide">Test crabe 1 s</button>
      <button id="testRot">Test rotation 1 s</button>
    </div>
    <p class="mut">Lancez un test, mesurez la distance (mm) ou l'angle (°) parcouru en 1 s et saisissez la valeur.</p>
  </div>
</section>
</main>

<button id="estop" class="danger">■ STOP</button>

<script>
const $ = s => document.querySelector(s);
const $$ = s => document.querySelectorAll(s);

const CMDS = {
  F:  {n:'Avancer',          u:'forward',     i:'▲'},
  B:  {n:'Reculer',          u:'backward',    i:'▼'},
  L:  {n:'Gauche (crabe)',   u:'left',        i:'◀'},
  R:  {n:'Droite (crabe)',   u:'right',       i:'▶'},
  TL: {n:'Rotation gauche',  u:'rotateleft',  i:'↺'},
  TR: {n:'Rotation droite',  u:'rotateright', i:'↻'},
  W:  {n:'Pause',            u:null,          i:'⏸'}
};
const MAX_STEP = 20000, MAX_STEPS = 100;

function api(path){
  return fetch(path, {cache:'no-store'}).then(r => r.ok ? r.text() : r.text().then(t => Promise.reject(t || r.status)));
}
const store = {
  get(k, d){ try{ const v = localStorage.getItem(k); return v ? JSON.parse(v) : d; }catch(e){ return d; } },
  set(k, v){ try{ localStorage.setItem(k, JSON.stringify(v)); }catch(e){} }
};

/* ---------- Onglets ---------- */
$$('nav button').forEach(b => b.onclick = () => {
  $$('nav button').forEach(x => x.classList.toggle('on', x === b));
  ['pilot','traj','draw'].forEach(id => $('#'+id).classList.toggle('hide', id !== b.dataset.tab));
  if (b.dataset.tab === 'draw') drawCanvas();
});

/* ---------- Arrêt d'urgence ---------- */
function stopAll(){ stopHold(false); api('/stop').catch(()=>{}); }
$('#estop').onclick = stopAll;

/* ---------- Verrou moteurs ---------- */
let armed = false, lockMsgTimer = null;
function setArmedUI(a){
  armed = a;
  document.body.classList.toggle('locked', !a);
  const b = $('#armBtn');
  b.classList.toggle('armed', a);
  b.textContent = a ? '🔓 Moteurs autorisés — appuyer pour verrouiller'
                    : '🔒 Moteurs verrouillés — appuyer pour autoriser';
  if (!a) stopHold(false);
}
function lockWarn(){
  $('#lockMsg').textContent = '🔒 Moteurs verrouillés : appuyez sur le bouton ci-dessus pour autoriser les mouvements.';
  clearTimeout(lockMsgTimer);
  lockMsgTimer = setTimeout(() => $('#lockMsg').textContent = '', 4000);
}
$('#armBtn').onclick = () => {
  api(armed ? '/disarm' : '/arm').then(() => setArmedUI(!armed)).catch(()=>{});
};

/* ================= PILOTAGE MANUEL ================= */
const HOLD_PULSE = 300;   // durée envoyée à chaque répétition (ms)
const HOLD_EVERY = 150;   // intervalle de répétition (ms)
let holdTimer = null, holdCmd = null, busy = false;

function sendMove(c, ms){
  // les routes /rotateleft et /rotateright divisent le temps par 2 côté ESP32
  const t = (c === 'TL' || c === 'TR') ? ms * 2 : ms;
  return api('/' + CMDS[c].u + '?time=' + Math.round(t));
}
function startHold(c){
  if (holdCmd === c) return;
  stopHold(false);
  holdCmd = c;
  $$('#pad button').forEach(b => b.classList.toggle('active', b.dataset.c === c));
  const tick = () => {
    if (busy) return;
    busy = true;
    sendMove(c, HOLD_PULSE).catch(()=>{}).finally(() => busy = false);
  };
  tick();
  holdTimer = setInterval(tick, HOLD_EVERY);
}
function stopHold(sendStop = true){
  if (holdTimer) clearInterval(holdTimer);
  const was = holdCmd;
  holdTimer = null; holdCmd = null;
  $$('#pad button.active').forEach(b => b.classList.remove('active'));
  if (sendStop && was) api('/stop').catch(()=>{});
}
function pressCmd(c){
  if (c === 'STOP') { stopAll(); return; }
  if (!armed) { lockWarn(); return; }
  if ($('#holdMode').checked) startHold(c);
  else sendMove(c, +$('#pulse').value).catch(()=>{});
}
function releaseCmd(){ if ($('#holdMode').checked) stopHold(true); }

$$('#pad button[data-c]').forEach(b => {
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); pressCmd(b.dataset.c); });
  ['pointerup','pointercancel','lostpointercapture'].forEach(ev => b.addEventListener(ev, releaseCmd));
  b.addEventListener('contextmenu', e => e.preventDefault());
});
window.addEventListener('blur', () => stopHold(true));
document.addEventListener('visibilitychange', () => { if (document.hidden) stopHold(true); });

const KEYS = {ArrowUp:'F',z:'F',w:'F',ArrowDown:'B',s:'B',ArrowLeft:'L',q:'L',ArrowRight:'R',d:'R',a:'TL',e:'TR'};
let keyDown = null;
document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  if (e.code === 'Space') { e.preventDefault(); stopAll(); return; }
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  const c = KEYS[k];
  if (!c || e.repeat) return;
  e.preventDefault();
  keyDown = k;
  pressCmd(c);
});
document.addEventListener('keyup', e => {
  const k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (k === keyDown) { keyDown = null; releaseCmd(); }
});

function updPulseUI(){
  $('#pulseV').textContent = $('#pulse').value + ' ms';
  $('#pulseRow').style.display = $('#holdMode').checked ? 'none' : 'flex';
}
$('#pulse').oninput = updPulseUI;
$('#holdMode').onchange = updPulseUI;
updPulseUI();

/* ================= TRAJECTOIRE ================= */
let steps = store.get('robot_steps', []);
let runningIdx = -1, runningActive = false;

Object.entries(CMDS).forEach(([k, v]) => {
  const o = document.createElement('option'); o.value = k; o.textContent = v.i + ' ' + v.n;
  $('#stepCmd').appendChild(o);
});

function saveSteps(){ store.set('robot_steps', steps); }
function renderSteps(){
  const t = $('#stepTable');
  t.innerHTML = '';
  let total = 0;
  steps.forEach((s, i) => {
    total += s.ms;
    const tr = document.createElement('tr');
    if (runningActive && i === runningIdx) tr.className = 'cur';
    else if (runningActive && i < runningIdx) tr.className = 'done';
    tr.innerHTML =
      '<td class="mut">' + (i+1) + '</td>' +
      '<td>' + CMDS[s.c].i + ' ' + CMDS[s.c].n + '</td>' +
      '<td><input type="number" min="50" max="' + MAX_STEP + '" step="50" value="' + s.ms + '"> ms</td>' +
      '<td><button data-a="up">↑</button> <button data-a="down">↓</button> <button data-a="del">✕</button></td>';
    tr.querySelector('input').onchange = e => {
      s.ms = Math.max(50, Math.min(MAX_STEP, Math.round(+e.target.value || 0)));
      e.target.value = s.ms; saveSteps(); renderInfo();
    };
    tr.querySelectorAll('button').forEach(b => b.onclick = () => {
      const a = b.dataset.a;
      if (a === 'del') steps.splice(i, 1);
      if (a === 'up' && i > 0) [steps[i-1], steps[i]] = [steps[i], steps[i-1]];
      if (a === 'down' && i < steps.length - 1) [steps[i+1], steps[i]] = [steps[i], steps[i+1]];
      saveSteps(); renderSteps();
    });
    t.appendChild(tr);
  });
  $('#emptyMsg').style.display = steps.length ? 'none' : 'block';
  renderInfo();
}
function renderInfo(){
  const total = steps.reduce((a, s) => a + s.ms, 0);
  $('#trajInfo').textContent = steps.length ? '(' + steps.length + ' étapes, ' + (total/1000).toFixed(1) + ' s)' : '';
}
function setMsg(t, err){ const m = $('#trajMsg'); m.textContent = t; m.style.color = err ? 'var(--bad)' : ''; }

$('#addStep').onclick = () => {
  if (steps.length >= MAX_STEPS) { setMsg('Maximum ' + MAX_STEPS + ' étapes.', true); return; }
  const ms = Math.max(50, Math.min(MAX_STEP, Math.round(+$('#stepMs').value || 0)));
  steps.push({c: $('#stepCmd').value, ms});
  saveSteps(); renderSteps();
};
$('#clearTraj').onclick = () => { steps = []; saveSteps(); renderSteps(); };

const PRESETS = {
  square:      () => { const a = []; for (let i=0;i<4;i++) a.push({c:'F',ms:1500},{c:'TR',ms:rotMs(90)}); return a; },
  square_crab: () => [{c:'F',ms:1500},{c:'R',ms:1500},{c:'B',ms:1500},{c:'L',ms:1500}],
  back_forth:  () => [{c:'F',ms:2000},{c:'W',ms:500},{c:'B',ms:2000}],
  slalom:      () => { const a = []; for (let i=0;i<3;i++) a.push({c:'F',ms:700},{c:'L',ms:600},{c:'F',ms:700},{c:'R',ms:600}); return a; },
  spin:        () => [{c:'TR',ms:rotMs(360)}]
};
$('#preset').onchange = e => {
  const p = PRESETS[e.target.value];
  if (p) { steps = p(); saveSteps(); renderSteps(); }
  e.target.value = '';
};

function encodeSteps(list){ return list.map(s => s.c + ':' + s.ms).join(','); }
function runSteps(list, repeat){
  if (!list.length) { setMsg('Aucune étape à exécuter.', true); return Promise.resolve(); }
  if (!armed) { lockWarn(); setMsg('Moteurs verrouillés : autorisez-les d\'abord.', true); return Promise.resolve(); }
  stopHold(false);
  const url = '/traj?repeat=' + repeat + '&seq=' + encodeURIComponent(encodeSteps(list));
  return api(url).then(() => setMsg('Trajectoire lancée.')).catch(e => {
    if (e === 'LOCKED') { setArmedUI(false); lockWarn(); setMsg('Moteurs verrouillés.', true); }
    else setMsg('Erreur : ' + e, true);
  });
}
$('#runTraj').onclick = () => runSteps(steps, Math.max(0, Math.round(+$('#repeat').value || 0)));
$('#stopTraj').onclick = () => { api('/traj/stop').then(() => setMsg('Arrêtée.')).catch(()=>{}); };

/* Parcours enregistrés */
function refreshSaved(){
  const all = store.get('robot_saved', {});
  const sel = $('#savedList');
  sel.innerHTML = '<option value="">Parcours enregistrés…</option>';
  Object.keys(all).sort().forEach(n => { const o = document.createElement('option'); o.value = n; o.textContent = n; sel.appendChild(o); });
}
$('#saveTraj').onclick = () => {
  const n = $('#saveName').value.trim();
  if (!n || !steps.length) { setMsg('Donnez un nom et au moins une étape.', true); return; }
  const all = store.get('robot_saved', {}); all[n] = steps; store.set('robot_saved', all);
  refreshSaved(); setMsg('« ' + n + ' » enregistré.');
};
$('#savedList').onchange = e => {
  const all = store.get('robot_saved', {});
  if (all[e.target.value]) { steps = JSON.parse(JSON.stringify(all[e.target.value])); $('#saveName').value = e.target.value; saveSteps(); renderSteps(); }
};
$('#delSaved').onclick = () => {
  const n = $('#savedList').value; if (!n) return;
  const all = store.get('robot_saved', {}); delete all[n]; store.set('robot_saved', all); refreshSaved();
};

/* ================= TRACE ================= */
const cv = $('#cv'), ctx = cv.getContext('2d');
let pts = store.get('robot_pts', []);   // points en mètres, origine = départ, +y = devant
const calib = store.get('robot_calib', {vLin:300, vSide:250, vRot:180});
$('#vLin').value = calib.vLin; $('#vSide').value = calib.vSide; $('#vRot').value = calib.vRot;
['vLin','vSide','vRot'].forEach(id => $('#'+id).onchange = () => {
  calib[id] = Math.max(1, +$('#'+id).value || 1); store.set('robot_calib', calib);
});
function rotMs(deg){ return Math.round(Math.abs(deg) / calib.vRot * 1000); }

function areaM(){ return Math.max(1, +$('#area').value || 4); }
function gridStep(){ const a = areaM(); return a <= 2 ? 0.25 : a <= 6 ? 0.5 : 1; }
function w2c(x, y){ const S = areaM(), W = cv.width; return [(x + S/2) / S * W, (S/2 - y) / S * W]; }
function c2w(px, py){ const S = areaM(), W = cv.width; return [px / W * S - S/2, S/2 - py / W * S]; }

function drawCanvas(){
  const dpr = window.devicePixelRatio || 1;
  const size = Math.round(cv.clientWidth * dpr);
  if (!size) return;
  if (cv.width !== size) { cv.width = size; cv.height = size; }
  const W = cv.width, S = areaM(), g = gridStep();
  ctx.clearRect(0, 0, W, W);
  // grille
  ctx.lineWidth = 1;
  for (let v = -S/2; v <= S/2 + 1e-6; v += g) {
    const major = Math.abs(v) < 1e-6;
    ctx.strokeStyle = major ? '#3a4452' : '#1d232b';
    const [x] = w2c(v, 0), [, y] = w2c(0, v);
    ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, W); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
  }
  ctx.fillStyle = '#8b95a3'; ctx.font = (11*dpr) + 'px system-ui';
  ctx.fillText('grille : ' + g + ' m', 6*dpr, W - 6*dpr);
  // chemin
  const all = [[0,0], ...pts.map(p => [p.x, p.y])];
  if (all.length > 1) {
    ctx.strokeStyle = '#3fa7ff'; ctx.lineWidth = 3*dpr; ctx.lineJoin = 'round';
    ctx.beginPath();
    all.forEach((p, i) => { const [x, y] = w2c(p[0], p[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke();
  }
  pts.forEach((p, i) => {
    const [x, y] = w2c(p.x, p.y);
    ctx.fillStyle = '#3fa7ff'; ctx.beginPath(); ctx.arc(x, y, 9*dpr, 0, 7); ctx.fill();
    ctx.fillStyle = '#04121f'; ctx.font = 'bold ' + (11*dpr) + 'px system-ui';
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(i+1, x, y);
  });
  ctx.textAlign = 'start'; ctx.textBaseline = 'alphabetic';
  // robot au départ (triangle vers le haut)
  const [rx, ry] = w2c(0, 0), r = 12*dpr;
  ctx.fillStyle = '#3ecf8e';
  ctx.beginPath(); ctx.moveTo(rx, ry - r); ctx.lineTo(rx - r*0.75, ry + r*0.7); ctx.lineTo(rx + r*0.75, ry + r*0.7); ctx.closePath(); ctx.fill();
  // info
  let d = 0; for (let i = 1; i < all.length; i++) d += Math.hypot(all[i][0]-all[i-1][0], all[i][1]-all[i-1][1]);
  $('#ptsInfo').textContent = pts.length + ' point(s), ' + d.toFixed(2) + ' m';
}
cv.addEventListener('pointerdown', e => {
  const rect = cv.getBoundingClientRect(), dpr = cv.width / rect.width;
  let [x, y] = c2w((e.clientX - rect.left) * dpr, (e.clientY - rect.top) * dpr);
  if ($('#snap').checked) { const g = gridStep() / 2; x = Math.round(x / g) * g; y = Math.round(y / g) * g; }
  x = +x.toFixed(3); y = +y.toFixed(3);
  const last = pts.length ? pts[pts.length-1] : {x:0, y:0};
  if (Math.hypot(x - last.x, y - last.y) < 0.02) return;
  pts.push({x, y}); store.set('robot_pts', pts); drawCanvas();
});
$('#undoPt').onclick = () => { pts.pop(); store.set('robot_pts', pts); drawCanvas(); };
$('#clearPts').onclick = () => { pts = []; store.set('robot_pts', pts); drawCanvas(); };
$('#area').onchange = drawCanvas;
window.addEventListener('resize', drawCanvas);

// découpe une étape trop longue en plusieurs
function pushStep(list, c, ms){
  ms = Math.round(ms);
  while (ms >= 30) { const m = Math.min(ms, MAX_STEP); list.push({c, ms: m}); ms -= m; }
}
function pathToSteps(){
  const out = [], mode = $('#drawMode').value;
  let x = 0, y = 0, h = 0;   // h = cap en degrés, 0 = vers le haut, + = sens horaire
  for (const p of pts) {
    const dx = p.x - x, dy = p.y - y, dist = Math.hypot(dx, dy);
    if (dist < 0.01) continue;
    if (mode === 'rot') {
      const target = Math.atan2(dx, dy) * 180 / Math.PI;
      let d = target - h;
      while (d > 180) d -= 360;
      while (d <= -180) d += 360;
      if (Math.abs(d) > 1) pushStep(out, d > 0 ? 'TR' : 'TL', rotMs(d));
      pushStep(out, 'F', dist * 1000 / calib.vLin * 1000);
      h = target;
    } else {
      // crabe : axe avant/arrière puis latéral, sans tourner
      if (Math.abs(dy) > 0.005) pushStep(out, dy > 0 ? 'F' : 'B', Math.abs(dy) * 1000 / calib.vLin * 1000);
      if (Math.abs(dx) > 0.005) pushStep(out, dx > 0 ? 'R' : 'L', Math.abs(dx) * 1000 / calib.vSide * 1000);
    }
    x = p.x; y = p.y;
  }
  return out;
}
function convert(){
  if (!pts.length) { alert('Posez au moins un point sur la grille.'); return null; }
  const s = pathToSteps();
  if (s.length > MAX_STEPS) { alert('Parcours trop long (' + s.length + ' étapes, max ' + MAX_STEPS + ').'); return null; }
  steps = s; saveSteps(); renderSteps();
  return s;
}
$('#toSteps').onclick = () => { if (convert()) $('nav button[data-tab="traj"]').click(); };
$('#runDraw').onclick = () => { const s = convert(); if (s) runSteps(s, 1); };
$('#testLin').onclick  = () => runSteps([{c:'F', ms:1000}], 1);
$('#testSide').onclick = () => runSteps([{c:'R', ms:1000}], 1);
$('#testRot').onclick  = () => runSteps([{c:'TR', ms:1000}], 1);

/* ================= ETAT DU ROBOT ================= */
let fails = 0;
function poll(){
  fetch('/status', {cache:'no-store'}).then(r => r.json()).then(st => {
    fails = 0;
    $('#conn').className = 'ok';
    if (st.armed !== armed) setArmedUI(st.armed);
    let txt = 'connecté';
    if (st.traj) txt = 'trajectoire ' + Math.min(st.idx+1, st.len) + '/' + st.len + (st.repeat !== 1 ? ' · tour ' + (st.loop+1) + (st.repeat ? '/' + st.repeat : '') : '');
    else if (st.moving) txt = 'en mouvement';
    $('#conn span').textContent = txt;
    const idx = st.traj ? st.idx : -1;
    if (idx !== runningIdx || st.traj !== runningActive) {
      runningIdx = idx; runningActive = st.traj; renderSteps();
    }
  }).catch(() => {
    if (++fails >= 2) { $('#conn').className = ''; $('#conn span').textContent = 'hors ligne'; }
  }).finally(() => setTimeout(poll, 500));
}

renderSteps(); refreshSaved(); drawCanvas(); poll();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// WIFI AP
// ============================================================
const char* ssid = "RobotESP32";
const char* password = "12345678";
AsyncWebServer server(80);

// ROBOT POSE
int16_t posex = 0;
int16_t posey = 0;
uint16_t posez = 0;

// ============================================================
// LIDAR LD06
// ============================================================
#define RX_LIDAR 33
#define TX_LIDAR -1 // Keep default/current UART TX assignment; leave it unconnected.
#define LD06_PWM_PIN 32
#define LIDAR_BAUD 230400
#define PC_BAUD 115200
// Mettre 0 pour travailler uniquement par câble USB : sans le WiFi, l'ESP32 consomme beaucoup moins
// (WiFi + moteur du lidar dépassent souvent ce que fournit un port USB : redémarrages « Brownout »).
#define USE_WIFI 1
LD06 lidar(Serial2, LD06_PWM_PIN);

// ============================================================
// OUTILS HTTP
// ============================================================
int getArg(AsyncWebServerRequest* req, const char* name, int def) {
  return req->hasParam(name) ? req->getParam(name)->value().toInt() : def;
}

void addCORS(AsyncWebServerResponse* resp) {
  resp->addHeader("Access-Control-Allow-Origin", "*");
  resp->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  resp->addHeader("Access-Control-Allow-Headers", "*");
}

void sendText(AsyncWebServerRequest* r, int code, const char* txt) {
  auto x = r->beginResponse(code, "text/plain", txt);
  addCORS(x); r->send(x);
}

// ============================================================
// MOTEURS
// ============================================================
bool moving = false;
uint32_t moveStarted = 0;
uint32_t moveDuration = 0;
SemaphoreHandle_t motorMutex;
const int MAX_MOVE_MS = 2000;

mecanumCar mecanumCar(19, 21);

// ------------------------------------------------------------
// VERROU DE SECURITE MOTEURS
// Au démarrage les moteurs sont VERROUILLES : toute commande de
// mouvement est refusée tant que vous n'avez pas déverrouillé
// depuis l'interface (/arm). Ils se reverrouillent automatiquement
// après AUTO_LOCK_MS sans aucun mouvement.
// ------------------------------------------------------------
volatile bool motorsArmed = false;
const uint32_t AUTO_LOCK_MS = 120000;   // 2 min d'inactivité -> verrouillage (0 = jamais)
uint32_t lastActivity = 0;

// Tant que le robot est au repos, on renvoie STOP régulièrement :
// si la carte moteur est alimentée après l'ESP32 (ou redémarre), elle
// reçoit quand même l'ordre d'arrêt au lieu de tourner toute seule.
const uint32_t IDLE_STOP_REFRESH_MS = 300;
uint32_t lastIdleStop = 0;

// ============================================================
// TRAJECTOIRE (exécutée sur l'ESP32, indépendante du WiFi)
// ============================================================
// Codes de commande d'une étape
enum : uint8_t { C_WAIT = 0, C_FWD, C_BACK, C_LEFT, C_RIGHT, C_ROTL, C_ROTR };

struct TrajStep { uint8_t cmd; uint16_t ms; };

// Déclaration explicite : empêche l'IDE Arduino de générer un prototype
// automatique placé avant la définition de TrajStep (erreur "TrajStep has not been declared").
int parseTrajectory(const String& seq, struct TrajStep* out);

const uint8_t  MAX_TRAJ_STEPS = 100;
const uint16_t MAX_STEP_MS    = 20000; // durée max d'une étape
const uint16_t STEP_GAP_MS    = 80;    // petit arrêt entre 2 étapes (évite les à-coups moteurs)

TrajStep traj[MAX_TRAJ_STEPS];
uint8_t  trajLen = 0;
uint8_t  trajIdx = 0;
uint16_t trajRepeat = 1;       // 0 = boucle infinie
uint16_t trajLoop = 0;         // tour en cours (0..)
bool     trajRunning = false;
bool     trajInGap = false;
uint32_t trajPhaseStart = 0;

// À appeler mutex pris
void applyCmd(uint8_t c) {
  switch (c) {
    case C_FWD:   mecanumCar.Advance();    break;
    case C_BACK:  mecanumCar.Back();       break;
    case C_LEFT:  mecanumCar.L_Move();     break;
    case C_RIGHT: mecanumCar.R_Move();     break;
    case C_ROTL:  mecanumCar.Turn_Left();  break;
    case C_ROTR:  mecanumCar.Turn_Right(); break;
    default:      mecanumCar.Stop();       break;
  }
}

int parseCmd(const String& s) {
  if (s == "F")  return C_FWD;
  if (s == "B")  return C_BACK;
  if (s == "L")  return C_LEFT;
  if (s == "R")  return C_RIGHT;
  if (s == "TL") return C_ROTL;
  if (s == "TR") return C_ROTR;
  if (s == "W")  return C_WAIT;
  return -1;
}

// Format : "F:1000,TR:450,W:300,L:800"
// Retourne le nombre d'étapes, ou -1 si erreur
int parseTrajectory(const String& seq, struct TrajStep* out) {
  int n = 0, start = 0;
  const int len = seq.length();
  while (start < len) {
    int end = seq.indexOf(',', start);
    if (end < 0) end = len;
    String tok = seq.substring(start, end);
    tok.trim();
    start = end + 1;
    if (tok.length() == 0) continue;
    const int colon = tok.indexOf(':');
    if (colon <= 0) return -1;
    String c = tok.substring(0, colon);
    c.toUpperCase();
    const int cmd = parseCmd(c);
    const long ms = tok.substring(colon + 1).toInt();
    if (cmd < 0 || ms <= 0) return -1;
    if (n >= MAX_TRAJ_STEPS) return -1;
    out[n].cmd = (uint8_t)cmd;
    out[n].ms  = (uint16_t)constrain(ms, 1L, (long)MAX_STEP_MS);
    n++;
  }
  return n;
}

// ============================================================
// MOUVEMENTS MANUELS
// ============================================================
void stopRobot() {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  trajRunning = false;          // un STOP annule aussi la trajectoire
  mecanumCar.Stop();
  moving = false;
  xSemaphoreGive(motorMutex);
}

void setArmed(bool on) {
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  motorsArmed = on;
  lastActivity = millis();
  if (!on) {                    // verrouiller = tout arrêter
    trajRunning = false;
    moving = false;
    mecanumCar.Stop();
  }
  xSemaphoreGive(motorMutex);
}

// Retourne false si le mouvement est refusé (moteurs verrouillés)
bool doMove(void (*fn)(), int t) {
  bool ok = true;
  xSemaphoreTake(motorMutex, portMAX_DELAY);
  trajRunning = false;          // une commande manuelle reprend la main
  if (!motorsArmed) {
    mecanumCar.Stop();
    moving = false;
    ok = false;
  } else if (t <= 0) {
    mecanumCar.Stop();
    moving = false;
  } else {
    moveDuration = (uint32_t)constrain(t, 1, MAX_MOVE_MS);
    moveStarted = millis();
    lastActivity = moveStarted;
    fn();
    moving = true;
  }
  xSemaphoreGive(motorMutex);
  return ok;
}

bool forward(int t)     { return doMove([](){ mecanumCar.Advance(); }, t); }
bool backward(int t)    { return doMove([](){ mecanumCar.Back(); }, t); }
bool left(int t)        { return doMove([](){ mecanumCar.L_Move(); }, t); }
bool right(int t)       { return doMove([](){ mecanumCar.R_Move(); }, t); }
bool rotateleft(int t)  { return doMove([](){ mecanumCar.Turn_Left(); }, t/2); }
bool rotateright(int t) { return doMove([](){ mecanumCar.Turn_Right(); }, t/2); }

void moveReply(AsyncWebServerRequest* r, bool ok) {
  if (ok) sendText(r, 200, "OK");
  else    sendText(r, 403, "LOCKED");   // moteurs verrouillés
}

// ============================================================
// SEND SCAN
// ============================================================
const uint16_t MAX_SCAN_POINTS = 460;
const size_t MAX_SCAN_BYTES = 8 + 5 * MAX_SCAN_POINTS;
uint8_t publishedScan[MAX_SCAN_BYTES];
size_t publishedSize = 0;
portMUX_TYPE scanMux = portMUX_INITIALIZER_UNLOCKED;

void publishScan() {
  const uint16_t n = lidar.getNbPointsInScan();
  if (n == 0 || n > MAX_SCAN_POINTS) return;
  uint8_t snapshot[MAX_SCAN_BYTES];
  snapshot[6] = n & 0xff;
  snapshot[7] = n >> 8;
  size_t off = 8;
  for (uint16_t i = 0; i < n; ++i) {
    const auto* pt = lidar.getPoints(i);
    const uint16_t angle = (uint16_t)(pt->angle * 100);
    snapshot[off++] = angle & 0xff;
    snapshot[off++] = angle >> 8;
    snapshot[off++] = pt->distance & 0xff;
    snapshot[off++] = pt->distance >> 8;
    snapshot[off++] = pt->intensity;
  }
  portENTER_CRITICAL(&scanMux);
  memcpy(publishedScan + 6, snapshot + 6, off - 6);
  publishedSize = off;
  portEXIT_CRITICAL(&scanMux);
}

void sendScanBinary(AsyncWebServerRequest* req) {
  uint8_t snapshot[MAX_SCAN_BYTES];
  size_t size;
  portENTER_CRITICAL(&scanMux);
  size = publishedSize;
  memcpy(snapshot, publishedScan, size);
  snapshot[0] = posex & 0xff; snapshot[1] = (uint16_t)posex >> 8;
  snapshot[2] = posey & 0xff; snapshot[3] = (uint16_t)posey >> 8;
  snapshot[4] = posez & 0xff; snapshot[5] = posez >> 8;
  portEXIT_CRITICAL(&scanMux);
  if (!size) {
    auto response = req->beginResponse(503, "text/plain", "NO_VALID_SCAN");
    addCORS(response); req->send(response);
    return;
  }
  auto response = req->beginResponseStream("application/octet-stream");
  response->write(snapshot, size);
  addCORS(response);
  req->send(response);
}

// ============================================================
// USB : commandes série (comme ESP32_LD06_Capture.ino)
// ============================================================
enum UsbOutMode { SER_IDLE, SER_CSV, SER_TELEPLOT };
UsbOutMode serialMode = SER_IDLE;
uint8_t serialToSend = 0;
uint32_t scanId = 0;
bool skipNext = false;   // après un envoi série (bloquant), le tour suivant peut être tronqué

void printInfo() {
  Serial.println(F("# ---------------------------------------------"));
  Serial.print(F("# WiFi : ")); Serial.print(ssid); Serial.print(F("  mot de passe : ")); Serial.println(password);
  Serial.print(F("# Adresse du robot : http://")); Serial.println(WiFi.softAPIP());
  Serial.println(F("# USB : s=1 scan | 1..9=N scans | c=flux CSV | t=Teleplot | x=stop"));
  Serial.println(F("# ---------------------------------------------"));
}

void sendScanCSV() {
  uint16_t n = lidar.getNbPointsInScan();
  Serial.print(F("#SCAN,")); Serial.print(scanId++); Serial.print(',');
  Serial.print(n); Serial.print(','); Serial.println(lidar.getSpeed());
  for (uint16_t i = 0; i < n; i++) {
    const auto* pt = lidar.getPoints(i);
    if (pt->distance == 0) continue;
    Serial.print(pt->angle, 2); Serial.print(',');
    Serial.print(pt->distance); Serial.print(',');
    Serial.println(pt->intensity);
  }
  Serial.println(F("#END"));
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's' || c == 'S')        serialToSend = 1;
    else if (c >= '1' && c <= '9')   serialToSend = c - '0';
    else if (c == 'c' || c == 'C')   serialMode = SER_CSV;
    else if (c == 't' || c == 'T')   serialMode = SER_TELEPLOT;
    else if (c == 'x' || c == 'X') { serialMode = SER_IDLE; serialToSend = 0; }
    else if (c == '?')               printInfo();
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  // --- MOTEURS A L'ARRET EN TOUT PREMIER ---
  // Stop immédiat (au cas où la carte moteur est déjà prête), puis Init()
  // qui attend 1 s le démarrage de la carte moteur et renvoie Stop.
  motorsArmed = false;
  mecanumCar.Stop();
  mecanumCar.Init();
  mecanumCar.Stop();

  Serial.begin(PC_BAUD);
  delay(200);

  motorMutex = xSemaphoreCreateMutex();
  if (!motorMutex) { while (true) { mecanumCar.Stop(); delay(1000); } }
  stopRobot();

  // LIDAR UART (tampon plus grand que dans le firmware du cours : l'envoi USB est bloquant)
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
  Serial2.setTimeout(2);
  lidar.init(); // Sets the Lidar PWM input on GPIO32 HIGH (no periodic PWM).

#if USE_WIFI
  // WIFI AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  delay(200);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // ---------- INTERFACE WEB ----------
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send_P(200, "text/html", INDEX_HTML);
  });

  // ---------- COMMANDES MANUELLES ----------
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r){
    stopRobot();
    sendText(r, 200, "STOPPED");
  });
  server.on("/forward", HTTP_GET, [](AsyncWebServerRequest* r){
    moveReply(r, forward(getArg(r,"time",300)));
  });
  server.on("/backward", HTTP_GET, [](AsyncWebServerRequest* r){
    moveReply(r, backward(getArg(r,"time",300)));
  });
  server.on("/left", HTTP_GET, [](AsyncWebServerRequest* r){
    moveReply(r, left(getArg(r,"time",300)));
  });
  server.on("/right", HTTP_GET, [](AsyncWebServerRequest* r){
    moveReply(r, right(getArg(r,"time",300)));
  });
  server.on("/rotateleft", HTTP_GET, [](AsyncWebServerRequest* r){
    moveReply(r, rotateleft(getArg(r,"time",300)));
  });
  server.on("/rotateright", HTTP_GET, [](AsyncWebServerRequest* r){
    moveReply(r, rotateright(getArg(r,"time",300)));
  });

  // ---------- VERROU MOTEURS ----------
  server.on("/arm", HTTP_GET, [](AsyncWebServerRequest* r){
    setArmed(true);
    sendText(r, 200, "ARMED");
  });
  server.on("/disarm", HTTP_GET, [](AsyncWebServerRequest* r){
    setArmed(false);
    sendText(r, 200, "DISARMED");
  });

  // ---------- TRAJECTOIRE ----------
  // /traj?seq=F:1000,TR:450,F:800&repeat=1   (repeat=0 -> boucle infinie)
  server.on("/traj", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("seq")) { sendText(r, 400, "MISSING_SEQ"); return; }
    static TrajStep tmp[MAX_TRAJ_STEPS];   // static : évite 400 octets sur la pile async
    const int n = parseTrajectory(r->getParam("seq")->value(), tmp);
    if (n <= 0) { sendText(r, 400, "BAD_SEQ"); return; }
    const int rep = constrain(getArg(r, "repeat", 1), 0, 1000);

    xSemaphoreTake(motorMutex, portMAX_DELAY);
    if (!motorsArmed) {
      xSemaphoreGive(motorMutex);
      sendText(r, 403, "LOCKED");
      return;
    }
    lastActivity = millis();
    memcpy(traj, tmp, sizeof(TrajStep) * n);
    trajLen = n;
    trajIdx = 0;
    trajRepeat = rep;
    trajLoop = 0;
    trajInGap = false;
    moving = false;
    trajPhaseStart = millis();
    applyCmd(traj[0].cmd);
    trajRunning = true;
    xSemaphoreGive(motorMutex);

    sendText(r, 200, "TRAJ_STARTED");
  });

  server.on("/traj/stop", HTTP_GET, [](AsyncWebServerRequest* r){
    stopRobot();
    sendText(r, 200, "TRAJ_STOPPED");
  });

  // Etat du robot (utilisé par l'interface pour suivre la progression)
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r){
    char buf[160];
    xSemaphoreTake(motorMutex, portMAX_DELAY);
    snprintf(buf, sizeof(buf),
      "{\"armed\":%s,\"traj\":%s,\"idx\":%u,\"len\":%u,\"loop\":%u,\"repeat\":%u,\"moving\":%s}",
      motorsArmed ? "true" : "false",
      trajRunning ? "true" : "false", trajIdx, trajLen, trajLoop, trajRepeat,
      (moving || (trajRunning && !trajInGap)) ? "true" : "false");
    xSemaphoreGive(motorMutex);
    auto x = r->beginResponse(200, "application/json", buf);
    addCORS(x); r->send(x);
  });

  // ---------- LIDAR / POSE ----------
  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest* r){
    sendScanBinary(r);
  });

  server.on("/pose", HTTP_GET, [](AsyncWebServerRequest* r){
    const int xPose = constrain(getArg(r,"x",0), -32768, 32767);
    const int yPose = constrain(getArg(r,"y",0), -32768, 32767);
    const int angle = getArg(r,"z",0);
    portENTER_CRITICAL(&scanMux);
    posex = xPose;
    posey = yPose;
    posez = ((angle % 36000) + 36000) % 36000;
    portEXIT_CRITICAL(&scanMux);
    sendText(r, 200, "OK");
  });

  // /ping avec CORS (sinon une page ouverte depuis le PC ne peut pas lire la réponse)
  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *r){
    sendText(r, 200, "OK");
  });

  server.begin();
#else
  Serial.println("WiFi desactive (USE_WIFI 0) : commandes USB seulement");
#endif
  printInfo();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  handleSerial();

  // Lire le lidar régulièrement ; chaque tour complet est publié pour le WiFi (et envoyé en USB si demandé)
  if (lidar.readScan()) {
    if (skipNext) {
      skipNext = false;
    } else {
      publishScan();
      bool usedSerial = false;
      if (serialToSend > 0 || serialMode == SER_CSV) {
        sendScanCSV();
        if (serialToSend > 0) serialToSend--;
        usedSerial = true;
      } else if (serialMode == SER_TELEPLOT) {
        lidar.printScanTeleplot(Serial);
        usedSerial = true;
      }
      if (usedSerial) {                       // l'envoi série à 115200 bauds est lent : buffer lidar vidé
        while (Serial2.available()) Serial2.read();
        skipNext = true;
      }
    }
  }

  // Laisser respirer la pile WiFi
  static uint32_t lastYield = 0;
  if (millis() - lastYield > 20) {
    lastYield = millis();
    yield();
  }

  xSemaphoreTake(motorMutex, portMAX_DELAY);
  const uint32_t now = millis();

  // Pendant une trajectoire, le robot est considéré comme actif
  if (trajRunning) lastActivity = now;

  // Verrouillage automatique après une période sans mouvement
  if (motorsArmed && AUTO_LOCK_MS > 0 && !moving && !trajRunning &&
      now - lastActivity >= AUTO_LOCK_MS) {
    motorsArmed = false;
    Serial.println("Moteurs reverrouilles (inactivite)");
  }

  // Sécurité : verrouillé => jamais de mouvement
  if (!motorsArmed && (trajRunning || moving)) {
    trajRunning = false;
    moving = false;
    mecanumCar.Stop();
    lastIdleStop = now;
  }

  if (trajRunning) {
    // --- Exécution de la trajectoire (machine à états) ---
    if (!trajInGap) {
      if (now - trajPhaseStart >= traj[trajIdx].ms) {
        mecanumCar.Stop();           // fin de l'étape -> courte pause
        trajInGap = true;
        trajPhaseStart = now;
      }
    } else if (now - trajPhaseStart >= STEP_GAP_MS) {
      trajInGap = false;
      trajIdx++;
      if (trajIdx >= trajLen) {
        trajLoop++;
        if (trajRepeat == 0 || trajLoop < trajRepeat) {
          trajIdx = 0;               // on recommence
        } else {
          trajRunning = false;       // trajectoire terminée
          trajIdx = trajLen;
          mecanumCar.Stop();
        }
      }
      if (trajRunning) {
        trajPhaseStart = now;
        applyCmd(traj[trajIdx].cmd);
      }
    }
  } else if (moving && (uint32_t)(now - moveStarted) >= moveDuration) {
    // --- Arrêt auto des mouvements manuels ---
    mecanumCar.Stop();
    moving = false;
    lastIdleStop = now;
  } else if (!moving && now - lastIdleStop >= IDLE_STOP_REFRESH_MS) {
    // --- Robot au repos : on maintient les moteurs à l'arrêt ---
    mecanumCar.Stop();
    lastIdleStop = now;
  }

  xSemaphoreGive(motorMutex);
}