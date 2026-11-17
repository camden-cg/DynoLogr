'use strict';

// ── STATE ──────────────────────────────────
const MAXPTS = 150;
let POLL_MS = 500, pollTimer = null, isPolling = true, sampleCount = 0;
const zeros   = {yaw:0, pitch:0, roll:0};
let rawVals   = {yaw:0, pitch:0, roll:0};
let trials = [], activeTrial = null;
let anChart = null, ceChart = null, cvChartA = null, cvChartB = null;
let editorData = [];
let mpuFiltered = true;  // mirrors ESP32's mpuRawMode (true = filtered)

const ALLCOLS = [
  {key:'ts',label:'Time',color:'#8a9bbf'},
  {key:'yaw',label:'Yaw',color:'#0891b2'},{key:'pitch',label:'Pitch',color:'#7c3aed'},{key:'roll',label:'Roll',color:'#d97706'},
  {key:'rpm',label:'RPM',color:'#2563eb'},{key:'velocity',label:'Velocity',color:'#059669'},
  {key:'accel',label:'Accel',color:'#0891b2'},{key:'distance',label:'Distance',color:'#7c3aed'},
  {key:'displacement',label:'Displacement',color:'#dc2626'},
  {key:'ticks',label:'Ticks',color:'#8a9bbf'},{key:'direction',label:'Direction',color:'#8a9bbf'},
  {key:'steering_deg',label:'Steer °',color:'#ea580c'},{key:'servo_angle',label:'Servo Raw',color:'#ea580c'},
  {key:'qw',label:'Qw',color:'#b8c5df'},{key:'qx',label:'Qx',color:'#b8c5df'},
  {key:'qy',label:'Qy',color:'#b8c5df'},{key:'qz',label:'Qz',color:'#b8c5df'},
];
let selCols = new Set(['ts','yaw','pitch','roll','rpm','velocity','accel','distance','displacement']);
const VCOLORS = {rpm:'#2563eb',velocity:'#059669',accel:'#0891b2',distance:'#7c3aed',displacement:'#dc2626',yaw:'#0891b2',pitch:'#7c3aed',roll:'#d97706',time:'#8a9bbf'};
const VRAW    = {rpm:'RPM',velocity:'Velocity (m/s)',accel:'Accel (m/s²)',distance:'Distance (m)',displacement:'Displacement (m)',yaw:'Yaw (°)',pitch:'Pitch (°)',roll:'Roll (°)',time:'Time (s)',ts_idx:'Sample #'};

// chart data buffers
const chartBufs = {
  rpm:[],velocity:[],accel:[],distance:[],displacement:[],
  yaw:[],pitch:[],roll:[]
};
const savedPlots = [];

// ── PAGE NAV ──────────────────────────────
const PAGE_LABELS = {live:'Live Dashboard',editor:'Chart Editor',analysis:'Analysis & Export'};
function showPage(name, btn){
  ['live','editor','analysis'].forEach(p =>
    document.getElementById('pg-'+p).classList.toggle('on', p===name));
  document.querySelectorAll('.nav-btn').forEach(b => b.classList.remove('on'));
  if(btn) btn.classList.add('on');
  document.getElementById('topbar-page').textContent = PAGE_LABELS[name]||name;
  if(name==='analysis') { populateTrialDDs(); }
}

function switchAnTab(tab, btn){
  document.querySelectorAll('.an-panel').forEach(p=>p.classList.remove('on'));
  document.getElementById('anp-'+tab).classList.add('on');
  document.querySelectorAll('.an-tab').forEach(b=>b.classList.remove('on'));
  btn.classList.add('on');
}

function togSec(id, hdr){
  const b = document.getElementById(id);
  const c = b.classList.toggle('hidden');
  hdr.classList.toggle('col', c);
}

// ── POLLING ───────────────────────────────
function getIP(){ return (document.getElementById('ipInput').value||'').trim(); }
function togglePolling(){
  isPolling = !isPolling;
  const btn = document.getElementById('toggleBtn');
  btn.textContent = isPolling ? 'Stop' : 'Start';
  btn.className   = isPolling ? 'btn sm p' : 'btn sm';
  if(isPolling) startPoll(); else clearTimeout(pollTimer);
}
function startPoll(){ clearTimeout(pollTimer); POLL_MS = parseInt(document.getElementById('pollInput').value)||500; doPoll(); }
async function doPoll(){
  if(!isPolling) return;
  const ip = getIP();
  if(ip){
    try{
      const r = await fetch(`http://${ip}/data`, {signal: AbortSignal.timeout(2000)});
      if(r.ok){ const d = await r.json(); ingest(d); } else setOnline(false);
    } catch{ setOnline(false); }
  } else setOnline(false);
  POLL_MS = parseInt(document.getElementById('pollInput').value)||500;
  pollTimer = setTimeout(doPoll, POLL_MS);
}
function setOnline(on){
  document.getElementById('statusDot').className = 'status-dot'+(on?' live':'');
  document.getElementById('statusText').textContent = on ? 'Connected' : 'Offline';
  document.getElementById('fi').textContent = 'ESP32: '+(getIP()||'—');
}

// ── DATA INGEST ────────────────────────────
function ingest(d){
  setOnline(true); sampleCount++;
  const now = performance.now()/1000, t = now.toFixed(2);
  document.getElementById('fs').textContent = 'Samples: '+sampleCount;
  document.getElementById('fl').textContent = 'Last: '+new Date().toLocaleTimeString();
  document.getElementById('fp').textContent = 'Poll: '+POLL_MS+'ms';

  rawVals.yaw   = d.yaw_deg??d.yaw??0;
  rawVals.pitch = d.pitch_deg??d.pitch??0;
  rawVals.roll  = d.roll_deg??d.roll??0;
  const yaw = rawVals.yaw-zeros.yaw, pitch = rawVals.pitch-zeros.pitch, roll = rawVals.roll-zeros.roll;

  const rpm  = d.rpm??0,  vel  = d.velocity_ms??d.velocity??0, acc  = d.accel_ms2??d.accel??0;
  const dist = d.distance_m??d.distance??0, disp = d.displacement_m??d.displacement??0;
  const ticks = d.ticks??0, dir = (d.direction??'FWD').toUpperCase();
  const qw = d.qw??1, qx = d.qx??0, qy = d.qy??0, qz = d.qz??0;
  const steerDeg = d.steering_deg??0, servoRaw = d.servo_angle??90;

  // sync MPU mode indicator from device
  if(d.mpu_filtered !== undefined) mpuFiltered = d.mpu_filtered;

  // kinetics cards
  document.getElementById('kv-rpm').innerHTML  = rpm.toFixed(0);
  document.getElementById('kv-vel').innerHTML  = vel.toFixed(3)+'<span class="k-unit">m/s</span>';
  document.getElementById('kv-acc').innerHTML  = acc.toFixed(3)+'<span class="k-unit">m/s²</span>';
  document.getElementById('kv-dist').innerHTML = dist.toFixed(3)+'<span class="k-unit">m</span>';
  document.getElementById('kv-disp').innerHTML = disp.toFixed(3)+'<span class="k-unit">m</span>';
  document.getElementById('kv-ticks').textContent = ticks;
  document.getElementById('kv-dir').textContent   = dir;
  document.getElementById('kv-steer').innerHTML   = steerDeg.toFixed(1)+'<span class="k-unit">°</span>';

  // gyro blocks
  document.getElementById('gv-yaw').textContent   = yaw.toFixed(1)+'°';
  document.getElementById('gv-pitch').textContent = pitch.toFixed(1)+'°';
  document.getElementById('gv-roll').textContent  = roll.toFixed(1)+'°';
  document.getElementById('gb-yaw').style.width   = Math.min(100,Math.max(0,(yaw+180)/3.6))+'%';
  document.getElementById('gb-pitch').style.width = Math.min(100,Math.max(0,(pitch+90)/1.8))+'%';
  document.getElementById('gb-roll').style.width  = Math.min(100,Math.max(0,(roll+90)/1.8))+'%';

  // YPR overlay
  document.getElementById('ypr-yaw').textContent   = yaw.toFixed(1)+'°';
  document.getElementById('ypr-pitch').textContent = pitch.toFixed(1)+'°';
  document.getElementById('ypr-roll').textContent  = roll.toFixed(1)+'°';

  // sidebar dp values
  document.getElementById('dp-rpm').textContent   = rpm.toFixed(0);
  document.getElementById('dp-vel').innerHTML     = vel.toFixed(3)+'<small style="font-size:9px;color:var(--tx3)"> m/s</small>';
  document.getElementById('dp-acc').innerHTML     = acc.toFixed(3)+'<small style="font-size:9px;color:var(--tx3)"> m/s²</small>';
  document.getElementById('dp-dist').innerHTML    = dist.toFixed(3)+'<small style="font-size:9px;color:var(--tx3)"> m</small>';
  document.getElementById('dp-disp').innerHTML    = disp.toFixed(3)+'<small style="font-size:9px;color:var(--tx3)"> m</small>';
  document.getElementById('dp-dir').textContent   = dir;
  document.getElementById('dp-ticks').textContent = ticks;
  document.getElementById('dp-yaw').innerHTML     = yaw.toFixed(1)+'<small style="font-size:9px;color:var(--tx3)">°</small>';
  document.getElementById('dp-pitch').innerHTML   = pitch.toFixed(1)+'<small style="font-size:9px;color:var(--tx3)">°</small>';
  document.getElementById('dp-roll').innerHTML    = roll.toFixed(1)+'<small style="font-size:9px;color:var(--tx3)">°</small>';
  document.getElementById('dp-steer').innerHTML   = steerDeg.toFixed(1)+'<small style="font-size:9px;color:var(--tx3)">°</small>';
  document.getElementById('dp-servo').innerHTML   = servoRaw+'<small style="font-size:9px;color:var(--tx3)">°</small>';
  document.getElementById('live-sub').textContent = `${rpm.toFixed(0)} RPM · ${vel.toFixed(2)} m/s · ${dir} · Steer ${steerDeg.toFixed(0)}°`;

  // steering gauge
  const steerPct = Math.min(Math.abs(steerDeg)/90*50, 50);
  document.getElementById('steer-bar-l').style.width = (steerDeg < 0 ? steerPct : 0)+'%';
  document.getElementById('steer-bar-r').style.width = (steerDeg > 0 ? steerPct : 0)+'%';

  // steering YPR chip
  document.getElementById('ypr-steer').textContent = steerDeg.toFixed(1)+'°';

  // update MPU mode button label
  const mpuBtn = document.getElementById('mpuModeBtn');
  if(mpuBtn) mpuBtn.textContent = mpuFiltered ? 'MPU: Filtered' : 'MPU: Raw';

  // chart buffers
  const pushBuf = (k, v) => { chartBufs[k].push({x:parseFloat(t),y:v}); if(chartBufs[k].length>MAXPTS) chartBufs[k].shift(); };
  pushBuf('rpm',rpm); pushBuf('velocity',vel); pushBuf('accel',acc);
  pushBuf('distance',dist); pushBuf('displacement',disp);
  pushBuf('yaw',yaw); pushBuf('pitch',pitch); pushBuf('roll',roll);

  const row = {ts:t,yaw,pitch,roll,rpm,velocity:vel,accel:acc,distance:dist,displacement:disp,ticks,direction:dir,steering_deg:steerDeg,servo_angle:servoRaw,qw,qx,qy,qz,time:parseFloat(t)};
  if(activeTrial) activeTrial.data.push(row);

  updateCar3D(qw,qx,qy,qz,vel,acc);
  updateSavedPlots();
  updateViewerStats();
}

// ── IMU ZERO ──────────────────────────────
function za(ax){ zeros[ax] = rawVals[ax]; }
function zeroAll(){ zeros.yaw=rawVals.yaw; zeros.pitch=rawVals.pitch; zeros.roll=rawVals.roll; }
async function resetDev(){
  const ip = getIP(); if(!ip) return;
  try{ await fetch(`http://${ip}/reset`,{method:'POST',signal:AbortSignal.timeout(2000)}); }catch{}
}

// ── MPU FILTER TOGGLE ─────────────────────
async function toggleMpuMode(){
  mpuFiltered = !mpuFiltered;
  const btn = document.getElementById('mpuModeBtn');
  if(btn){ btn.textContent = mpuFiltered ? 'MPU: Filtered' : 'MPU: Raw'; }
  const ip = getIP(); if(!ip) return;
  try{
    await fetch(`http://${ip}/mpu_mode`,{
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({filtered: mpuFiltered}),
      signal: AbortSignal.timeout(2000)
    });
  }catch{}
}

// ── TRIALS ────────────────────────────────
function trialAction(){
  if(!activeTrial){
    const name = document.getElementById('trialNameInput').value.trim() || ('Trial '+(trials.length+1));
    activeTrial = {id:Date.now(),name,data:[],startTime:Date.now(),endTime:null,selected:false};
    trials.push(activeTrial);
    setTBtn(true);
    document.getElementById('statusDot').className = 'status-dot rec';
    document.getElementById('recIndicator').classList.add('on');
  } else {
    activeTrial.endTime = Date.now(); activeTrial = null;
    setTBtn(false);
    document.getElementById('statusDot').className = 'status-dot live';
    document.getElementById('recIndicator').classList.remove('on');
  }
  renderTrials(); populateTrialDDs();
}

// mirror for analysis page
function trialAction2(){
  document.getElementById('trialNameInput').value = document.getElementById('trialNameInput2').value;
  trialAction();
  document.getElementById('trialNameInput2').value = '';
}

function setTBtn(rec){
  ['trialBtn','trialBtnPanel','trialBtnPanel2'].forEach(id=>{
    const b = document.getElementById(id); if(!b) return;
    b.textContent = rec ? 'End Trial' : 'Start Trial';
    if(id==='trialBtn') b.className = rec ? 'btn sm rec' : 'btn sm';
    else b.className = rec ? 'btn rec' : 'btn success';
  });
}
function renderTrials(){
  const list = document.getElementById('trialsList'), empty = document.getElementById('trialsEmpty');
  if(!trials.length){ empty.style.display=''; list.innerHTML=''; list.appendChild(empty); updDlSum(); return; }
  empty.style.display = 'none'; list.innerHTML = '';
  trials.forEach(t=>{
    const dur = t.endTime ? Math.round((t.endTime-t.startTime)/1000)+'s' : activeTrial===t ? 'rec…' : '—';
    const div = document.createElement('div');
    div.className = 'trow'+(t.selected?' sel':'');
    div.onclick = ()=>{ t.selected=!t.selected; renderTrials(); updDlSum(); };
    div.innerHTML = `<div class="tcheck">${t.selected?'✓':''}</div><div class="tname">${t.name}</div><div class="tmeta">${t.data.length}pts · ${dur}</div><span class="tbadge ${activeTrial===t?'rec':'done'}">${activeTrial===t?'REC':'done'}</span><button class="tdel2" onclick="event.stopPropagation();delTrial(${t.id})">✕</button>`;
    list.appendChild(div);
  });
  updDlSum();
}
function delTrial(id){
  if(activeTrial&&activeTrial.id===id){ activeTrial=null; setTBtn(false); }
  trials = trials.filter(t=>t.id!==id);
  renderTrials(); populateTrialDDs();
}
function selAll()  { trials.forEach(t=>t.selected=true);  renderTrials(); }
function selNone() { trials.forEach(t=>t.selected=false); renderTrials(); }
function updDlSum(){
  const s = trials.filter(t=>t.selected), pts = s.reduce((a,t)=>a+t.data.length,0);
  document.getElementById('dlSummary').textContent = s.length ? `${s.length} trial · ${pts} rows` : '';
}
function populateTrialDDs(){
  ['cva-trial','cvb-trial','ce-trial-sel'].forEach(id=>{
    const el = document.getElementById(id); if(!el) return;
    const cur = el.value;
    el.innerHTML = '<option value="">— none —</option>'+trials.map(t=>`<option value="${t.id}">${t.name} (${t.data.length}pts)</option>`).join('');
    el.value = cur;
  });
}

// ── EXPORT CSV ────────────────────────────
function buildColSel(){
  const wrap = document.getElementById('colSel');
  ALLCOLS.forEach(c=>{
    const tag = document.createElement('div');
    tag.className = 'coltag'+(selCols.has(c.key)?' on':'');
    tag.innerHTML = `<span class="coltag-dot" style="color:${c.color}"></span>${c.label}`;
    tag.onclick = ()=>{
      if(selCols.has(c.key)) selCols.delete(c.key); else selCols.add(c.key);
      tag.className = 'coltag'+(selCols.has(c.key)?' on':'');
      updDlSum();
    };
    wrap.appendChild(tag);
  });
}
function dlSelected(){
  const sel = trials.filter(t=>t.selected);
  if(!sel.length){ alert('No trials selected.'); return; }
  const cols = ALLCOLS.filter(c=>selCols.has(c.key));
  if(!cols.length){ alert('No columns selected.'); return; }
  const lines = ['trial,'+cols.map(c=>c.key).join(',')];
  sel.forEach(t=>t.data.forEach(r=>
    lines.push(`"${t.name}",`+cols.map(c=>r[c.key]??'').join(','))
  ));
  const blob = new Blob([lines.join('\n')], {type:'text/csv;charset=utf-8;'});
  const url  = URL.createObjectURL(blob);
  const a    = document.createElement('a');
  a.href=url; a.download=`kinecar_${new Date().toISOString().slice(0,19).replace(/:/g,'-')}.csv`;
  document.body.appendChild(a); a.click(); document.body.removeChild(a);
  setTimeout(()=>URL.revokeObjectURL(url),3000);
}

// ── CHART DEFAULTS ────────────────────────
function cDefaults(){
  return {
    responsive:true, maintainAspectRatio:false, animation:{duration:0},
    plugins:{legend:{display:false},tooltip:{
      backgroundColor:'rgba(255,255,255,.95)', titleColor:'#0f1c3a', bodyColor:'#4a5a80',
      borderColor:'rgba(100,140,220,.2)', borderWidth:1,
      titleFont:{family:"'DM Mono',monospace",size:10},
      bodyFont:{family:"'DM Mono',monospace",size:10},
      padding:8, cornerRadius:8
    }},
    scales:{
      x:{grid:{color:'rgba(100,140,220,.06)'},ticks:{color:'#8a9bbf',font:{family:"'DM Mono',monospace",size:9}},border:{color:'rgba(100,140,220,.12)'}},
      y:{grid:{color:'rgba(100,140,220,.06)'},ticks:{color:'#8a9bbf',font:{family:"'DM Mono',monospace",size:9}},border:{color:'rgba(100,140,220,.12)'}}
    }
  };
}

// ── LIVE PLOT MODAL ────────────────────────
let activePltKey = null;
function openPlt(key){ activePltKey=key; document.getElementById('pname').value=''; document.getElementById('pModal').classList.add('open'); }
function closePlt(){ document.getElementById('pModal').classList.remove('open'); activePltKey=null; }
function createPlt(){
  const y=document.getElementById('py').value, x=document.getElementById('px').value;
  const name=document.getElementById('pname').value||`${VRAW[y]||y} vs ${VRAW[x]||x}`;
  const id='plt_'+Date.now();
  savedPlots.push({id,name,x,y,chart:null});
  renderSavedPlots(); closePlt();
}
function renderSavedPlots(){
  const wrap = document.getElementById('ce-plot-list');
  wrap.innerHTML = '';
  savedPlots.forEach(p=>{
    const chip = document.createElement('div');
    chip.style.cssText = 'display:flex;align-items:center;gap:4px;background:var(--bg3);border:1px solid var(--bd2);border-radius:8px;padding:3px 8px;font-size:10px;color:var(--tx2)';
    chip.innerHTML = `<span style="font-family:var(--mono)">${p.name}</span><button onclick="removePlot('${p.id}')" style="background:none;border:none;cursor:pointer;color:var(--tx3);font-size:11px;padding:0 0 0 4px">✕</button>`;
    wrap.appendChild(chip);
  });
}
function removePlot(id){
  const idx = savedPlots.findIndex(p=>p.id===id);
  if(idx>=0){ if(savedPlots[idx].chart) savedPlots[idx].chart.destroy(); savedPlots.splice(idx,1); }
  renderSavedPlots();
}
function updateSavedPlots(){ /* updates happen via rebuildEditorChart */ }

// ── CHART EDITOR ──────────────────────────
function getTrialById(id){ return trials.find(t=>String(t.id)===String(id))||null; }
function initEditorData(){
  const tid = document.getElementById('ce-trial-sel')?.value||'';
  const t = getTrialById(tid);
  editorData = t && t.data.length ? t.data.map((r,i)=>({x:i,y:r.velocity??0,...r})) : Array.from({length:10},(_,i)=>({x:i,y:0}));
}
function addEditorRow(){
  editorData.push({x:editorData.length,y:0});
  rebuildEditorChart();
}
function clearEditorData(){
  editorData = []; rebuildEditorChart();
}
function applyFormula(){
  const f = document.getElementById('ce-formula').value.trim();
  const out = document.getElementById('ce-eq-out');
  if(!f){ out.style.display='none'; return; }
  try{
    const sample = editorData[0]||{};
    const keys = Object.keys(sample);
    const fn = new Function(...keys, 'return ('+f+')');
    const res = fn(...keys.map(k=>sample[k]));
    out.style.display=''; out.textContent = '= '+Number(res).toFixed(4);
  }catch{ out.style.display=''; out.textContent='error'; }
}
function rebuildEditorChart(){
  const type = document.getElementById('ce-type')?.value||'scatter';
  const xk   = document.getElementById('ce-x')?.value||'ts_idx';
  const f     = document.getElementById('ce-formula')?.value.trim();
  initEditorData();

  const pts = editorData.map((r,i)=>{
    let y = r[xk==='ts_idx'?'y':xk]??r.y??0;
    if(f){ try{ const keys=Object.keys(r); y=new Function(...keys,'return ('+f+')')(...keys.map(k=>r[k])); }catch{} }
    return {x:xk==='ts_idx'?i:(r[xk]??i), y};
  });

  if(ceChart){ ceChart.destroy(); ceChart=null; }
  const o = cDefaults(); o.plugins.legend.display=false;
  const col = '#2563eb';
  ceChart = new Chart(document.getElementById('ceChart').getContext('2d'),{
    type: type==='scatter'?'scatter':'line',
    data:{datasets:[{label:'data',data:pts,borderColor:col,backgroundColor:col+'22',borderWidth:2,pointRadius:type==='bar'?0:2,fill:type==='bar',tension:0.3,type:type}]},
    options:o
  });

  // rebuild table
  const tb = document.getElementById('editorTBody'); tb.innerHTML='';
  pts.forEach((p,i)=>{
    const tr=document.createElement('tr');
    tr.innerHTML=`<td style="color:var(--tx3)">${i+1}</td><td><input value="${Number(p.x).toFixed(3)}" oninput="editorData[${i}].x=parseFloat(this.value)||0;rebuildEditorChart()"></td><td><input value="${Number(p.y).toFixed(3)}" oninput="editorData[${i}].y=parseFloat(this.value)||0;rebuildEditorChart()"></td><td><button class="tdel" onclick="editorData.splice(${i},1);rebuildEditorChart()">✕</button></td>`;
    tb.appendChild(tr);
  });
}

// ── TREND FITTING ────────────────────────
function addTrend(ds,pts,type,col){
  if(pts.length<2)return;
  const xs=pts.map(p=>p.x), ys=pts.map(p=>p.y);
  const n=xs.length;
  let fn;
  if(type==='linear'){
    const sx=xs.reduce((a,v)=>a+v,0),sy=ys.reduce((a,v)=>a+v,0);
    const sxy=xs.reduce((a,v,i)=>a+v*ys[i],0),sxx=xs.reduce((a,v)=>a+v*v,0);
    const m=(n*sxy-sx*sy)/(n*sxx-sx*sx||1), b=(sy-m*sx)/n;
    fn=x=>m*x+b;
  } else if(type==='poly'){
    const xm=xs.reduce((a,v)=>a+v,0)/n, ym=ys.reduce((a,v)=>a+v,0)/n;
    const sxx=xs.reduce((a,v)=>a+(v-xm)**2,0)||1;
    const sxy=xs.reduce((a,v,i)=>a+(v-xm)*(ys[i]-ym),0);
    const sxxy=xs.reduce((a,v,i)=>a+(v-xm)**2*(ys[i]-ym),0);
    const sxxxx=xs.reduce((a,v)=>a+(v-xm)**4,0)||1;
    const b2=sxxy/sxxxx, b1=(sxy-b2*xs.reduce((a,v)=>a+(v-xm)**3,0))/sxx, b0=ym-b2*(xs.reduce((a,v)=>a+(v-xm)**2,0)/n);
    fn=x=>b0+b1*(x-xm)+b2*(x-xm)**2;
  } else if(type==='exp'){
    const lys=ys.map(y=>Math.log(Math.max(y,1e-9)));
    const sx=xs.reduce((a,v)=>a+v,0),slny=lys.reduce((a,v)=>a+v,0);
    const sxlny=xs.reduce((a,v,i)=>a+v*lys[i],0),sxx=xs.reduce((a,v)=>a+v*v,0);
    const b=(n*sxlny-sx*slny)/(n*sxx-sx*sx||1), a=Math.exp((slny-b*sx)/n);
    fn=x=>a*Math.exp(b*x);
  }
  if(!fn)return;
  const xmn=Math.min(...xs), xmx=Math.max(...xs), step=(xmx-xmn)/40||1;
  const tPts=[];
  for(let x=xmn;x<=xmx+step*.1;x+=step) tPts.push({x,y:fn(x)});
  ds.push({label:type+' fit',type:'line',data:tPts,borderColor:col,borderWidth:1.5,borderDash:[4,3],pointRadius:0,fill:false,tension:0});
}

// ── CHART VIEWER ─────────────────────────
function rebuildCVA(){
  const tid=document.getElementById('cva-trial')?.value||'';
  const yk=document.getElementById('cva-y')?.value||'velocity';
  const trend=document.getElementById('cva-trend')?.value||'none';
  const t=getTrialById(tid);
  if(cvChartA){cvChartA.destroy();cvChartA=null;}
  if(!t||!t.data.length) return;
  const col=VCOLORS[yk]||'#2563eb';
  const pts=t.data.map((r,i)=>({x:i,y:r[yk]??0}));
  const ds=[{label:t.name,type:'scatter',data:pts,borderColor:col,backgroundColor:col+'22',borderWidth:1.5,pointRadius:2,showLine:true,tension:0.2}];
  if(trend!=='none') addTrend(ds,pts,trend,col);
  const o=cDefaults(); o.plugins.legend.display=true;
  cvChartA=new Chart(document.getElementById('cv-chartA').getContext('2d'),{type:'scatter',data:{datasets:ds},options:o});
  updateViewerStats();
}
function rebuildCVB(){
  const tid=document.getElementById('cvb-trial')?.value||'';
  const yk=document.getElementById('cvb-y')?.value||'accel';
  const trend=document.getElementById('cvb-trend')?.value||'none';
  const t=getTrialById(tid);
  if(cvChartB){cvChartB.destroy();cvChartB=null;}
  if(!t||!t.data.length) return;
  const col=VCOLORS[yk]||'#2563eb';
  const pts=t.data.map((r,i)=>({x:i,y:r[yk]??0}));
  const ds=[{label:t.name,type:'scatter',data:pts,borderColor:col,backgroundColor:col+'22',borderWidth:1.5,pointRadius:2,showLine:true,tension:0.2}];
  if(trend!=='none') addTrend(ds,pts,trend,col);
  const o=cDefaults(); o.plugins.legend.display=true;
  cvChartB=new Chart(document.getElementById('cv-chartB').getContext('2d'),{type:'scatter',data:{datasets:ds},options:o});
  updateViewerStats();
}
function updateViewerStats(){
  const tA=getTrialById(document.getElementById('cva-trial')?.value||'');
  if(tA&&tA.data.length){const m=Math.max(...tA.data.map(r=>r.velocity??0));document.getElementById('cvs-vel').innerHTML=m.toFixed(3)+'<span class="cv-stat-unit">m/s</span>';document.getElementById('cvs-vel-sub').textContent=tA.name+' · peak';}
  else{document.getElementById('cvs-vel').innerHTML='—';document.getElementById('cvs-vel-sub').textContent='no trial';}
  const tB=getTrialById(document.getElementById('cvb-trial')?.value||'');
  if(tB&&tB.data.length){const m=Math.max(...tB.data.map(r=>Math.abs(r.accel??0)));document.getElementById('cvs-acc').innerHTML=m.toFixed(3)+'<span class="cv-stat-unit">m/s²</span>';document.getElementById('cvs-acc-sub').textContent=tB.name+' · peak';}
  else{document.getElementById('cvs-acc').innerHTML='—';document.getElementById('cvs-acc-sub').textContent='no trial';}
}

// ── ANALYSIS ─────────────────────────────
function plotAn(){
  const xk=document.getElementById('an-x').value,yk=document.getElementById('an-y').value;
  const showInd=document.getElementById('an-ind').checked,showAvg=document.getElementById('an-avg').checked;
  const showLin=document.getElementById('an-lin').checked,showPoly=document.getElementById('an-poly').checked;
  const sel=trials.filter(t=>t.selected&&t.data.length>0);
  if(!sel.length){alert('Select at least one trial first.');return;}
  document.getElementById('anWrap').style.display='';
  if(anChart){anChart.destroy();anChart=null;}
  const cols=['#2563eb','#0891b2','#059669','#7c3aed','#dc2626','#d97706'];
  const datasets=[];
  sel.forEach((t,ti)=>{
    const pts=t.data.map((p,i)=>({x:xk==='ts_idx'?i:xk==='time'?parseFloat(p.ts||i):(p[xk]??0),y:p[yk]??0}));
    const col=cols[ti%cols.length];
    if(showInd)datasets.push({label:t.name,type:'scatter',data:pts,borderColor:col,backgroundColor:col+'22',borderWidth:1.5,pointRadius:2,showLine:true,tension:0.2});
    if(showLin)addTrend(datasets,pts,'linear',col);
    if(showPoly)addTrend(datasets,pts,'poly',col);
  });
  if(showAvg&&sel.length>1){
    const all=sel.flatMap(t=>t.data.map((p,i)=>({x:xk==='ts_idx'?i:xk==='time'?parseFloat(p.ts||i):(p[xk]??0),y:p[yk]??0})));
    all.sort((a,b)=>a.x-b.x);
    const BINS=60,xmn=all[0].x,xmx=all[all.length-1].x,rng=xmx-xmn||1;
    const bkts=Array.from({length:BINS},()=>({sum:0,n:0}));
    all.forEach(p=>{const bi=Math.min(BINS-1,Math.floor(((p.x-xmn)/rng)*BINS));bkts[bi].sum+=p.y;bkts[bi].n++;});
    datasets.push({label:'Average',type:'line',data:bkts.map((b,i)=>b.n?{x:xmn+(i+.5)*rng/BINS,y:b.sum/b.n}:null).filter(Boolean),borderColor:'#0f1c3a',borderWidth:2.5,pointRadius:0,fill:false,tension:0.4});
  }
  const o=cDefaults(); o.plugins.legend.display=true;
  o.scales.x.title={display:true,text:VRAW[xk]||xk,color:'#8a9bbf',font:{size:10}};
  o.scales.y.title={display:true,text:VRAW[yk]||yk,color:'#8a9bbf',font:{size:10}};
  anChart=new Chart(document.getElementById('anChart').getContext('2d'),{type:'scatter',data:{datasets},options:o});
}
function clearAn(){ if(anChart){anChart.destroy();anChart=null;} document.getElementById('anWrap').style.display='none'; }

// ══════════════════════════════════════════
//  3D CAR  — with velocity & acceleration vectors
// ══════════════════════════════════════════
let c3s,c3cam,c3r,carGroup,lastQ={qw:1,qx:0,qy:0,qz:0},zeroQ=null;
let velArrow=null, accArrow=null, forceArrow=null;
let liveVel=0, liveAcc=0;

function q2t(qw,qx,qy,qz){ return new THREE.Quaternion(qx,qz,-qy,qw).normalize(); }
function za3d(){
  zeroQ = q2t(lastQ.qw,lastQ.qx,lastQ.qy,lastQ.qz).invert();
  if(carGroup){ carGroup.quaternion.set(0,0,0,1); r3d(); }
}

function makeArrow(color, length){
  const group = new THREE.Group();
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(0.03, 0.03, length, 8),
    new THREE.MeshLambertMaterial({color})
  );
  shaft.position.y = length/2;
  const head = new THREE.Mesh(
    new THREE.CylinderGeometry(0, 0.08, 0.22, 8),
    new THREE.MeshLambertMaterial({color})
  );
  head.position.y = length + 0.11;
  group.add(shaft); group.add(head);
  return group;
}

function init3D(){
  const cv=document.getElementById('car3d'), wp=document.getElementById('car3dWrap');
  if(c3r){ c3r.dispose(); c3r=null; }
  const w=wp.clientWidth||600, h=wp.clientHeight||300;

  c3s  = new THREE.Scene();
  c3cam= new THREE.PerspectiveCamera(38,w/h,.1,100);
  c3cam.position.set(4.5,2.8,5.2);
  c3cam.lookAt(0,.4,0);

  c3r  = new THREE.WebGLRenderer({canvas:cv,alpha:true,antialias:true});
  c3r.setPixelRatio(Math.min(devicePixelRatio,2));
  c3r.setSize(w,h);
  c3r.setClearColor(0x000000,0);

  // lighting
  c3s.add(new THREE.AmbientLight(0xdbeafe,.7));
  const key=new THREE.DirectionalLight(0xffffff,1.0); key.position.set(5,8,6); c3s.add(key);
  const fill=new THREE.DirectionalLight(0xbfdbfe,.4); fill.position.set(-4,2,-5); c3s.add(fill);
  const rim=new THREE.DirectionalLight(0xffffff,.2);  rim.position.set(0,-3,4);  c3s.add(rim);

  // subtle grid
  const gc=0xe4ecf8;
  const grid=new THREE.GridHelper(12,24,gc,gc);
  grid.material.opacity=.7; grid.material.transparent=true;
  c3s.add(grid);

  buildCar();

  // build vector arrows (parented to scene so they stay world-oriented)
  velArrow   = makeArrow(0x2563eb, 1.0); c3s.add(velArrow);
  accArrow   = makeArrow(0xdc2626, 0.8); c3s.add(accArrow);
  forceArrow = makeArrow(0x059669, 0.9); c3s.add(forceArrow);
  // position above car
  velArrow.position.set(0, 1.8, 0);
  accArrow.position.set(0, 1.8, 0);
  forceArrow.position.set(0, 1.8, 0);
  updateVectors(0, 0);

  new ResizeObserver(()=>{
    if(!c3r||!c3cam)return;
    const w2=wp.clientWidth||600,h2=wp.clientHeight||300;
    c3cam.aspect=w2/h2; c3cam.updateProjectionMatrix();
    c3r.setSize(w2,h2); r3d();
  }).observe(wp);
}

function updateVectors(vel, acc){
  if(!velArrow) return;
  const maxScale = 2.0;
  const velScale = Math.min(Math.abs(vel)/5.0 * maxScale, maxScale);
  const accScale = Math.min(Math.abs(acc)/5.0 * maxScale, maxScale);

  velArrow.scale.set(1, Math.max(0.05, velScale), 1);
  accArrow.scale.set(1, Math.max(0.05, accScale), 1);

  // velocity points forward (+Z), accel points up if positive
  velArrow.rotation.x = vel < 0 ? Math.PI : 0;
  velArrow.rotation.z = Math.PI/2;  // point along X (car forward)

  accArrow.rotation.x = acc < 0 ? Math.PI : 0;

  // combined force vector (diagonal)
  const fx = vel, fy = acc;
  const mag = Math.sqrt(fx*fx+fy*fy) || 0.01;
  forceArrow.scale.set(1, Math.min(mag/5.0 * maxScale, maxScale)*0.8 || 0.05, 1);
  forceArrow.rotation.z = -Math.atan2(fx, fy);
  forceArrow.position.set(0.8, 1.8, 0);
  velArrow.position.set(-0.8, 1.8, 0);
  accArrow.position.set(0, 1.8, 0);

  r3d();
}

function buildCar() {
  if (!c3s) return;
  if (carGroup) { c3s.remove(carGroup); carGroup = null; }
  carGroup = new THREE.Group();

  const mk = (geo, mat, x, y, z) => {
    const m = new THREE.Mesh(geo, mat);
    m.position.set(x, y, z);
    carGroup.add(m);
    return m;
  };

  // ── MATERIALS ───────────────────────────────────────────────

  const H2  = new THREE.MeshLambertMaterial({ color: 0xBAD8FD });
  const H4  = new THREE.MeshLambertMaterial({ color: 0x60A5FA });
  const H6  = new THREE.MeshLambertMaterial({ color: 0x0874F7 });
  const H7  = new THREE.MeshLambertMaterial({ color: 0x065FCB });
  const H8  = new THREE.MeshLambertMaterial({ color: 0x054A9E });
  const H10 = new THREE.MeshLambertMaterial({ color: 0x022045 });

  const glassM = new THREE.MeshLambertMaterial({ color: 0xBAD8FD, transparent: true, opacity: 0.85 });
  const lightF = new THREE.MeshLambertMaterial({ color: 0xfef9c3, emissive: 0xfef9c3, emissiveIntensity: 0.6 });
  const lightR = new THREE.MeshLambertMaterial({ color: 0xfca5a5, emissive: 0xfca5a5, emissiveIntensity: 0.5 });

  const wheelM = new THREE.MeshLambertMaterial({ color: 0x348DF9 });
  const rimM   = new THREE.MeshLambertMaterial({ color: 0x0874F7 });
  const hubM   = new THREE.MeshLambertMaterial({ color: 0x022045 });

  // ── CHASSIS ─────────────────────────────────────────────────

  mk(new THREE.BoxGeometry(2.70, 0.13, 1.10), H10,  0,     0.065,  0);  // underbody plate
  mk(new THREE.BoxGeometry(2.16, 0.34, 1.05), H7,   0,     0.310,  0);  // main body slab
  mk(new THREE.BoxGeometry(0.14, 0.34, 1.05), H7,   1.15,  0.310,  0);  // front face
  mk(new THREE.BoxGeometry(0.14, 0.34, 1.05), H6,  -1.15,  0.310,  0);  // rear face

  // ── SIDE SKIRTS ─────────────────────────────────────────────

  mk(new THREE.BoxGeometry(2.10, 0.09, 0.03), H10,  0, 0.195,  0.555);  // skirt L
  mk(new THREE.BoxGeometry(2.10, 0.09, 0.03), H10,  0, 0.195, -0.555);  // skirt R

  // ── CABIN ───────────────────────────────────────────────────
  // Body slab top: y = 0.480  (center 0.310 + half 0.170)
  // Lower cabin sits flush on body: center y = 0.480 + 0.130 = 0.610

  // Beltline — bridges body slab to greenhouse, full width
  mk(new THREE.BoxGeometry(1.02, 0.26, 0.92), H8,  -0.08, 0.72, 0);   // beltline box
  // Beltline top: 0.610 + 0.130 = 0.740

  // Roof cap
  mk(new THREE.BoxGeometry(1.02, 0.04, 0.92), H10, -0.08, 0.87, 0);   // roof cap

  // ── GLASS (CONNECTED TO CABIN) ───────────────────────────────

  // vertical span: 0.740 → 0.800
  // center: 0.770, height: 0.06

  mk(new THREE.BoxGeometry(0.05, 0.25, 0.82), glassM,  0.48, 0.670,  0.00); // windshield
  mk(new THREE.BoxGeometry(0.05, 0.25, 0.80), glassM, -0.55, 0.670,  0.00); // rear screen

  mk(new THREE.BoxGeometry(0.76, 0.25, 0.04), glassM, -0.075, 0.670,  0.48); // side glass L
  mk(new THREE.BoxGeometry(0.76, 0.25, 0.04), glassM, -0.075, 0.670, -0.48); // side glass R

  // ── FRONT END ───────────────────────────────────────────────

  mk(new THREE.BoxGeometry(0.055, 0.11, 1.03), H10,  1.24,  0.19,  0);    // front splitter
  mk(new THREE.BoxGeometry(0.13,  0.09, 0.27), H10,  1.185, 0.27,  0.33); // intake vent L
  mk(new THREE.BoxGeometry(0.13,  0.09, 0.27), H6,   1.185, 0.27, -0.33); // intake vent R
  mk(new THREE.BoxGeometry(0.075, 0.095, 0.22), lightF, 1.19, 0.40,  0.36); // headlight L
  mk(new THREE.BoxGeometry(0.075, 0.095, 0.22), lightF, 1.19, 0.40, -0.36); // headlight R

  // ── REAR WING ───────────────────────────────────────────────

  mk(new THREE.BoxGeometry(0.38,  0.044, 0.96), H4,  -0.82,  0.90,  0);   // blade
  mk(new THREE.BoxGeometry(0.055, 0.28,  0.055), H8, -0.82,  0.76,  0.36); // strut L
  mk(new THREE.BoxGeometry(0.055, 0.28,  0.055), H8, -0.82,  0.76, -0.36); // strut R
  mk(new THREE.BoxGeometry(0.055, 0.055, 0.14),  H8, -0.82,  0.65,  0.36); // strut foot L
  mk(new THREE.BoxGeometry(0.055, 0.055, 0.14),  H8, -0.82,  0.65, -0.36); // strut foot R

  // ── REAR END ────────────────────────────────────────────────

  mk(new THREE.BoxGeometry(0.075, 0.095, 0.32), lightR, -1.185, 0.40,  0.33); // taillight L
  mk(new THREE.BoxGeometry(0.075, 0.095, 0.32), lightR, -1.185, 0.40, -0.33); // taillight R

  // ── WHEELS ──────────────────────────────────────────────────

  const tyreG = new THREE.CylinderGeometry(0.26,  0.26,  0.15, 20);
  const rimG  = new THREE.CylinderGeometry(0.16,  0.16,  0.17, 12);
  const hubG  = new THREE.CylinderGeometry(0.055, 0.055, 0.19,  8);

  [
    [ 0.82, 0.26,  0.60],  // front L
    [ 0.82, 0.26, -0.60],  // front R
    [-0.82, 0.26,  0.60],  // rear  L
    [-0.82, 0.26, -0.60],  // rear  R
  ].forEach(([x, y, z]) => {
    const tyre = new THREE.Mesh(tyreG, wheelM); tyre.rotation.x = Math.PI / 2; tyre.position.set(x, y, z); carGroup.add(tyre);
    const rim  = new THREE.Mesh(rimG,  rimM);   rim.rotation.x  = Math.PI / 2; rim.position.set(x, y, z);  carGroup.add(rim);
    const hub  = new THREE.Mesh(hubG,  hubM);   hub.rotation.x  = Math.PI / 2; hub.position.set(x, y, z);  carGroup.add(hub);
  });

  // ── ORIENTATION & RENDER ────────────────────────────────────

  const q = q2t(lastQ.qw, lastQ.qx, lastQ.qy, lastQ.qz);
  if (zeroQ) q.premultiply(zeroQ);
  carGroup.quaternion.copy(q);
  c3s.add(carGroup);
  r3d();
}

function updateCar3D(qw,qx,qy,qz,vel,acc){
  lastQ={qw,qx,qy,qz};
  liveVel = vel||0; liveAcc = acc||0;
  if(!carGroup)return;
  const q=q2t(qw,qx,qy,qz);
  if(zeroQ) q.premultiply(zeroQ);
  carGroup.quaternion.copy(q);
  updateVectors(liveVel, liveAcc);
  r3d();
}
function r3d(){ if(c3r&&c3s&&c3cam) c3r.render(c3s,c3cam); }

// ── INIT ──────────────────────────────────
buildColSel();
renderTrials();
populateTrialDDs();
initEditorData();
rebuildEditorChart();
requestAnimationFrame(()=>{ zeroQ=new THREE.Quaternion(); init3D(); });
startPoll();
document.addEventListener('keydown',e=>{ if(e.key==='Escape') closePlt(); });
document.getElementById('trialNameInput').addEventListener('keydown',e=>{ if(e.key==='Enter') trialAction(); });
document.getElementById('pModal').addEventListener('click',e=>{ if(e.target===e.currentTarget) closePlt(); });
