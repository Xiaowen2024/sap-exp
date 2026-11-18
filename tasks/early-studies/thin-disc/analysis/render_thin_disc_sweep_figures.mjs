#!/usr/bin/env node
/* Build self-contained SVG summaries from the recorded native thin-disc sweeps.
 * No simulator is re-run here; this is a visualization of the checked-in CSVs. */
import fs from 'node:fs';

const root = new URL('..', import.meta.url).pathname;
const readCsv = (path) => {
  const [header, ...rows] = fs.readFileSync(path, 'utf8').trim().split(/\r?\n/);
  const keys = header.split(',');
  return rows.map((line) => Object.fromEntries(line.split(',').map((v, i) => [keys[i], v])));
};
const n = (v) => Number(v);
const bool = (v) => v === true || v === 'True' || v === 'true';
const esc = (s) => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;');
const fmt = (v, digits = 2) => Number(v).toFixed(digits).replace(/\.0+$/, '');
const status = (r) => bool(r.failure) ? 'fail' : n(r.settling_time_s) >= 1.999 ? 'slow' : 'settled';
const colors = { fail: '#d85757', slow: '#e6b655', settled: '#42a86b', mj: '#427bc7', sap: '#8854c7', ink: '#15243b', grid: '#d7dde7', muted: '#5c6c80' };
function svgStart(w, h, title) {
  return `<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}" role="img" aria-labelledby="title desc"><title id="title">${esc(title)}</title><desc id="desc">Native parameter sweep summary for the three thin discs gravity-settling benchmark.</desc><style>text{font-family:Arial,Helvetica,sans-serif;fill:${colors.ink}}.title{font-weight:700;font-size:24px}.subtitle{font-size:13px;fill:${colors.muted}}.panel{fill:#fff;stroke:${colors.grid};stroke-width:1}.pt{font-size:14px;font-weight:700}.label{font-size:11px;fill:${colors.muted}}.tick{font-size:10px;fill:${colors.muted}}.note{font-size:11px;fill:${colors.muted}}.value{font-size:10px;font-weight:600}.legend{font-size:11px}</style><rect width="100%" height="100%" fill="#f8fafc"/><text class="title" x="36" y="38">${esc(title)}</text>`;
}
function panel(x, y, w, h, title, note) {
  return `<rect class="panel" x="${x}" y="${y}" width="${w}" height="${h}" rx="8"/><text class="pt" x="${x + 16}" y="${y + 25}">${esc(title)}</text>${note ? `<text class="note" x="${x + 16}" y="${y + 42}">${esc(note)}</text>` : ''}`;
}
function linePanel({x,y,w,h,title,note, series, xvals, yMax, yLabel, xLabel, logY=false}) {
  let out = panel(x,y,w,h,title,note);
  const L=x+52,R=x+w-18,T=y+58,B=y+h-38;
  const yy = (v) => {
    if (logY) { const lo=-4, hi=Math.log10(yMax); return B-(Math.log10(Math.max(v,1e-4))-lo)/(hi-lo)*(B-T); }
    return B - Math.min(v,yMax)/yMax*(B-T);
  };
  const xx = (v) => L + xvals.indexOf(v) / Math.max(1,xvals.length-1) * (R-L);
  for(let i=0;i<=4;i++) { const py=T+(B-T)*i/4; out+=`<line x1="${L}" x2="${R}" y1="${py}" y2="${py}" stroke="${colors.grid}"/><text class="tick" text-anchor="end" x="${L-5}" y="${py+3}">${logY ? `1e${Math.round(Math.log10(yMax)-(Math.log10(yMax)+4)*i/4)}` : fmt(yMax*(1-i/4),1)}</text>`; }
  out += `<line x1="${L}" x2="${R}" y1="${B}" y2="${B}" stroke="${colors.ink}"/><line x1="${L}" x2="${L}" y1="${T}" y2="${B}" stroke="${colors.ink}"/>`;
  xvals.forEach(v=>out+=`<text class="tick" text-anchor="middle" x="${xx(v)}" y="${B+15}">${esc(v)}</text>`);
  out += `<text class="label" text-anchor="middle" x="${(L+R)/2}" y="${y+h-8}">${esc(xLabel)}</text><text class="label" transform="translate(${x+14} ${(T+B)/2}) rotate(-90)" text-anchor="middle">${esc(yLabel)}</text>`;
  series.forEach((s,si)=> { const pts=s.values.filter(p=>Number.isFinite(p.y)).map(p=>`${xx(p.x)},${yy(p.y)}`).join(' '); out+=`<polyline points="${pts}" fill="none" stroke="${s.color}" stroke-width="2.4"/>`; s.values.forEach(p=>{if(Number.isFinite(p.y))out+=`<circle cx="${xx(p.x)}" cy="${yy(p.y)}" r="3" fill="${s.color}"/>`;}); out+=`<text class="legend" x="${R-110}" y="${T+14+si*15}" fill="${s.color}">● ${esc(s.name)}</text>`;});
  return out;
}
function heatmap({x,y,w,h,title,note, rows, cols, get, label}) {
  let out=panel(x,y,w,h,title,note); const L=x+85,R=x+w-16,T=y+62,B=y+h-35, cw=(R-L)/cols.length,ch=(B-T)/rows.length;
  cols.forEach((c,i)=>out+=`<text class="tick" text-anchor="middle" x="${L+(i+.5)*cw}" y="${T-8}">${esc(c)}</text>`);
  rows.forEach((r,j)=>{out+=`<text class="tick" text-anchor="end" x="${L-6}" y="${T+(j+.57)*ch}">${esc(r)}</text>`;cols.forEach((c,i)=>{const v=get(r,c);const fill=colors[v]||'#eef1f5';out+=`<rect x="${L+i*cw+1}" y="${T+j*ch+1}" width="${cw-2}" height="${ch-2}" rx="2" fill="${fill}"/>`; if(v)out+=`<text class="value" text-anchor="middle" x="${L+(i+.5)*cw}" y="${T+(j+.57)*ch}" fill="#fff">${v==='settled'?'✓':v==='slow'?'~':'×'}</text>`;});});
  out+=`<text class="label" text-anchor="middle" x="${(L+R)/2}" y="${y+h-8}">${esc(label.x)}</text><text class="label" transform="translate(${x+16} ${(T+B)/2}) rotate(-90)" text-anchor="middle">${esc(label.y)}</text>`;
  [['settled','settled < 2 s'],['slow','correct order, still moving'],['fail','wrong order / escape']].forEach(([k,t],i)=>out+=`<rect x="${x+14+i*128}" y="${y+48}" width="10" height="10" fill="${colors[k]}"/><text class="legend" x="${x+28+i*128}" y="${y+57}">${esc(t)}</text>`);
  return out;
}
function write(name, data) { fs.writeFileSync(`${root}/docs/assets/${name}`, data); }

const mj = readCsv(`${root}/exp-results/thin-discs/mujoco-comprehensive-sweep/results.csv`);
const primary = mj.filter(r=>r.sweep_group==='primary_dt_x_solref');
const damp = mj.filter(r=>r.sweep_group==='dampratio');
const imp = mj.filter(r=>r.sweep_group==='solimp_dmax');
const integ = mj.filter(r=>r.sweep_group==='integrator_x_solver');
const dts=[0.1,0.25,0.5,1,2,5], refs=[0.2,0.5,1,2,4,8,16];
let out=svgStart(1500,940,'MuJoCo — thin-disc native parameter sweep');
out+=`<text class="subtitle" x="36" y="60">Three 202-mm × 4-mm analytic discs, 2-s gravity settling. Green = correct order and settled; amber = correct order but residual motion at 2 s; red = failure.</text>`;
out+=heatmap({x:30,y:82,w:700,h:390,title:'A. Contact time scale × timestep',note:'Primary sweep. solref time constant primarily controls compliance.',rows:refs.map(String),cols:dts.map(String),get:(r,c)=>{const q=primary.find(a=>n(a.solref_timeconst_ms)===n(r)&&n(a.dt_ms)===n(c));return q?status(q):null;},label:{x:'dt (ms)',y:'solref time constant (ms)'}});
const lineData=[0.1,0.25,0.5,1,2].map((dt,i)=>({name:`dt=${dt} ms`,color:['#427bc7','#1d9b86','#d98b24','#8d62c4','#d85757'][i],values:refs.map(ref=>{const q=primary.find(a=>n(a.dt_ms)===dt&&n(a.solref_timeconst_ms)===ref);return{x:String(ref),y:q?n(q.max_penetration_mm):NaN};})}));
out+=linePanel({x:750,y:82,w:720,h:390,title:'B. Softening contact raises peak overlap',note:'Primary sweep; failure points remain shown to expose the boundary.',series:lineData,xvals:refs.map(String),yMax:6,yLabel:'peak overlap (mm)',xLabel:'solref time constant (ms)'});
const dampSeries=[1,2,4].map((ref,i)=>({name:`solref=${ref} ms`,color:['#427bc7','#1d9b86','#d98b24'][i],values:[0.5,1,2].map(dr=>{const q=damp.find(a=>n(a.solref_timeconst_ms)===ref&&n(a.dampratio)===dr);return{x:String(dr),y:q?n(q.settling_time_s):NaN};})}));
out+=linePanel({x:30,y:495,w:700,h:350,title:'C. Damping controls ring-down',note:'At the 1-ms setting, 0.5 is visibly under-damped; 2 settles fastest.',series:dampSeries,xvals:['0.5','1','2'],yMax:2,yLabel:'settling time (s; 2 = not settled)',xLabel:'dampratio'});
const impSeries=[1,2,4].map((ref,i)=>({name:`solref=${ref} ms`,color:['#427bc7','#1d9b86','#d98b24'][i],values:[0.9,0.95,0.99,0.999].map(d=>{const q=imp.find(a=>n(a.solref_timeconst_ms)===ref&&Math.abs(n(a.solimp_d0)-d)<1e-7);return{x:String(d),y:q?n(q.settling_time_s):NaN};})}));
out+=linePanel({x:750,y:495,w:720,h:350,title:'D. solimp changes settling non-monotonically',note:'d0=dwidth=dmax was varied together; this is not only one scalar coefficient.',series:impSeries,xvals:['0.9','0.95','0.99','0.999'],yMax:2,yLabel:'settling time (s; 2 = not settled)',xLabel:'constant impedance d'});
const st=integ.filter(r=>n(r.solref_timeconst_ms)===1); const nIter=st.filter(r=>r.solver==='Newton').reduce((a,r)=>a+n(r.solver_iterations_mean),0)/Math.max(1,st.filter(r=>r.solver==='Newton').length); const cIter=st.filter(r=>r.solver==='CG').reduce((a,r)=>a+n(r.solver_iterations_mean),0)/Math.max(1,st.filter(r=>r.solver==='CG').length);
out+=`<rect x="30" y="870" width="1440" height="48" rx="8" fill="#eaf0f7"/><text class="pt" x="48" y="892">Integrator / solver ablation:</text><text class="subtitle" x="255" y="892">at dt=0.5 ms, stable solref values preserve order with nearly identical peak overlap across Euler, implicitfast and RK4. Newton averaged ${fmt(nIter,1)} iterations versus CG ${fmt(cIter,1)} in the solref=1-ms ablation; cost changes more than the trajectory.</text><text class="note" x="48" y="908">Interpretation: first choose an adequate contact time scale, then use damping and impedance to manage ring-down. Solver/integrator are secondary in the tested stable region.</text>`;
out+='</svg>'; write('thin-disc-mujoco-sweep.svg',out);

const sp=readCsv(`${root}/exp-results/thin-discs/sap-comprehensive-sweep/results_primary.csv`), tau=readCsv(`${root}/exp-results/thin-discs/sap-comprehensive-sweep/results_tau.csv`), gap=readCsv(`${root}/exp-results/thin-discs/sap-comprehensive-sweep/results_gap.csv`), preset=readCsv(`${root}/exp-results/thin-discs/sap-comprehensive-sweep/results_preset.csv`);
const sdt=[0.1,0.25,0.5,1,2,5], kes=[1000,3000,10000,30000,100000,300000,1000000];
out=svgStart(1500,940,'SAP/Warp — thin-disc native parameter sweep');
out+=`<text class="subtitle" x="36" y="60">Three 202-mm × 4-mm analytic discs, 2-s gravity settling; Drake preset in the primary sweep. Shape parameters are shown; equal-material contacts combine to pair parameters.</text>`;
out+=heatmap({x:30,y:82,w:700,h:390,title:'A. Stiffness × timestep',note:'Low ke fails; high ke is stable but coarse dt can retain tail motion.',rows:kes.map(k=>k>=1e6?'1e6':k>=1e3?`${k/1000}e3`:String(k)),cols:sdt.map(String),get:(r,c)=>{const k=r==='1e6'?1e6:Number(r.replace('e3',''))*1000;const q=sp.find(a=>n(a.ke_N_m)===k&&n(a.dt_ms)===n(c));return q?status(q):null;},label:{x:'dt (ms)',y:'shape ke (N/m)'}});
const sapLineKs=[10000,100000,1000000]; const sapSeries=sapLineKs.map((k,i)=>({name:`ke=${k===1000000?'1e6':k/1000+'e3'}`,color:['#d98b24','#1d9b86','#8854c7'][i],values:sdt.map(dt=>{const q=sp.find(a=>n(a.ke_N_m)===k&&n(a.dt_ms)===dt);return{x:String(dt),y:q?n(q.max_penetration_mm):NaN};})}));
out+=linePanel({x:750,y:82,w:720,h:390,title:'B. dt raises peak overlap at a fixed ke',note:'The y axis is capped at 4 mm so escaped low-ke cases do not hide the stable trends.',series:sapSeries,xvals:sdt.map(String),yMax:4,yLabel:'peak overlap (mm)',xLabel:'dt (ms)'});
const tauSeries=[10000,100000].map((k,i)=>({name:`ke=${k/1000}e3`,color:['#d98b24','#1d9b86'][i],values:[0.5,1,3,10,30].map(tv=>{const q=tau.find(a=>n(a.ke_N_m)===k&&n(a.tau_ms)===tv);return{x:String(tv),y:q?n(q.max_penetration_mm):NaN};})}));
out+=linePanel({x:30,y:495,w:700,h:350,title:'C. Larger tau reduces peak overlap in this settling trajectory',note:'At ke=1e5, tau=1–30 ms all settle; at ke=1e4, tau=30 ms finally settles.',series:tauSeries,xvals:['0.5','1','3','10','30'],yMax:4,yLabel:'peak overlap (mm)',xLabel:'shape tau (ms; kd=ke·tau)'});
const gapSeries=[{name:'peak overlap',color:'#8854c7',values:[0,0.05,0.1,0.5].map(g=>{const q=gap.find(a=>n(a.rigid_gap_mm)===g);return{x:String(g),y:q?n(q.max_penetration_mm):NaN};})},{name:'tail jitter ×100',color:'#d85757',values:[0,0.05,0.1,0.5].map(g=>{const q=gap.find(a=>n(a.rigid_gap_mm)===g);return{x:String(g),y:q?n(q.tail_com_jitter_pp_mm)*100:NaN};})}];
out+=linePanel({x:750,y:495,w:720,h:350,title:'D. rigid_gap prevents the zero-gap transient in this case',note:'ke=1e5, tau=3 ms, dt=.5 ms. Jitter is multiplied ×100 only for display.',series:gapSeries,xvals:['0','0.05','0.1','0.5'],yMax:1,yLabel:'mm (jitter ×100)',xLabel:'rigid_gap (mm)'});
const rowsP=[10000,100000,1000000]; let pText=''; rowsP.forEach((k,j)=>{const vals=['approx32','approx64','drake'].map(p=>preset.find(a=>n(a.ke_N_m)===k&&a.preset===p)); pText+=`<text class="legend" x="48" y="${899+j*11}">ke=${k===1000000?'1e6':k/1000+'e3'}: ${vals.map(q=>`${q.preset} ${fmt(n(q.max_penetration_mm),2)} mm / ${fmt(n(q.settling_time_s),3)} s`).join('   |   ')}</text>`;});
out+=`<rect x="30" y="860" width="1440" height="72" rx="8" fill="#eee8f8"/><text class="pt" x="48" y="880">Preset ablation at dt=.5 ms</text><text class="subtitle" x="350" y="880">Drake gives lower peak overlap than approx32/64 at ke=1e5–1e6; all three presets remain ordered in this sampled region.</text>${pText}</svg>`; write('thin-disc-sap-sweep.svg',out);
