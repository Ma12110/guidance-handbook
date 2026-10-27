(function(){
'use strict';
let active=null;
const escape=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const fmt=x=>typeof x==='number'?(Math.abs(x)>=1000?x.toExponential(2):Number(x.toFixed(3)).toString()):String(x);
const STATIC=new Set([2,6,9,15]);
const X_LABEL={2:'坐标偏航角 / °',6:'空间信标相位 / °',9:'预测时域 / s',15:'高度 / km'};
const finite=(p,d)=>Array.isArray(p)&&p.length>=d&&p.slice(0,d).every(Number.isFinite);
const interpolate=(a,b,t)=>a.map((v,i)=>v+(b[i]-v)*t);
function bounds(points,d){const lo=Array(d).fill(Infinity),hi=Array(d).fill(-Infinity);for(const p of points)for(let i=0;i<d;i++){lo[i]=Math.min(lo[i],p[i]);hi[i]=Math.max(hi[i],p[i])}return{lo,hi}}
function bracket(times,t){if(t<=times[0])return{idx:0,f:0};let a=0,b=times.length-1;while(a+1<b){const m=(a+b)>>1;if(times[m]<=t)a=m;else b=m}if(t>=times[b])return{idx:b,f:0};return{idx:a,f:(t-times[a])/(times[b]-times[a])}}
window.unmountChapterLab=function(){if(active){active.dispose();active=null}};
window.mountChapterLab=function(n,host){
 window.unmountChapterLab();const def=window.ChapterLabs?.[n];if(!def){host.innerHTML='<div class="lab-error">本章实验未加载，请刷新页面。</div>';return}
 const isStatic=def.presentation==='static'||STATIC.has(n),root=document.createElement('section');root.className='chapter-lab';root.setAttribute('aria-label',def.title);host.append(root);
 root.innerHTML=`<div class="lab-heading"><small>CHAPTER ${String(n).padStart(2,'0')} · INTERACTIVE LAB</small><h3>${escape(def.title)}</h3><p>${escape(def.description)}</p></div><div class="lab-controls"></div><div class="lab-actions"><button type="button" data-action="play">▶ 播放</button><button type="button" data-action="reset">重置</button><select aria-label="视图模式"><option value="3d">三维透视 · 可旋转</option><option value="2d">二维 XY 俯视</option></select><input type="range" min="0" max="1000" value="1000" aria-label="实验时间进度"><span class="lab-status"></span></div><div class="lab-canvas-box scene-box"><div class="lab-canvas-caption">拖动旋转 · 滚轮缩放 · 方向键调整视角 · 红 X / 绿 Y / 蓝 Z</div><canvas class="lab-scene" tabindex="0" aria-label="可旋转三维空间轨迹"></canvas></div><div class="lab-legend"></div><div class="lab-canvas-box chart-box"><div class="lab-canvas-caption"></div><canvas class="lab-chart" aria-label="实验数值曲线"></canvas></div><div class="lab-summary-label" style="font-size:12px;color:#687f91;margin:12px 0 4px">完整计算结果（不随回放位置改变）</div><div class="lab-metrics"></div><p class="lab-note"></p>`;
 const q=s=>root.querySelector(s),controls=q('.lab-controls'),scene=q('.lab-scene'),chart=q('.lab-chart'),scrub=q('[aria-label="实验时间进度"]'),play=q('[data-action="play"]'),mode=q('select'),status=q('.lab-status');
 let params={},data={},paths=[],series=[],groups=[],pb=null,cb=null,timeStart=0,timeEnd=1,fraction=1,playing=false,raf=0,previous=0,drawRaf=0,calcTimer=0,yaw=-.65,pitch=.52,zoom=1.8,drag=null,dead=false,revision=0,sc=null,cc=null,automaticView=true;
 const clean=[];
 function listen(el,event,fn,opt){el.addEventListener(event,fn,opt);clean.push(()=>el.removeEventListener(event,fn,opt))}
 function invalidate(){sc=null;cc=null}
 function requestDraw(){if(!dead&&!drawRaf)drawRaf=requestAnimationFrame(()=>{drawRaf=0;draw()})}
 function stop(){playing=false;play.textContent='▶ 播放';cancelAnimationFrame(raf);previous=0}
 function cancelCalculation(){clearTimeout(calcTimer);calcTimer=0}
 // Only the latest slider state is solved after 90 ms without another input event.
 function scheduleCalculation(){cancelCalculation();calcTimer=setTimeout(()=>{calcTimer=0;calculate()},90)}
 function flushCalculation(){if(calcTimer){cancelCalculation();calculate()}}
 for(const p of def.params||[]){params[p.key]=p.value;const d=document.createElement('div');d.className='lab-param';const id=`lab-${n}-${p.key}`;d.innerHTML=`<label for="${escape(id)}">${escape(p.label)}<output>${escape(fmt(p.value))}</output></label><input id="${escape(id)}" type="range" min="${p.min}" max="${p.max}" step="${p.step}" value="${p.value}">`;controls.append(d);const input=d.querySelector('input');listen(input,'input',e=>{params[p.key]=+e.target.value;d.querySelector('output').textContent=fmt(params[p.key]);stop();status.textContent='参数更新中…';scheduleCalculation()});listen(input,'change',flushCalculation)}
 function calculate(){if(dead)return;try{
  data=def.run({...params})||{};revision++;
  paths=(data.paths||[]).filter(s=>s.kind!=='chart').map(s=>({...s,points:(s.points||[]).filter(p=>finite(p,3))}));
  series=(data.series||[]).map((s,i)=>({...s,interpolation:s.interpolation||((n===12&&i===0)||(n===13&&i===2)||(n===16&&i<2)?'step':'linear'),points:(s.points||[]).filter(p=>finite(p,2))}));
  for(const s of series)for(let i=1;i<s.points.length;i++)if(s.points[i][0]<s.points[i-1][0])throw new Error('曲线横轴必须单调：'+s.name);
  const grouped=new Map();for(const s of series){const key=(s.group||'曲线比较')+'|'+(s.unit||'');if(!grouped.has(key))grouped.set(key,{name:s.group||'曲线比较',unit:s.unit||'',series:[]});grouped.get(key).series.push(s)}groups=[...grouped.values()];chart.style.height=Math.max(240,groups.length*180+34)+'px';
  const pp=paths.flatMap(s=>s.points),sp=series.flatMap(s=>s.points);pb=pp.length?bounds(pp,3):null;cb=sp.length?bounds(sp,2):null;
  timeStart=cb?cb.lo[0]:0;timeEnd=cb?cb.hi[0]:1;if(!(timeEnd>timeStart))timeEnd=timeStart+1;
  play.hidden=isStatic;scrub.hidden=isStatic;
  status.textContent=isStatic?({2:'坐标与矢量参数图',6:'观测相位下的空间几何',9:'预测时域扫描',15:'高度剖面参数扫描'}[n]||'静态参数图'):'';
  q('.chart-box .lab-canvas-caption').textContent='横轴：'+(data.xLabel||X_LABEL[n]||'时间 / s')+' · 不同量纲分图显示，各图纵轴独立';
  q('.lab-note').textContent=(data.note||def.note||'')+(isStatic?'':' 完整时间过程预先计算；播放是7.5秒的缩放回放，不是实时求解。');
  q('.lab-metrics').innerHTML=Object.entries(data.metrics||{}).map(([k,v])=>`<div class="lab-metric"><span>${escape(k)}</span><strong>${escape(fmt(v))}</strong></div>`).join('');
  q('.lab-legend').innerHTML=[...paths,...series].map(s=>`<span><i style="background:${/^#[a-fA-F0-9]{3,8}$/.test(s.color||'')?s.color:'#2f6689'}"></i>${escape(s.name)}</span>`).join('');
  q('.scene-box').hidden=!paths.length;q('.chart-box').hidden=!series.length;mode.hidden=!paths.length;if(automaticView&&pb)mode.value=pb.hi[2]-pb.lo[2]<1e-9?'2d':'3d';q('.lab-summary-label').textContent=isStatic?'完整参数计算结果':'完整计算结果（不随回放位置改变）';fraction=1;scrub.value=1000;invalidate();requestDraw();
 }catch(e){q('.lab-note').textContent='计算失败：'+e.message}}
 function context(canvas){const r=canvas.getBoundingClientRect(),w=Math.max(160,r.width),h=r.height||240,dpr=Math.min(2,window.devicePixelRatio||1);if(canvas.width!==Math.round(w*dpr)||canvas.height!==Math.round(h*dpr)){canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr)}const c=canvas.getContext('2d');c.setTransform(dpr,0,0,dpr,0,0);c.clearRect(0,0,w,h);return[c,w,h]}
 function buildScene(w,h){
  const lo=pb.lo.map(v=>Math.min(0,v)),hi=pb.hi.map(v=>Math.max(0,v)),center=lo.map((x,i)=>(x+hi[i])/2),span=Math.max(...hi.map((x,i)=>x-lo[i]),.1),planar=mode.value==='2d';let scale=Math.min(w*.7,h*.62)/span*zoom,ox=0,oy=0;
  const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sn=Math.sin(pitch);
  function project(p){const x=p[0]-center[0],y=p[1]-center[1],z=p[2]-center[2];if(planar)return[w/2+x*scale+ox,h/2-y*scale+oy];const xx=cy*x-sy*y,yy=sy*x+cy*y,zz=cp*z-sn*yy,depth=cp*yy+sn*z,pers=1/(1+depth/(span*4));return[w/2+xx*scale*pers+ox,h/2-zz*scale*pers+oy]}
  const box=bounds(paths.flatMap(s=>s.points.map(project)),2),fit=Math.min(1,(w-66)/Math.max(1,box.hi[0]-box.lo[0]),(h-100)/Math.max(1,box.hi[1]-box.lo[1]));scale*=fit;ox=-(.5*(box.lo[0]+box.hi[0])-w/2)*fit;oy=-(.5*(box.lo[1]+box.hi[1])-h/2)*fit+12;
  const floor=lo[2],minx=center[0]-span*.6,maxx=center[0]+span*.6,miny=center[1]-span*.6,maxy=center[1]+span*.6,grid=[];
  for(let i=0;i<=10;i++){const t=i/10;grid.push([[minx+(maxx-minx)*t,miny,floor],[minx+(maxx-minx)*t,maxy,floor]].map(project));grid.push([[minx,miny+(maxy-miny)*t,floor],[maxx,miny+(maxy-miny)*t,floor]].map(project))}
  const origin=[0,0,0],len=span*.35;
  return{key:[revision,w,h,yaw,pitch,zoom,mode.value].join('|'),project,planar,floor,grid,corners:[[minx,miny,floor],[maxx,miny,floor],[maxx,maxy,floor],[minx,maxy,floor]].map(project),axes:[[[len,0,0],'#bd4d48','X'],[[0,len,0],'#32866b','Y'],[[0,0,len],'#3974b6','Z']].map(([p,color,label])=>({a:project(origin),b:project(p),color,label})),paths:paths.map(s=>{const times=s.times||(series[0]?.points.length===s.points.length?series[0].points.map(p=>p[0]):null);return{...s,times,screen:s.points.map(project),fixed:isStatic||s.fixed===true||s.points.length<=4||!times}})};
 }
 function segment(c,a,b,color,width=1,dash=[]){c.beginPath();c.moveTo(a[0],a[1]);c.lineTo(b[0],b[1]);c.strokeStyle=color;c.lineWidth=width;c.setLineDash(dash);c.stroke();c.setLineDash([])}
 function drawScene(){if(!pb||q('.scene-box').hidden)return;const[c,w,h]=context(scene),key=[revision,w,h,yaw,pitch,zoom,mode.value].join('|');if(sc?.key!==key)sc=buildScene(w,h);
  scene.setAttribute('aria-label',sc.planar?'二维 XY 位置投影图':'可旋转三维位置图');q('.scene-box .lab-canvas-caption').textContent=sc.planar?'二维 XY 俯视 · 滚轮缩放 · 物理原点 O 与 X/Y 轴':'三维透视 · 拖动旋转 · 滚轮缩放 · 方向键调整 · 红X / 绿Y / 蓝Z';
  c.fillStyle=sc.planar?'#f8fafc':'#eff5fa';c.fillRect(0,0,w,h);if(!sc.planar){c.beginPath();sc.corners.forEach((p,i)=>i?c.lineTo(...p):c.moveTo(...p));c.closePath();c.fillStyle='#dce7f1';c.fill()}for(const [a,b]of sc.grid)segment(c,a,b,'#cad9e5');for(const a of sc.axes){if(sc.planar&&a.label==='Z')continue;segment(c,a.a,a.b,a.color,2);c.fillStyle=a.color;c.font='bold 12px sans-serif';c.fillText(a.label,a.b[0]+5,a.b[1]-5)}
  if(sc.axes.length){const o=sc.axes[0].a;c.fillStyle='#506b80';c.font='11px sans-serif';c.fillText('O',o[0]-11,o[1]+13)}
  sc.paths.forEach((s,k)=>{if(!s.points.length)return;const {idx,f}=s.fixed?{idx:s.points.length-1,f:0}:bracket(s.times,timeStart+fraction*(timeEnd-timeStart));let last=s.points[idx],screenLast=s.screen[idx];if(f&&idx+1<s.points.length){last=interpolate(last,s.points[idx+1],f);screenLast=sc.project(last)}c.beginPath();for(let i=0;i<=idx;i++){const p=s.screen[i];i?c.lineTo(...p):c.moveTo(...p)}if(f)c.lineTo(...screenLast);c.strokeStyle=s.color||'#286b8c';c.lineWidth=2.5;c.stroke();if(!sc.planar){const ground=sc.project([last[0],last[1],sc.floor]);segment(c,screenLast,ground,s.color||'#286b8c',1,[4,4]);c.beginPath();c.ellipse(...ground,5,2.5,0,0,Math.PI*2);c.fillStyle='#7c95a63d';c.fill()}c.beginPath();c.arc(...screenLast,5,0,Math.PI*2);c.fillStyle=s.color||'#286b8c';c.fill();c.strokeStyle='#fff';c.lineWidth=1.5;c.stroke();const label=s.name+(sc.planar?'':` · z=${fmt(last[2])}`);c.font='11px sans-serif';const lw=Math.min(w-24,c.measureText(label).width+16),x=w-lw-12,y=39+k*21;c.fillStyle='#ffffffed';c.fillRect(x-4,y-12,lw+4,18);c.fillStyle=s.color||'#286b8c';c.fillRect(x,y-7,8,3);c.fillStyle='#30495c';c.fillText(label,x+13,y,Math.max(50,lw-17))});
  c.fillStyle='#5e788e';c.font='11px sans-serif';c.fillText(sc.planar?'二维 XY 俯视（不显示高度）':`三维透视 · 参考投影平面 z=${fmt(sc.floor)} · 轴线从物理原点出发`,12,19);
 }
 function buildChart(w,h){
  let xmin=cb.lo[0],xmax=cb.hi[0];if(xmax===xmin)xmax=xmin+1;const L=w<500?58:70,R=w-18,X=x=>L+(x-xmin)/(xmax-xmin)*(R-L),panelH=(h-30)/groups.length;
  const panels=groups.map((g,i)=>{const b=bounds(g.series.flatMap(s=>s.points),2);let ymin=b.lo[1],ymax=b.hi[1];if(ymax===ymin){ymin-=.5;ymax+=.5}const pad=(ymax-ymin)*.08;ymin-=pad;ymax+=pad;const top=i*panelH,T=top+31,B=top+panelH-31,Y=y=>B-(y-ymin)/(ymax-ymin)*(B-T);return{...g,top,T,B,ymin,ymax,Y,series:g.series.map(s=>({...s,times:s.points.map(p=>p[0]),screen:s.points.map(p=>[X(p[0]),Y(p[1])])}))}});
  return{key:[revision,w,h].join('|'),xmin,xmax,L,R,X,panels};
 }
 function drawChart(){
  if(!cb||q('.chart-box').hidden)return;const[c,w,h]=context(chart),key=[revision,w,h].join('|');if(cc?.key!==key)cc=buildChart(w,h);const current=timeStart+fraction*(timeEnd-timeStart);
  for(const panel of cc.panels){
   c.textAlign='left';c.fillStyle='#294f69';c.font='bold 12px sans-serif';c.fillText(panel.name+(panel.unit?' / '+panel.unit:''),cc.L,panel.top+17,w-cc.L-15);c.font='10px sans-serif';
   for(let j=0;j<=4;j++){const y=panel.ymin+(panel.ymax-panel.ymin)*j/4;segment(c,[cc.L,panel.Y(y)],[cc.R,panel.Y(y)],'#e1e8ef');c.fillStyle='#6d8194';c.textAlign='right';c.fillText(fmt(y),cc.L-7,panel.Y(y)+3);const x=cc.xmin+(cc.xmax-cc.xmin)*j/4;c.textAlign='center';c.fillText(fmt(x),cc.X(x),panel.B+17)}c.textAlign='left';
   c.save();c.beginPath();c.rect(cc.L,panel.T,cc.R-cc.L,panel.B-panel.T);c.clip();
   for(const s of panel.series){
    if(!s.points.length)continue;const point=isStatic?{idx:s.points.length-1,f:0}:bracket(s.times,current);c.beginPath();for(let i=0;i<=point.idx;i++){const p=s.screen[i];if(!i)c.moveTo(...p);else{if(s.interpolation==='step')c.lineTo(p[0],s.screen[i-1][1]);c.lineTo(...p)}}if(point.f&&point.idx+1<s.points.length){const a=s.screen[point.idx],b=s.screen[point.idx+1],p=interpolate(a,b,point.f);if(s.interpolation==='step')p[1]=a[1];c.lineTo(...p)}c.strokeStyle=s.color||'#2f6689';c.lineWidth=2;c.stroke();
   }
   if(!isStatic)segment(c,[cc.X(current),panel.T],[cc.X(current),panel.B],'#8ba1b2',1,[3,4]);c.restore();
  }
  c.textAlign='center';c.fillStyle='#6d8194';c.font='11px sans-serif';c.fillText(data.xLabel||X_LABEL[n]||'时间 / s',w/2,h-4);c.textAlign='left';
 }
 function draw(){if(dead)return;if(!isStatic)status.textContent=`t = ${fmt(timeStart+fraction*(timeEnd-timeStart))} / ${fmt(timeEnd)} s`;drawScene();drawChart()}
 // The animation only draws cached numerical results; it never reruns a model or scans bounds.
 function tick(t){if(!playing||dead)return;if(previous)fraction=Math.min(1,fraction+Math.min(100,t-previous)/7500);previous=t;scrub.value=Math.round(fraction*1000);draw();if(fraction>=1)stop();else raf=requestAnimationFrame(tick)}
 listen(play,'click',()=>{if(isStatic)return;flushCalculation();if(playing){stop();return}if(fraction>=1)fraction=0;playing=true;previous=0;play.textContent='❚❚ 暂停';raf=requestAnimationFrame(tick)});
 listen(q('[data-action="reset"]'),'click',()=>{stop();cancelCalculation();automaticView=true;yaw=-.65;pitch=.52;zoom=1.8;for(const p of def.params||[]){params[p.key]=p.value;const el=root.querySelector('#'+CSS.escape(`lab-${n}-${p.key}`));el.value=p.value;el.parentNode.querySelector('output').textContent=fmt(p.value)}calculate()});
 listen(scrub,'input',()=>{stop();fraction=+scrub.value/1000;requestDraw()});listen(mode,'change',()=>{automaticView=false;sc=null;requestDraw()});
 listen(scene,'pointerdown',e=>{if(mode.value==='2d'||e.button!==0)return;drag={x:e.clientX,y:e.clientY};scene.setPointerCapture(e.pointerId)});listen(scene,'pointermove',e=>{if(!drag)return;yaw+=(e.clientX-drag.x)*.008;pitch=Math.max(-1.3,Math.min(1.3,pitch+(e.clientY-drag.y)*.008));drag={x:e.clientX,y:e.clientY};sc=null;requestDraw()});listen(scene,'pointerup',()=>drag=null);listen(scene,'pointercancel',()=>drag=null);
 listen(scene,'wheel',e=>{e.preventDefault();zoom=Math.max(.35,Math.min(3,zoom*Math.exp(-e.deltaY*.001)));sc=null;requestDraw()},{passive:false});listen(scene,'keydown',e=>{if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown'].includes(e.key))return;e.preventDefault();if(e.key==='ArrowLeft')yaw-=.1;if(e.key==='ArrowRight')yaw+=.1;if(e.key==='ArrowUp')pitch=Math.min(1.3,pitch+.1);if(e.key==='ArrowDown')pitch=Math.max(-1.3,pitch-.1);sc=null;requestDraw()});
 listen(document,'visibilitychange',()=>{if(document.visibilityState==='hidden')stop()});
 const ro=new ResizeObserver(()=>{invalidate();requestDraw()});ro.observe(root);active={dispose(){dead=true;stop();cancelCalculation();cancelAnimationFrame(drawRaf);ro.disconnect();clean.forEach(f=>f());root.remove()}};calculate();
};
})();
