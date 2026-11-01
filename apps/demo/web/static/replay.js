"use strict";
const $ = id => document.getElementById(id);
const canvas = $("replay-canvas"), stage = $("replay-stage"), ctx = canvas.getContext("2d");
let session = null, frame = null, index = -1, requestId = 0, playing = false, previousTick = 0, playTime = 0;
let ceresAvailable = false;
let map = null, mapImage = null, world = {scale: 20, x: 0, y: 0}, drag = null;
const fmt = (x, digits=2) => Number(x).toFixed(digits);
function label(id, value){ $(id).textContent = value; }
function alertUser(message){ $("replay-alert").hidden = !message; $("replay-alert").textContent = message || ""; }
function available(){ return !!session; }
async function getJSON(path){
  const r = await fetch(path, {cache:"no-store"}); const body = await r.json();
  if (!r.ok) throw Error(body.error || `HTTP ${r.status}`);
  return body;
}
function setPlaying(value){
  playing = !!value && available(); previousTick = 0; playTime = frame?.t ?? session?.first_t ?? 0;
  label("replay-play", playing ? "❚❚ Pause" : "▶ Play");
}
function setControls(enabled){
  for(const id of ["replay-play","replay-step-back","replay-step-next","replay-slider"]) $(id).disabled = !enabled;
}
function coords(x, y){ const rect=stage.getBoundingClientRect(); return [world.x+x*world.scale, world.y-y*world.scale]; }
function fit(){
  if(!session)return;
  const poses=session.trajectory, xs=poses.map(p=>p[1]), ys=poses.map(p=>p[2]);
  if(frame){
    const [x,y,yaw]=frame.pose, c=Math.cos(yaw), s=Math.sin(yaw);
    for(const p of frame.points){xs.push(x+c*p[0]-s*p[1]);ys.push(y+s*p[0]+c*p[1]);}
  }
  const minX=Math.min(...xs), maxX=Math.max(...xs), minY=Math.min(...ys), maxY=Math.max(...ys);
  const rect=stage.getBoundingClientRect();
  world.scale=Math.max(.1,Math.min(1000,(rect.width-80)/Math.max(3,maxX-minX),(rect.height-80)/Math.max(3,maxY-minY)));
  world.x=rect.width/2-(minX+maxX)/2*world.scale;
  world.y=rect.height/2+(minY+maxY)/2*world.scale;
  render();
}
function drawLine(a,b){const aa=coords(a[0],a[1]),bb=coords(b[0],b[1]);ctx.moveTo(...aa);ctx.lineTo(...bb);}
function render(){
  const r=stage.getBoundingClientRect(), dpr=window.devicePixelRatio||1;
  if(canvas.width!==Math.round(r.width*dpr)||canvas.height!==Math.round(r.height*dpr)){
    canvas.width=Math.round(r.width*dpr);canvas.height=Math.round(r.height*dpr);
  }
  ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,r.width,r.height);
  if(!session)return;
  if(map && mapImage){
    const origin=map.origin, resolution=map.resolution_m;
    ctx.save();ctx.setTransform(dpr*world.scale,0,0,-dpr*world.scale,dpr*world.x,dpr*world.y);
    ctx.translate(origin.x,origin.y);ctx.rotate(origin.yaw_rad);
    ctx.translate(0,map.height*resolution);ctx.scale(1,-1);
    ctx.globalAlpha=.64;ctx.imageSmoothingEnabled=false;
    ctx.drawImage(mapImage,0,0,map.width*resolution,map.height*resolution);ctx.restore();
  }
  ctx.strokeStyle="#547e9c";ctx.lineWidth=1;ctx.globalAlpha=.42;
  const minX=-world.x/world.scale, maxX=(r.width-world.x)/world.scale;
  const minY=(world.y-r.height)/world.scale,maxY=world.y/world.scale;
  const tick=world.scale>35?1:world.scale>8?5:20;
  ctx.beginPath();for(let x=Math.ceil(minX/tick)*tick;x<=maxX;x+=tick)drawLine([x,minY],[x,maxY]);
  for(let y=Math.ceil(minY/tick)*tick;y<=maxY;y+=tick)drawLine([minX,y],[maxX,y]);ctx.stroke();ctx.globalAlpha=1;
  ctx.strokeStyle="#64a7f0";ctx.lineWidth=1.7;ctx.beginPath();
  session.trajectory.forEach((p,i)=>{const pos=coords(p[1],p[2]);if(i===0)ctx.moveTo(...pos);else ctx.lineTo(...pos);});ctx.stroke();
  if(frame){
    const [x,y,yaw]=frame.pose,c=Math.cos(yaw),s=Math.sin(yaw),origin=coords(x,y);
    ctx.strokeStyle="rgba(67,216,179,.16)";ctx.lineWidth=1;ctx.beginPath();
    for(const p of frame.points){const point=coords(x+c*p[0]-s*p[1],y+s*p[0]+c*p[1]);ctx.moveTo(...origin);ctx.lineTo(...point);}ctx.stroke();
    ctx.fillStyle="#52e5b9";ctx.beginPath();
    for(const p of frame.points){const point=coords(x+c*p[0]-s*p[1],y+s*p[0]+c*p[1]);ctx.moveTo(point[0]+2.1,point[1]);ctx.arc(...point,2.1,0,Math.PI*2);}ctx.fill();
    // Orange is an independently *computed* scan-to-frozen-map estimate,
    // NOT independent physical truth. Only accepted matches are drawn.
    if ($("replay-use-ceres").checked && frame.matching?.status === "accepted") {
      const match=frame.matching, [mx,my,ma]=match.estimate;
      const endpoint=coords(mx,my), ca=Math.cos(ma),sa=Math.sin(ma);
      ctx.lineWidth=1.5;ctx.strokeStyle="#f6b35d";ctx.beginPath();
      ctx.moveTo(...origin);ctx.lineTo(...endpoint);ctx.stroke();
      ctx.fillStyle="#ffc978";ctx.beginPath();
      for(const p of frame.points){
        const point=coords(mx+ca*p[0]-sa*p[1],my+sa*p[0]+ca*p[1]);
        ctx.moveTo(point[0]+2,point[1]);ctx.arc(...point,2,0,Math.PI*2);
      }
      ctx.fill();
      ctx.save();ctx.translate(...endpoint);ctx.rotate(-ma);ctx.strokeStyle="#ffb65b";ctx.lineWidth=2;
      ctx.beginPath();ctx.moveTo(15,0);ctx.lineTo(-10,-8);ctx.lineTo(-6,0);ctx.lineTo(-10,8);ctx.closePath();ctx.stroke();ctx.restore();
    }
    ctx.save();ctx.translate(...origin);ctx.rotate(-yaw);ctx.fillStyle="#ffc978";
    ctx.beginPath();ctx.moveTo(15,0);ctx.lineTo(-9,-8);ctx.lineTo(-6,0);ctx.lineTo(-9,8);ctx.closePath();ctx.fill();ctx.restore();
  }
  label("replay-scale",`1 m ≈ ${Math.round(world.scale)} px`);
}
function updateFrameInfo(){
  if(!frame)return;
  label("replay-frame-no",`${frame.index+1} / ${session.frame_count}`);
  label("replay-position",`${frame.index+1} / ${session.frame_count}`);
  label("replay-time",`${fmt(frame.t,3)} s IPC`);
  label("replay-elapsed",`${fmt(frame.t-session.first_t,2)} s`);
  label("replay-pose",frame.pose.map(v=>fmt(v,3)).join(", "));
  label("replay-odom",frame.odom.map(v=>fmt(v,3)).join(", "));
  label("replay-hits",`${frame.hit_count} / ${frame.beam_count}`);
  label("replay-rendered",`${frame.points.length} sampled hits`);
  const m=frame.matching;
  if(!m){
    label("replay-matching", frame.matching_state==="invalid" ? "Invalid Ceres sidecar (refused)" :
      ceresAvailable ? "Not processed at this frame (sampled)" : "No Ceres results loaded");
    label("replay-match-metrics","—");
  } else {
    label("replay-matching", `${m.status.replaceAll("_"," ")} · ${m.iterations} iterations`);
    label("replay-match-metrics",`${(100*m.overlap).toFixed(1)}% overlap · ${m.supported_hits}/${m.input_hits} EDT hits · `+
      `${m.mean_distance_m===null ? "no supported distance" : fmt(m.mean_distance_m,3)+" m mean distance"} · `+
      `robust cost ${fmt(m.initial_robust_cost,2)} → ${fmt(m.robust_cost,2)}`);
  }
  $("replay-slider").value=frame.index;
}
async function seek(i){
  if(!session)return;
  const n=Math.max(0,Math.min(session.frame_count-1,Math.round(i)));
  const request=++requestId;
  try{
    const result=await getJSON(`/api/replays/${encodeURIComponent(session.name)}/frame/${n}`);
    if(request!==requestId)return;
    frame=result;index=n;updateFrameInfo();render();
  }catch(error){if(request===requestId){setPlaying(false);alertUser(`Scan could not be loaded: ${error.message}`);}}
}
async function refreshSessions(){
  setPlaying(false);
  try{
    const result=await getJSON("/api/replays");
    const names=result.sessions;
    const select=$("session-list"), old=select.value;
    select.replaceChildren();
    for(const name of names){const o=document.createElement("option");o.value=name;o.textContent=name;select.append(o);}
    if(!names.length){const o=document.createElement("option");o.textContent="No recorded sessions";o.value="";select.append(o);session=null;frame=null;$("replay-empty").hidden=false;setControls(false);render();return;}
    select.value=names.includes(old)?old:names[0];await loadSession(select.value);
  }catch(error){alertUser(`Cannot list sessions: ${error.message}`);}
}
async function refreshMaps(){
  try{
    const r=await getJSON("/api/maps");const select=$("overlay-map");select.replaceChildren();
    const none=document.createElement("option");none.value="";none.textContent="No map overlay";select.append(none);
    for(const m of r.maps){const o=document.createElement("option");o.value=m.id;o.textContent=m.name;select.append(o);}
  }catch(error){alertUser(`Cannot list maps: ${error.message}`);}
}
async function loadSession(name){
  setPlaying(false);session=null;frame=null;requestId++;$("replay-holdout-report").hidden=true;$("replay-empty").hidden=true;setControls(false);map=null;mapImage=null;$("overlay-map").value="";render();
  if(!name){$("replay-empty").hidden=false;return;}
  try{
    session=await getJSON(`/api/replays/${encodeURIComponent(name)}`);
    ceresAvailable=session.ceres?.status === "valid";
    $("replay-use-ceres").disabled=!ceresAvailable;
    $("replay-use-ceres").checked=false;
    const holdout=session.ceres?.evaluation;
    const report=$("replay-holdout-report");
    report.hidden = !(holdout?.training_split_checked);
    if (holdout?.training_split_checked) {
      const counts=holdout.status_counts;
      report.textContent = `TRAINING-EXCLUDED FR079 SCANS · ${holdout.evaluated_scans} evaluated · `+
        `${holdout.accepted} accepted (${(100*holdout.acceptance_fraction).toFixed(1)}%) · `+
        `${(100*holdout.mean_overlap).toFixed(1)}% mean EDT overlap · `+
        `cost ${holdout.mean_initial_robust_cost.toFixed(2)} → ${holdout.mean_final_robust_cost.toFixed(2)} · `+
        `no independent ground truth / no per-beam timing`;
    }
    label("replay-ceres-source",ceresAvailable?
      `${session.ceres.processed} matched scan attempts · ${session.ceres.accepted} accepted · frozen ${session.ceres.map_name}`:
      session.ceres?.status==="invalid" ? session.ceres.error : "No native Ceres sidecar found");
    label("replay-dataset",session.source.dataset);
    label("replay-pose-kind",session.source.pose_kind.replaceAll("_"," "));
    label("replay-frame-kind",session.source.frame_id);
    label("replay-clock",session.source.timestamp_kind);
    label("replay-source-hash",session.source.source_sha256);
    $("replay-slider").max=session.frame_count-1;setControls(true);
    alertUser(session.ceres?.status==="invalid" ? session.ceres.error : "");
    await seek(0);fit();
  }catch(error){session=null;setControls(false);alertUser(`Session refused: ${error.message}`);}
}
async function chooseMap(name){
  map=null;mapImage=null;
  label("replay-map-note","No map alignment assumed");render();
  if(!name||!session)return;
  try{
    const m=await getJSON(`/api/maps/${encodeURIComponent(name)}`);
    if(m.frame_id!==session.source.frame_id){
      $("overlay-map").value="";alertUser("Map overlay refused: declared frame labels differ. A coordinate transform must be independently established.");return;
    }
    const response=await fetch(`/api/maps/${encodeURIComponent(name)}/image.png`,{cache:"no-store"});
    if(!response.ok)throw Error("map raster refused");
    const url=URL.createObjectURL(await response.blob()), img=new Image();
    try{await new Promise((resolve,reject)=>{img.onload=resolve;img.onerror=reject;img.src=url;});}
    finally{URL.revokeObjectURL(url);}
    map=m;mapImage=img;
    label("replay-map-note","UNVERIFIED overlay: equal frame labels are not calibration");
    alertUser("Visualization only: matching frame labels do NOT prove scan-to-map alignment or ground truth.");render();
  }catch(error){$("overlay-map").value="";alertUser(`Map overlay refused: ${error.message}`);}
}
function tick(now){
  if(playing&&session){
    if(previousTick){
      playTime += (now-previousTick)/1000*Number($("replay-speed").value);
      let next=index;
      // Look ahead only until the next recorded scan boundary.
      while(next+1<session.frame_count&&session.trajectory[next+1][0]<=playTime)next++;
      if(next!==index)seek(next);
      if(next===session.frame_count-1)setPlaying(false);
    }
    previousTick=now;
  }
  requestAnimationFrame(tick);
}
$("replay-refresh").addEventListener("click",()=>{refreshSessions();refreshMaps();});
$("session-list").addEventListener("change",event=>loadSession(event.target.value));
$("overlay-map").addEventListener("change",event=>chooseMap(event.target.value));
$("replay-fit").addEventListener("click",fit);
$("replay-use-ceres").addEventListener("change",render);
$("replay-play").addEventListener("click",()=>setPlaying(!playing));
$("replay-step-back").addEventListener("click",()=>{setPlaying(false);seek(index-1);});
$("replay-step-next").addEventListener("click",()=>{setPlaying(false);seek(index+1);});
$("replay-slider").addEventListener("input",event=>{setPlaying(false);seek(Number(event.target.value));});
window.addEventListener("keydown",event=>{if(event.code==="Space"&&!(["INPUT","SELECT","TEXTAREA","BUTTON"].includes(document.activeElement.tagName))){event.preventDefault();setPlaying(!playing);}});
window.addEventListener("resize",render);
stage.addEventListener("wheel", event => {
  if (!available()) return;
  event.preventDefault();
  // Do not zoom during middle-button panning or on horizontal-only input.
  if (drag?.button === 1 || (event.buttons & 4) || event.deltaY === 0) return;
  const rect=stage.getBoundingClientRect();
  const x=event.clientX-rect.left, y=event.clientY-rect.top;
  const next=Math.max(.1,Math.min(1000,world.scale*(event.deltaY<0?1.18:1/1.18)));
  const factor=next/world.scale;
  world.x=x-(x-world.x)*factor;world.y=y-(y-world.y)*factor;world.scale=next;render();
}, {passive:false});
// Cancel native middle-button autoscroll/clipboard paste on both event paths.
for (const type of ["mousedown", "auxclick"]) {
  stage.addEventListener(type, event => { if (event.button === 1) event.preventDefault(); });
}
stage.addEventListener("pointerdown", event => {
  if (event.button === 1) event.preventDefault();
  if (!available() || drag || (event.button !== 0 && event.button !== 1)) return;
  drag={pointerId:event.pointerId, button:event.button,
        x:event.clientX,y:event.clientY,ox:world.x,oy:world.y};
  stage.setPointerCapture(event.pointerId);
});
stage.addEventListener("pointermove", event => {
  if (!drag || event.pointerId !== drag.pointerId) return;
  world.x=drag.ox+event.clientX-drag.x;world.y=drag.oy+event.clientY-drag.y;render();
});
function endPan(event) {
  if (!drag || event.pointerId !== drag.pointerId) return;
  drag=null;
  if (stage.hasPointerCapture(event.pointerId)) stage.releasePointerCapture(event.pointerId);
}
for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
  stage.addEventListener(type, endPan);
}
refreshSessions();refreshMaps();requestAnimationFrame(tick);
