"use strict";
(() => {
  const el = id => document.getElementById(id);
  const stage = el("fusion-stage");
  const canvas = el("fusion-canvas");
  const ctx = canvas.getContext("2d", {alpha:false});
  const colors = {
    wheel_only:"#d4a36b", lidar_only:"#b99df8",
    gyro_wheel:"#63a4ed", fused_scan_matched:"#59e2bd",
    graph_optimized:"#f17f79", synthetic_truth_for_scoring:"#f1d36d"
  };
  const labels = {
    wheel_only:"Wheel only", lidar_only:"LiDAR only",
    gyro_wheel:"Gyro + wheel", fused_scan_matched:"Scan matched",
    graph_optimized:"Graph optimized"
  };
  const $run = el("fusion-runs"), $map = el("fusion-map");
  let report=null, poses=null, trace=null, runId="", frameIndex=0;
  let mapImage=null, mapMeta=null, loadingToken=0, imageToken=0;
  let playing=false, lastTime=0, carry=0, view={x:0,y:0,scale:20};
  let drag=null;
  const showMessage = (message) => {
    const alert=el("fusion-alert");
    alert.textContent=message; alert.hidden=!message;
  };
  const shown = (id,value) => {el(id).textContent=String(value);};
  const fmt = (n,digits=2) => Number.isFinite(Number(n)) ?
    Number(n).toFixed(digits) : "—";
  const methods = () => poses ? Object.keys(poses.paths) : [];
  const visible = method => {
    const input=document.querySelector('input[data-path="'+method+'"]');
    return !!input && !input.closest("label").hidden && input.checked;
  };
  async function getJson(url) {
    const response=await fetch(url,{cache:"no-store"});
    if(!response.ok)throw new Error(url+" — HTTP "+response.status);
    return response.json();
  }
  const urlBase=() => "/api/fusion/runs/"+encodeURIComponent(runId);
  function setReady(ready) {
    for(const id of ["fusion-play","fusion-back","fusion-next","fusion-slider"])
      el(id).disabled=!ready;
    el("fusion-empty").hidden=ready;
  }
  function stop() {
    playing=false;lastTime=0;carry=0;
    shown("fusion-play","▶ Play");
  }
  function setFrame(idx) {
    if(!trace)return;
    frameIndex=Math.max(0,Math.min(trace.frames.length-1,Math.round(idx)));
    el("fusion-slider").value=String(frameIndex);
    shown("fusion-position",(frameIndex+1)+" / "+trace.frames.length);
    const f=trace.frames[frameIndex];
    shown("fusion-frame-no",(frameIndex+1)+" / "+trace.frame_count);
    const elapsed=(Number(f.time_ns)-Number(trace.frames[0].time_ns))*1e-9;
    shown("fusion-time",fmt(elapsed,2)+" s elapsed");
    shown("fusion-scan-hits",f.hits+" / "+f.beam_count+
      " ("+f.points_laser.length+" plotted)");
    shown("fusion-match-status",f.matching.status);
    shown("fusion-match-reason",f.matching.reason.replaceAll("_"," "));
    shown("fusion-supported",f.matching.attempted ?
      f.matching.supported_hits+" / "+f.matching.input_hits : "Training scan — not queried");
    shown("fusion-overlap",f.matching.attempted ?
      fmt(100*f.matching.overlap,1)+"%" : "—");
    shown("fusion-match-count",f.matching.cumulative_accepted+
      " accepted / "+f.matching.cumulative_rejected+" rejected");
    shown("fusion-insertion",f.mapping.lidar_only_inserted?
      "LiDAR-only: inserted" : "LiDAR-only: rejected (not inserted)");
    shown("fusion-loop-status",f.loop_events.length ?
      f.loop_events.map(x=>x.from+" → "+x.to+" "+x.decision).join("; "):
      "No independent loop proposal");
    draw();
  }
  function setMetrics() {
    shown("fusion-scene",report.scene.replace("synthetic-","").replace("-v1",""));
    shown("fusion-backend",report.matcher);
    shown("fusion-accepted",report.heldout_accepted+" / "+report.heldout_attempted);
    shown("fusion-loops",report.graph_accepted_loops+" / "+report.graph_loop_candidates);
    const scores=el("fusion-scores");
    scores.replaceChildren();
    for(const method of methods()) {
      const m=report.results[method];
      const card=document.createElement("div");card.className="fusion-score";
      const dot=document.createElement("span");dot.className="marker";
      dot.style.background=colors[method];
      const name=document.createElement("b");name.textContent=labels[method];
      const value=document.createElement("span");
      value.textContent=fmt(m.translation_rmse_m,3)+" m RMSE";
      const detail=document.createElement("small");
      detail.textContent="Map IoU "+(m.map.metrics.occupied_iou==null?
        "undefined":fmt(m.map.metrics.occupied_iou,3))+
        " · known "+fmt(100*m.map.metrics.known_coverage,1)+"%";
      card.append(dot,name,value,detail);scores.append(card);
    }
  }
  function bounds() {
    let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
    const add=(x,y)=>{if(Number.isFinite(x)&&Number.isFinite(y)){
      minX=Math.min(minX,x);minY=Math.min(minY,y);
      maxX=Math.max(maxX,x);maxY=Math.max(maxY,y);
    }};
    if(poses) {
      for(const path of Object.values(poses.paths))
        for(const p of path)add(p[0],p[1]);
      for(const p of poses.synthetic_truth_for_scoring)add(p[0],p[1]);
    }
    if(mapMeta) {
      const {width,height,resolution_m,origin}=mapMeta;
      for(const [x,y] of [[0,0],[width*resolution_m,0],
                           [0,height*resolution_m],
                           [width*resolution_m,height*resolution_m]]) {
        add(origin.x+Math.cos(origin.yaw_rad)*x-Math.sin(origin.yaw_rad)*y,
            origin.y+Math.sin(origin.yaw_rad)*x+Math.cos(origin.yaw_rad)*y);
      }
    }
    if(!Number.isFinite(minX))return {minX:0,minY:0,maxX:15,maxY:12};
    const padding=Math.max(.8,.08*Math.max(maxX-minX,maxY-minY));
    return {minX:minX-padding,minY:minY-padding,
            maxX:maxX+padding,maxY:maxY+padding};
  }
  function fit() {
    const b=bounds(),rect=stage.getBoundingClientRect();
    if(!rect.width||!rect.height)return;
    view.scale=Math.min((rect.width-35)/Math.max(.01,b.maxX-b.minX),
                        (rect.height-35)/Math.max(.01,b.maxY-b.minY));
    view.x=(rect.width-view.scale*(b.maxX+b.minX))/2;
    view.y=(rect.height+view.scale*(b.maxY+b.minY))/2;
    draw();
  }
  function resize() {
    const rect=stage.getBoundingClientRect();
    const dpr=Math.min(window.devicePixelRatio||1,2);
    canvas.width=Math.round(rect.width*dpr);
    canvas.height=Math.round(rect.height*dpr);
    draw();
  }
  function worldLaser(pose,point) {
    const yaw=pose[2]+trace.base_from_laser_yaw_rad;
    const baseAngle=pose[2];
    const [lx,ly]=trace.laser_from_base_translation_m;
    const x=pose[0]+lx*Math.cos(baseAngle)-ly*Math.sin(baseAngle);
    const y=pose[1]+lx*Math.sin(baseAngle)+ly*Math.cos(baseAngle);
    return [x+point[0]*Math.cos(yaw)-point[1]*Math.sin(yaw),
            y+point[0]*Math.sin(yaw)+point[1]*Math.cos(yaw)];
  }
  function pathLayer(path,method) {
    const end=Math.min(frameIndex,path.length-1);
    if(end<0)return;
    ctx.strokeStyle=colors[method];ctx.lineWidth=2/view.scale;
    ctx.beginPath();
    path.slice(0,end+1).forEach((p,i)=>{
      if(i===0)ctx.moveTo(p[0],p[1]);else ctx.lineTo(p[0],p[1]);
    });ctx.stroke();
    const p=path[end];
    ctx.save();ctx.translate(p[0],p[1]);ctx.rotate(p[2]);
    ctx.fillStyle=colors[method];ctx.strokeStyle="#091322";
    ctx.lineWidth=1.6/view.scale;
    ctx.beginPath();ctx.moveTo(10/view.scale,0);
    ctx.lineTo(-7/view.scale,-6/view.scale);
    ctx.lineTo(-5/view.scale,0);ctx.lineTo(-7/view.scale,6/view.scale);
    ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
  }
  function renderMap() {
    if(!mapImage||!mapMeta)return;
    const {origin,width,height,resolution_m}=mapMeta;
    const w=width*resolution_m,h=height*resolution_m;
    ctx.save();ctx.globalAlpha=.76;
    ctx.translate(origin.x,origin.y);ctx.rotate(origin.yaw_rad);
    ctx.translate(0,h);ctx.scale(1,-1);
    ctx.drawImage(mapImage,0,0,w,h);
    ctx.restore();
  }
  function dots() {
    if(!el("fusion-history").checked)return;
    const path=poses.paths.fused_scan_matched;
    ctx.fillStyle="#2ac39b";ctx.globalAlpha=.25;
    const radius=Math.max(.45/view.scale,.009);
    for(let i=0;i<=frameIndex;i++){
      const frame=trace.frames[i],p=path[i];
      // Replay samples are downsampled again for accumulated-dot clarity.
      for(let k=0;k<frame.points_laser.length;k+=4){
        const at=worldLaser(p,frame.points_laser[k]);
        ctx.fillRect(at[0]-radius,at[1]-radius,2*radius,2*radius);
      }
    }
    ctx.globalAlpha=1;
  }
  function beams() {
    if(!el("fusion-hits").checked)return;
    const pose=poses.paths.fused_scan_matched[frameIndex];
    const frame=trace.frames[frameIndex];
    const origin=worldLaser(pose,[0,0]);
    ctx.lineWidth=.75/view.scale;ctx.strokeStyle="rgba(92,230,188,.3)";
    ctx.fillStyle="#66f2bd";
    for(const point of frame.points_laser) {
      const hit=worldLaser(pose,point);
      ctx.beginPath();ctx.moveTo(origin[0],origin[1]);
      ctx.lineTo(hit[0],hit[1]);ctx.stroke();
      ctx.fillRect(hit[0]-1.2/view.scale,hit[1]-1.2/view.scale,
                   2.4/view.scale,2.4/view.scale);
    }
  }
  function loopEdges() {
    if(!el("fusion-edges").checked)return;
    const path=poses.paths.graph_optimized||poses.paths.fused_scan_matched;
    ctx.lineWidth=2/view.scale;
    for(let i=0;i<=frameIndex;i++)
      for(const event of trace.frames[i].loop_events){
        const a=path[event.from],b=path[event.to];
        ctx.strokeStyle=event.decision==="accepted"?"#f7be6c":"#a35858";
        ctx.globalAlpha=event.decision==="accepted"?.9:.3;
        ctx.beginPath();ctx.moveTo(a[0],a[1]);ctx.lineTo(b[0],b[1]);ctx.stroke();
      }
    ctx.globalAlpha=1;
  }
  function draw() {
    const dpr=Math.min(window.devicePixelRatio||1,2);
    const rect=stage.getBoundingClientRect(),w=rect.width,h=rect.height;
    if(!w||!h||!canvas.width)return;
    ctx.setTransform(dpr,0,0,dpr,0,0);
    ctx.fillStyle="#0b1827";ctx.fillRect(0,0,w,h);
    if(!trace||!poses)return;
    ctx.setTransform(dpr*view.scale,0,0,-dpr*view.scale,
                     dpr*view.x,dpr*view.y);
    renderMap();dots();loopEdges();
    for(const key of Object.keys(poses.paths))
      if(visible(key))pathLayer(poses.paths[key],key);
    if(el("fusion-truth").checked)
      pathLayer(poses.synthetic_truth_for_scoring,"synthetic_truth_for_scoring");
    beams();
    ctx.setTransform(dpr,0,0,dpr,0,0);
    shown("fusion-scale",fmt(100/view.scale,1)+" cm per 100px");
  }
  async function selectMap(method) {
    const token=++imageToken;
    if(mapImage){URL.revokeObjectURL(mapImage.src);mapImage=null;}
    mapMeta=null;
    if(!method) {
      shown("fusion-map-note","Accumulated measured hit samples are not an occupancy grid. No final-map overlay selected.");
      draw();return;
    }
    try {
      const base=urlBase()+"/maps/"+encodeURIComponent(method);
      const [metadata,response]=await Promise.all([getJson(base),
        fetch(base+"/image.png",{cache:"no-store"})]);
      if(!response.ok)throw new Error("map image HTTP "+response.status);
      const blob=await response.blob();
      if(blob.size>5*1024*1024)throw new Error("map PNG exceeds visualization limit");
      const objectURL=URL.createObjectURL(blob);
      const image=new Image();
      await new Promise((resolve,reject)=>{
        image.onload=resolve;image.onerror=()=>reject(new Error("invalid map PNG"));
        image.src=objectURL;
      });
      if(token!==imageToken){URL.revokeObjectURL(objectURL);return;}
      mapMeta=metadata;mapImage=image;
      shown("fusion-map-note","Displayed map is the FINAL "+labels[method]+
        " occupancy snapshot. Points/trails stop at the selected frame; the image does not.");
      draw();
    }catch(error){
      if(token!==imageToken)return;
      showMessage("Final map unavailable: "+error.message);
      draw();
    }
  }
  async function loadRun(name) {
    stop();const token=++loadingToken;
    runId=name;report=null;poses=null;trace=null;
    imageToken++;mapImage=null;mapMeta=null;
    setReady(false);showMessage("");
    if(!name){shown("fusion-scene","—");draw();return;}
    try {
      const base=urlBase();
      const [r,p,t]=await Promise.all([
        getJson(base),getJson(base+"/trajectories"),getJson(base+"/replay")]);
      if(token!==loadingToken)return;
      if(r.frames!==t.frame_count || p.timestamps_ns.length!==t.frames.length)
        throw new Error("replay and report frame counts differ");
      report=r;poses=p;trace=t;
      for(const input of document.querySelectorAll("[data-path]")){
        const available=Object.hasOwn(poses.paths,input.dataset.path);
        input.closest("label").hidden=!available;
      }
      $map.replaceChildren(new Option("None — observation trail only",""));
      for(const method of methods())$map.add(new Option(labels[method]+" · final",method));
      $map.value="";frameIndex=0;setMetrics();setReady(true);
      el("fusion-slider").max=String(t.frames.length-1);
      resize();fit();setFrame(0);
      shown("fusion-map-note","Hit samples accumulate using the scan-matched estimated trajectory. They are NOT intermediate occupancy. Select a final map separately.");
    }catch(error){
      if(token!==loadingToken)return;
      showMessage("Could not load verified fusion run: "+error.message);
      setReady(false);draw();
    }
  }
  async function refresh() {
    stop();loadRun("");$run.replaceChildren(new Option("Scanning run directory…",""));
    try{
      const found=await getJson("/api/fusion/runs");
      $run.replaceChildren(new Option("Choose a fusion run",""));
      for(const item of found.runs)$run.add(new Option(item.name,item.id));
      if(!found.runs.length)showMessage("No fusion runs found. Pass --fusion-dir pointing to a directory containing slam_offline_fusion output directories.");
      else {$run.value=found.runs[0].id;await loadRun($run.value);}
    }catch(error){$run.replaceChildren(new Option("Unable to list runs",""));
      showMessage("Fusion run discovery failed: "+error.message);}
  }
  const frameDelta=()=>trace&&frameIndex<trace.frames.length-1?
    Math.max(.08,Math.min(2,(Number(trace.frames[frameIndex+1].time_ns)-
                            Number(trace.frames[frameIndex].time_ns))*1e-9)):.3;
  function tick(time){
    if(playing&&trace){
      if(lastTime){
        carry+=(time-lastTime)/1000*Number(el("fusion-speed").value);
        while(carry>=frameDelta()&&frameIndex<trace.frames.length-1) {
          carry-=frameDelta();setFrame(frameIndex+1);
        }
        if(frameIndex>=trace.frames.length-1)stop();
      }
      lastTime=time;
    }
    requestAnimationFrame(tick);
  }
  function togglePlay() {
    if(!trace)return;
    if(playing){stop();return;}
    if(frameIndex>=trace.frames.length-1)setFrame(0);
    playing=true;lastTime=0;carry=0;shown("fusion-play","❚❚ Pause");
  }
  $run.addEventListener("change",()=>loadRun($run.value));
  $map.addEventListener("change",()=>selectMap($map.value));
  el("fusion-refresh").addEventListener("click",refresh);
  el("fusion-fit").addEventListener("click",fit);
  el("fusion-play").addEventListener("click",togglePlay);
  el("fusion-back").addEventListener("click",()=>{stop();setFrame(frameIndex-1);});
  el("fusion-next").addEventListener("click",()=>{stop();setFrame(frameIndex+1);});
  el("fusion-slider").addEventListener("input",e=>{stop();setFrame(Number(e.target.value));});
  for(const input of document.querySelectorAll(".fusion-layers input"))
    input.addEventListener("change",draw);
  document.addEventListener("keydown",event=>{
    if(event.code==="Space"&&!["INPUT","SELECT","BUTTON","TEXTAREA"].includes(document.activeElement.tagName)){
      event.preventDefault();togglePlay();
    }
    if(event.code==="ArrowRight"&&document.activeElement===stage){
      stop();setFrame(frameIndex+1);event.preventDefault();
    }
    if(event.code==="ArrowLeft"&&document.activeElement===stage){
      stop();setFrame(frameIndex-1);event.preventDefault();
    }
  });
  stage.addEventListener("pointerdown",e=>{
    if(e.button!==0&&e.button!==1)return;
    drag={id:e.pointerId,x:e.clientX,y:e.clientY};
    stage.setPointerCapture(e.pointerId);
  });
  stage.addEventListener("pointermove",e=>{
    if(!drag||drag.id!==e.pointerId)return;
    view.x+=e.clientX-drag.x;view.y+=e.clientY-drag.y;
    drag.x=e.clientX;drag.y=e.clientY;draw();
  });
  stage.addEventListener("pointerup",()=>{drag=null;});
  stage.addEventListener("pointercancel",()=>{drag=null;});
  stage.addEventListener("wheel",e=>{
    e.preventDefault();
    const rect=stage.getBoundingClientRect(),x=e.clientX-rect.left,y=e.clientY-rect.top;
    const mult=Math.exp(-Math.max(-200,Math.min(200,e.deltaY))*.0015);
    const next=Math.max(2,Math.min(3000,view.scale*mult));
    const ratio=next/view.scale;
    view.x=x-(x-view.x)*ratio;view.y=y-(y-view.y)*ratio;
    view.scale=next;draw();
  },{passive:false});
  new ResizeObserver(resize).observe(stage);
  requestAnimationFrame(tick);
  refresh();
})();
