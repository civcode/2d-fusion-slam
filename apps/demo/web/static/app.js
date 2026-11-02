"use strict";

const $ = (id) => document.getElementById(id);
const stage = $("map-stage");
const canvas = $("map-canvas");
const ctx = canvas.getContext("2d");
const number = (x) => Number(x).toLocaleString("en-US");
let maps = [];
let selected = "";
let metadata = null;
let mapImage = null;
let view = { scale: 1, x: 0, y: 0 };
let drag = null;
let chosenCell = null;
let requestVersion = 0;
const expandedRuns = new Set();
const lastRunSelection = new Map();
const METHOD_VARIANTS = Object.freeze([
  {id:"graph_optimized",label:"Graph optimized",
   description:"Map rebuilt from the optimized pose graph; loop closure is not guaranteed."},
  {id:"fused_scan_matched",label:"Scan-matched fusion",
   description:"Gyro + wheel poses corrected by accepted LiDAR scan matches."},
  {id:"gyro_wheel",label:"Gyro + wheel",
   description:"Fused motion estimate without scan-matching corrections."},
  {id:"lidar_only",label:"LiDAR only",
   description:"Independent LiDAR-only matching and mapping baseline."},
  {id:"wheel_only",label:"Wheel odometry",
   description:"Wheel-derived trajectory without LiDAR pose corrections."}
]);
const METHOD_ORDER = new Map(METHOD_VARIANTS.map((item,i)=>[item.id,i]));


function warning(text) {
  const el = $("warning");
  el.hidden = !text;
  el.textContent = text || "";
}
function text(id, value) { $(id).textContent = value; }
function setEmpty(visible, message) {
  $("map-empty").hidden = !visible;
  if (message) $("map-empty").querySelector("p").textContent = message;
  $("map-overlay").hidden = visible;
}
function updateStats() {
  const m = metadata;
  $("synthetic-note").hidden = !m || m.truth_class !== "synthetic_exact";
  $("fusion-map-note").hidden = !m || !selected.startsWith("fusion@");
  text("stat-size", m ? `${number(m.width)} × ${number(m.height)}` : "—");
  text("stat-resolution", m ? `${m.resolution_m * 100} cm per cell` : "Grid cells");
  text("stat-known", m ? number(m.known) : "—");
  text("stat-known-fraction", m ? `${(100 * m.known / (m.width * m.height)).toFixed(1)}% of grid observed` : "Coverage of available grid");
  text("stat-occupied", m ? number(m.occupied) : "—");
  text("stat-revision", m ? m.revision : "—");
  text("stat-verified", m ? "File checksum & fingerprint verified" : "No file loaded");
  text("meta-frame", m ? m.frame_id : "—");
  text("meta-source", m ? m.source_id : "—");
  text("meta-origin", m ? `${m.origin.x.toFixed(3)}, ${m.origin.y.toFixed(3)} m` : "—");
  text("meta-yaw", m ? `${m.origin.yaw_rad.toFixed(4)} rad` : "—");
  text("meta-fingerprint", m ? m.fingerprint : "—");
  text("meta-calibrated", m ? (m.metric_origin_verified ? "Claimed verified" : "Not verified") : "—");
  text("map-title", selectedMapTitle());
}
function methodDisplay(method) {
  return METHOD_VARIANTS.find(entry=>entry.id===method) ||
    {label:method.replaceAll("_"," ").replace(/\b\w/g,c=>c.toUpperCase()),
     description:"Additional saved map reconstruction method."};
}
function fusionIdentity(item) {
  if(item.role!=="fusion")return null;
  // Explicit fields from the native API; legacy listings are still accepted.
  const match=/^fusion@([A-Za-z0-9_-]+)@([A-Za-z0-9_]+)$/.exec(item.id);
  if(!match)return null;
  return {run:match[1],method:match[2],scene:item.scene||match[1]};
}
function sceneTitle(scene,run) {
  const source=String(scene||run).toLowerCase().split(/[-_]/);
  if(source.includes("apartment"))return "Apartment";
  if(source.includes("warehouse"))return "Warehouse";
  const readable=String(run).replace(/[-_]\d+$/,"").replace(/[-_]+/g," ").trim();
  return readable.replace(/\b\w/g,c=>c.toUpperCase())||run;
}
function groupMaps(items) {
  const runs=new Map(), standalone=[];
  for(const item of items) {
    const identity=fusionIdentity(item);
    if(!identity){standalone.push(item);continue;}
    if(!runs.has(identity.run))runs.set(identity.run,{
      run:identity.run,title:sceneTitle(identity.scene,identity.run),variants:[]});
    runs.get(identity.run).variants.push({...item,method:identity.method});
  }
  const groups=[...runs.values()];
  for(const group of groups)group.variants.sort((a,b)=>
    (METHOD_ORDER.get(a.method)??100)-(METHOD_ORDER.get(b.method)??100) ||
    a.method.localeCompare(b.method));
  groups.sort((a,b)=>a.title.localeCompare(b.title)||a.run.localeCompare(b.run));
  standalone.sort((a,b)=>a.name.localeCompare(b.name));
  return {groups,standalone};
}
function selectedMapTitle() {
  const item=maps.find(m=>m.id===selected);
  if(!item)return selected||"Occupancy grid";
  const identity=fusionIdentity(item);
  return identity
    ? sceneTitle(identity.scene,identity.run)+" · "+methodDisplay(identity.method).label
    : item.name;
}
function makeRadio(item,title,description) {
  const row=document.createElement("label");
  row.className="map-variant"+(selected===item.id?" selected":"");
  const radio=document.createElement("input");
  radio.type="radio";radio.name="map-selection";radio.value=item.id;
  radio.checked=selected===item.id;
  radio.addEventListener("change",()=>{if(radio.checked)selectMap(item.id);});
  const content=document.createElement("span");
  content.className="map-variant-copy";
  const heading=document.createElement("span");
  heading.className="variant-name";heading.textContent=title;
  const details=document.createElement("span");
  details.className="variant-description";details.textContent=description;
  content.append(heading,details);row.append(radio,content);
  return row;
}
function paintList() {
  const list=$("map-list");
  list.replaceChildren();
  if(!maps.length) {
    const p=document.createElement("div");p.className="list-placeholder";
    p.textContent="No .slammap exports in this directory yet.";
    list.append(p);return;
  }
  const {groups,standalone}=groupMaps(maps);
  for(const group of groups){
    const container=document.createElement("details");
    container.className="scene-group";
    container.open=expandedRuns.has(group.run)||
      group.variants.some(m=>m.id===selected);
    const summary=document.createElement("summary");
    summary.className="scene-summary";
    const name=document.createElement("span");
    name.className="scene-title";name.textContent=group.title;
    const note=document.createElement("span");
    note.className="scene-run";note.textContent=group.run;
    const header=document.createElement("span");
    header.className="scene-summary-copy";header.append(name,note);
    const count=document.createElement("span");
    count.className="scene-method-count";
    count.textContent=group.variants.length+" methods";
    summary.append(header,count);
    // Preserve expanded runs when rebuilding radios after a selection.
    summary.addEventListener("click",event=>{
      event.preventDefault();
      if(expandedRuns.has(group.run)){
        expandedRuns.delete(group.run);
        paintList();
      }else{
        expandedRuns.add(group.run);
        const preferred=lastRunSelection.get(group.run);
        const target=group.variants.find(m=>m.id===preferred)||group.variants[0];
        if(!group.variants.some(m=>m.id===selected))selectMap(target.id);
        else paintList();
      }
    });
    container.append(summary);
    const options=document.createElement("div");
    options.className="scene-variants";
    for(const item of group.variants){
      const method=methodDisplay(item.method);
      options.append(makeRadio(item,method.label,method.description));
    }
    container.append(options);list.append(container);
  }
  if(standalone.length){
    const section=document.createElement("section");
    section.className="standalone-maps";
    const heading=document.createElement("h3");
    heading.textContent="Other maps";
    section.append(heading);
    for(const item of standalone){
      const role=item.role==="active"?"Published active map":
                 item.role==="previous"?"Previous map generation":
                 item.id.startsWith("sample-")?"Synthetic example map":
                 "Standalone occupancy map";
      section.append(makeRadio(item,item.name,role));
    }
    list.append(section);
  }
}
async function api(url) {
  const res = await fetch(url, {cache: "no-store"});
  const body = await res.json();
  if (!res.ok) throw new Error(body.error || `HTTP ${res.status}`);
  return body;
}
async function refresh() {
  try {
    const index = await api("/api/maps");
    maps = index.maps;
    const registry = index.registry;
    if (registry.status === "invalid" || registry.status === "degraded") {
      warning(registry.error || "The map generation registry failed validation.");
    } else {
      warning("");
    }
    const preferred = registry.active?.integrity === "valid" ? registry.active.file : "";
    if (!selected || !maps.some((m) => m.id === selected)) {
      selected = "";
      const requested = new URLSearchParams(window.location.search).get("map");
      const {groups} = groupMaps(maps);
      // Explicit deep links take priority. A published active generation
      // retains precedence; other scenes default to their highest-ranked method.
      const chosen = maps.find(m => m.id === requested) ||
        maps.find(m => m.id === preferred) ||
        groups[0]?.variants[0] || maps[0];
      if(chosen)await selectMap(chosen.id);
      else { metadata = null; mapImage = null; updateStats(); setEmpty(true); render(); }
    }
    paintList();
  } catch (err) {
    warning(`Cannot read map directory: ${err.message}`);
    setEmpty(true, "Failed to read the map directory. Check the server console.");
  }
}
async function selectMap(id) {
  const version = ++requestVersion;
  selected = id;
  const identity=fusionIdentity(maps.find(m=>m.id===id)||{});
  if(identity){
    lastRunSelection.set(identity.run,id);
    expandedRuns.add(identity.run);
  }
  chosenCell = null;
  $("cell-prompt").hidden = false;
  $("cell-details").hidden = true;
  metadata = null;
  mapImage = null;
  updateStats();
  setEmpty(true, "Validating map integrity and preparing raster…");
  paintList();
  try {
    const m = await api(`/api/maps/${encodeURIComponent(id)}`);
    const imageResponse = await fetch(`/api/maps/${encodeURIComponent(id)}/image.png`, {cache: "no-store"});
    if (!imageResponse.ok) throw new Error(`Map raster refused (HTTP ${imageResponse.status})`);
    const imageUrl = URL.createObjectURL(await imageResponse.blob());
    const img = new Image();
    try {
      await new Promise((resolve, reject) => {
        img.onload = resolve;
        img.onerror = () => reject(new Error("Map raster could not be decoded"));
        img.src = imageUrl;
      });
    } finally {
      URL.revokeObjectURL(imageUrl);
    }
    if (version !== requestVersion) return;
    metadata = m;
    mapImage = img;
    updateStats();
    setEmpty(false);
    fitMap();
  } catch (err) {
    if (version !== requestVersion) return;
    warning(`Map “${id}” refused: ${err.message}`);
    setEmpty(true, "The map failed validation. Select another export or repair the data.");
  }
}
function stageSize() {
  const rect = stage.getBoundingClientRect();
  return { width: rect.width, height: rect.height };
}
function syncCanvas() {
  const {width, height} = stageSize();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  render();
}
function fitMap() {
  if (!mapImage) return;
  const {width, height} = stageSize();
  view.scale = Math.min((width - 54) / mapImage.width, (height - 54) / mapImage.height);
  view.scale = Math.max(view.scale, 0.025);
  view.x = (width - mapImage.width * view.scale) / 2;
  view.y = (height - mapImage.height * view.scale) / 2;
  render();
}
function render() {
  const {width, height} = stageSize();
  ctx.clearRect(0, 0, width, height);
  if (mapImage) {
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapImage, view.x, view.y, mapImage.width * view.scale, mapImage.height * view.scale);
    ctx.strokeStyle = "rgba(77, 211, 184, .65)";
    ctx.lineWidth = 1;
    ctx.strokeRect(view.x + .5, view.y + .5, mapImage.width * view.scale, mapImage.height * view.scale);
    if (chosenCell && metadata) {
      const cx = view.x + (chosenCell.x + .5) / metadata.width * mapImage.width * view.scale;
      const cy = view.y + (metadata.height - chosenCell.y - .5) / metadata.height * mapImage.height * view.scale;
      ctx.strokeStyle = "#ffca7a";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(cx, cy, 9, 0, Math.PI * 2);
      ctx.moveTo(cx - 15, cy); ctx.lineTo(cx + 15, cy);
      ctx.moveTo(cx, cy - 15); ctx.lineTo(cx, cy + 15);
      ctx.stroke();
    }
  }
  text("zoom-readout", mapImage ? `${Math.round(view.scale * 100)}%` : "—");
}
function zoomAt(amount, clientX, clientY) {
  if (!mapImage) return;
  const rect = stage.getBoundingClientRect();
  const px = clientX - rect.left, py = clientY - rect.top;
  const next = Math.min(40, Math.max(0.025, view.scale * amount));
  const ratio = next / view.scale;
  view.x = px - (px - view.x) * ratio;
  view.y = py - (py - view.y) * ratio;
  view.scale = next;
  render();
}
function hitTest(clientX, clientY) {
  if (!metadata || !mapImage) return null;
  const rect = stage.getBoundingClientRect();
  const ux = (clientX - rect.left - view.x) / (mapImage.width * view.scale);
  const uy = (clientY - rect.top - view.y) / (mapImage.height * view.scale);
  if (ux < 0 || uy < 0 || ux >= 1 || uy >= 1) return null;
  return {x: Math.min(metadata.width - 1, Math.floor(ux * metadata.width)),
          y: metadata.height - 1 - Math.floor(uy * metadata.height)};
}
async function inspectCell(point) {
  if (!point || !selected) return;
  chosenCell = point;
  render();
  const id = selected;
  try {
    const result = await api(`/api/maps/${encodeURIComponent(id)}/cell?x=${point.x}&y=${point.y}`);
    if (selected !== id || !chosenCell || point.x !== chosenCell.x || point.y !== chosenCell.y) return;
    text("cell-xy", `${result.x}, ${result.y}`);
    text("cell-world", `${result.world_x.toFixed(3)}, ${result.world_y.toFixed(3)} m`);
    text("cell-observed", result.observed ? "Yes" : "No — unknown");
    text("cell-prob", result.observed ? `${(100 * result.occupied_probability).toFixed(1)}%` : "Not available");
    text("cell-logodds", result.observed ? result.log_odds.toFixed(4) : "—");
    $("cell-details").hidden = false;
    $("cell-prompt").hidden = true;
  } catch (err) { warning(`Cell inspection failed: ${err.message}`); }
}

$("refresh").addEventListener("click", refresh);
$("fit").addEventListener("click", fitMap);
$("zoom-in").addEventListener("click", () => { const b = stage.getBoundingClientRect(); zoomAt(1.4, b.left + b.width/2, b.top + b.height/2); });
$("zoom-out").addEventListener("click", () => { const b = stage.getBoundingClientRect(); zoomAt(1/1.4, b.left + b.width/2, b.top + b.height/2); });
stage.addEventListener("wheel", (event) => {
  if (!mapImage) return;
  event.preventDefault();
  // A held middle button is a pan, not zoom. Horizontal-only wheel input
  // (deltaY === 0) must not be mistaken for a downward wheel step either.
  if (drag?.button === 1 || (event.buttons & 4) || event.deltaY === 0) return;
  zoomAt(event.deltaY < 0 ? 1.14 : 1/1.14, event.clientX, event.clientY);
}, {passive:false});
// Cancel native middle-button autoscroll/clipboard paste on both event paths.
for (const type of ["mousedown", "auxclick"]) {
  stage.addEventListener(type, (event) => { if (event.button === 1) event.preventDefault(); });
}
stage.addEventListener("pointerdown", (event) => {
  if (event.button === 1) event.preventDefault();
  if (!mapImage || drag || (event.button !== 0 && event.button !== 1)) return;
  drag = {pointerId:event.pointerId, button:event.button,
          startX:event.clientX, startY:event.clientY, x:view.x, y:view.y, moved:false};
  stage.setPointerCapture(event.pointerId);
});
stage.addEventListener("pointermove", (event) => {
  if (!drag || event.pointerId !== drag.pointerId) return;
  const dx=event.clientX-drag.startX, dy=event.clientY-drag.startY;
  if (Math.abs(dx)+Math.abs(dy)>5) drag.moved=true;
  if (drag.moved) {view.x=drag.x+dx; view.y=drag.y+dy; render();}
});
function endPan(event) {
  if (!drag || event.pointerId !== drag.pointerId) return;
  const inspect = event.type === "pointerup" && drag.button === 0 && !drag.moved;
  drag = null;
  if (stage.hasPointerCapture(event.pointerId)) stage.releasePointerCapture(event.pointerId);
  if (inspect) inspectCell(hitTest(event.clientX, event.clientY));
}
for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
  stage.addEventListener(type, endPan);
}
stage.addEventListener("keydown", (event) => { if (event.key.toLowerCase() === "f") {fitMap(); event.preventDefault();} });
window.addEventListener("resize", syncCanvas);
syncCanvas();
refresh();
