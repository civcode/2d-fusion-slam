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
  text("map-title", selected || "Occupancy grid");
}
function mapItem(item) {
  const b = document.createElement("button");
  b.type = "button";
  b.className = "map-item" + (item.id === selected ? " selected" : "");
  b.setAttribute("aria-pressed", String(item.id === selected));
  const icon = document.createElement("span");
  icon.className = "file-icon";
  icon.textContent = "▦";
  const names = document.createElement("span");
  names.className = "file-label";
  const title = document.createElement("span");
  title.className = "file-name";
  title.textContent = item.name;
  const role = document.createElement("span");
  role.className = "file-meta" + (item.role === "active" ? " active-label" : "");
  role.textContent = item.role === "file" ? "STANDALONE MAP" : item.role.toUpperCase() + " GENERATION";
  names.append(title, role);
  b.append(icon, names);
  b.addEventListener("click", () => selectMap(item.id));
  return b;
}
function paintList() {
  const list = $("map-list");
  list.replaceChildren();
  if (!maps.length) {
    const p = document.createElement("div");
    p.className = "list-placeholder";
    p.textContent = "No .slammap exports in this directory yet.";
    list.append(p);
  }
  maps.forEach((item) => list.append(mapItem(item)));
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
      if (preferred && maps.some((m) => m.id === preferred)) await selectMap(preferred);
      else if (maps.length) await selectMap(maps[0].id);
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
