"use strict";
// Native/browser replay regression with only Node builtins. Optional in CTest:
// Docker's minimal test image intentionally does not install Node.
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const vm = require("node:vm");

async function main() {
  const source = fs.readFileSync(
    path.resolve(__dirname, "../../apps/demo/web/static/replay.js"), "utf8");
  let nextRaf;
  const requests = [];
  const ctx = new Proxy({}, {
    get(target, key) { return target[key] ?? (() => {}); },
    set(target, key, value) { target[key] = value; return true; }
  });
  const elements = new Map();
  function element(id) {
    if (!elements.has(id))elements.set(id, {
      value: id === "replay-speed" ? "4" : "",
      hidden: false, disabled: false, checked: false,
      textContent: "", innerHTML: "", width: 0, height: 0,
      addEventListener() {}, setAttribute() {}, append() {}, replaceChildren() {},
      getContext() { return ctx; },
      getBoundingClientRect() { return {width:1000, height:650, left:0, top:0}; },
      setPointerCapture() {}, hasPointerCapture() {return false;}
    });
    return elements.get(id);
  }
  const frames = Array.from({length:20}, (_, i) => ({
    index:i,t:100+i*.2,pose:[i*.2,0,0],odom:[i*.2,0,0],
    beam_count:360,hit_count:1,points:[[1,0]],
    matching:null,matching_state:"absent"
  }));
  const sha = "a".repeat(64);
  const meta = {
    name:"recording.slamreplay.json",format:"FUSREPLAY1",
    source:{source_sha256:sha,dataset:"mock CARMEN",
      frame_id:"fr079_odom_scan_frame_UNVERIFIED",
      pose_kind:"estimated_fr079_odometry_scan_matched_not_ground_truth",
      timestamp_kind:"CARMEN_IPC_scan_level"},
    ceres:{status:"absent"},frame_count:frames.length,
    first_t:frames[0].t,last_t:frames.at(-1).t,
    trajectory:frames.map(f => [f.t,...f.pose])
  };
  const fetch = async url => {
    requests.push(url);
    let value;
    if (url === "/api/replays")value = {sessions:[meta.name]};
    else if (url === "/api/maps")value = {maps:[]};
    else if (url === "/api/replays/" + meta.name)value = meta;
    else if (url === "/api/replays/" + meta.name + "/frames")
      value = {format:"FUSREPLAYFRAMES1",source_sha256:sha,frames};
    else throw Error("Unexpected per-frame or unauthorized browser request: " + url);
    return {ok:true, json:async () => value};
  };
  const scope = {
    document:{getElementById:element,createElement:() => element("option"),
      activeElement:{tagName:"DIV"}},
    window:{devicePixelRatio:1,addEventListener(){}},
    requestAnimationFrame(fn){ nextRaf=fn; },
    fetch,Image:class {},URL:{},
  };
  vm.runInNewContext(source +
    "\nglobalThis.driver={seek,setPlaying,get:()=>({index,playing,frame:frame?.index,loaded:scanFrames?.length})};",
    scope,{timeout:3000});
  for(let i=0;i<40;i++)await Promise.resolve();
  const driver=scope.driver;
  assert.equal(driver.get().loaded,20);
  assert.equal(driver.get().index,0);
  assert.equal(element("replay-trajectory-label").textContent,"Estimated SLAM trajectory");
  driver.setPlaying(true);
  for(let i=0;i<45;i++)nextRaf(1000+i*25);
  assert.ok(driver.get().index>0,"play must advance without the server");
  assert.ok(driver.get().frame>0,"rendered scan must advance");
  assert.ok(!requests.some(url => url.includes("/frame/")),
    "animation must not reread the full source log per scan");
  for(let i=45;i<140;i++)nextRaf(1000+i*25);
  assert.equal(driver.get().index,19,"play must reach the last scan");
  assert.equal(driver.get().playing,false,"play must stop at the end");
  driver.setPlaying(true);
  assert.equal(driver.get().index,0,"replay should restart from first scan");
  driver.setPlaying(false);
  driver.seek(17);
  assert.equal(driver.get().index,17,"manual seeking remains immediate");
  assert.equal(requests.length,4,"only session metadata and one frame batch fetched");
  console.log("Replay browser local-seek, play, restart and request-count regression passed");
}
main().catch(e => {console.error(e);process.exitCode=1;});
