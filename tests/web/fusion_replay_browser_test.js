"use strict";
const assert=require("node:assert/strict");
const fs=require("node:fs");
const path=require("node:path");
const vm=require("node:vm");

// Browser-light smoke: actual fusion.js is exercised with DOM/canvas stubs,
// no npm packages, sockets, GitHub Actions or physical robot interactions.
const script=fs.readFileSync(path.resolve(
    __dirname,"../../apps/demo/web/static/fusion.js"),"utf8");

function dataset(withRaw=true) {
  const frame=(index)=>({
    index,time_ns:String(1000000000+300000000*index),
    hits:3,beam_count:240,
    points_laser:[[2,0],[0,2]],
    ...(withRaw?{raw_points_laser:[[3,0],[0,3]]}:{}),
    matching:{attempted:index===1,status:index===1?"accepted":"training",
      reason:index===1?"accepted":"not_evaluated_training_scan",
      supported_hits:2,input_hits:2,overlap:.5,
      cumulative_accepted:index,cumulative_rejected:0},
    mapping:{lidar_only_inserted:true},loop_events:[]
  });
  const poses={
    format:"FUSPOSES1",
    paths:{
      wheel_only:[[2,2,0],[2.2,2,0]],
      lidar_only:[[2,2,0],[2.3,2,0]],
      gyro_wheel:[[2,2,0],[2.4,2,0]],
      fused_scan_matched:[[2,2,0],[2.5,2,0]],
      graph_optimized:[[2,2,0],[2.7,2,0]]
    },
    synthetic_truth_for_scoring:[[2,2,0],[2.3,2,0]],
    timestamps_ns:["1000000000","1300000000"]
  };
  const map={metrics:{occupied_iou:.5,known_coverage:.2}};
  const results=Object.fromEntries(Object.keys(poses.paths).map(m=>[
    m,{translation_rmse_m:.1,map}
  ]));
  return {
    report:{scene:"synthetic-office-v1",frames:2,matcher:"ceres",
      heldout_accepted:1,heldout_attempted:1,graph_accepted_loops:0,
      graph_loop_candidates:0,results},
    poses,
    trace:{
      frame_count:2,frames:[frame(0),frame(1)],
      base_from_laser_yaw_rad:.02,laser_from_base_translation_m:[.12,.01]
    }
  };
}
function dom() {
  const nodes=new Map();
  const noop=()=>{};
  const ctx=new Proxy({}, {get:(obj,key)=>obj[key]??noop,
    set:(obj,key,val)=>{obj[key]=val;return true;}});
  function element(id) {
    if(!nodes.has(id)){
      const obj={id,hidden:false,disabled:false,textContent:"",value:"",
        checked:id==="fusion-hits"||id==="fusion-history",
        width:0,height:0,style:{},handlers:{},children:[],options:[],
        append(...v){this.children.push(...v);},
        add(o){this.children.push(o);this.options.push(o);},
        replaceChildren(...v){this.children=v;},
        addEventListener(type,cb){this.handlers[type]=cb;},
        getBoundingClientRect(){return {width:900,height:500,left:0,top:0};},
        getContext(){return ctx;},
        setPointerCapture(){},focus(){},
        querySelector(query){
          if(id==="fusion-view"&&query==='option[value="lidar"]')
            return this.rawOption||(this.rawOption={disabled:false});
          return null;
        }
      };
      nodes.set(id,obj);
    }
    return nodes.get(id);
  }
  const methods=["wheel_only","lidar_only","gyro_wheel","fused_scan_matched","graph_optimized"];
  const pathInputs=methods.map(name=>({
    dataset:{path:name},checked:name==="fused_scan_matched",
    label:{hidden:name==="graph_optimized"},handlers:{},
    closest(){return this.label;},
    addEventListener(type,cb){this.handlers[type]=cb;}
  }));
  element("fusion-view").value="slam";
  element("fusion-projection").value="fused_scan_matched";
  element("fusion-projection").options=[
    {value:"fused_scan_matched",disabled:false},
    {value:"graph_optimized",disabled:false},
    {value:"gyro_wheel",disabled:false}
  ];
  const document={
    getElementById:element,
    querySelector(query) {
      const match=/^input\[data-path="([^"]+)"\]$/.exec(query);
      return match?pathInputs.find(input=>input.dataset.path===match[1]):null;
    },
    querySelectorAll(query) {
      if(query==="[data-path]")return pathInputs;
      if(query===".fusion-layers input")return pathInputs;
      return [];
    },
    createElement(tag){return {tagName:tag,style:{},children:[],
      append(...v){this.children.push(...v);},
      replaceChildren(...v){this.children=v;}};},
    activeElement:{tagName:"BODY"},
    handlers:{},
    addEventListener(type,cb){this.handlers[type]=cb;}
  };
  return {element,document,ctx};
}
async function launch(withRaw,query) {
  const {element,document,ctx}=dom();
  const {report,poses,trace}=dataset(withRaw);
  const urls=[];
  const fetch=async url=>{
    urls.push(url);
    const value=url==="/api/fusion/runs"?
      {runs:[{id:"office-continuous-s01",name:"office-continuous-s01"}]}:
      url.endsWith("/trajectories")?poses:
      url.endsWith("/replay")?trace:
      url.endsWith("/profile")?null:report;
    return {ok:value!==null,status:value===null?404:200,
      json:async()=>value};
  };
  const scope={
    document,window:{location:{search:query},devicePixelRatio:1},
    fetch,Option:function(label,value){this.label=label;this.value=value;},
    URLSearchParams,URL:{revokeObjectURL(){}},ResizeObserver:class {
      observe(){}
    },
    requestAnimationFrame(){},
    Image:class{}
  };
  const entry=script.replace(/\}\)\(\);\s*$/,
    ';globalThis.__test={scanPoints,scanPose,setView,setFrame,loadRun};})();');
  assert.notEqual(entry,script,"test injection anchor must exist");
  vm.runInNewContext(entry,scope,{timeout:5000});
  for(let i=0;i<100;i++)await Promise.resolve();
  return {...scope.__test,element,ctx,urls,trace};
}
async function main() {
  const raw=await launch(true,"?run=office-continuous-s01&view=lidar");
  assert.equal(raw.element("fusion-runs").value,"office-continuous-s01");
  assert.equal(raw.element("fusion-view").value,"lidar");
  assert.equal(raw.element("fusion-scan-kind").textContent,"Raw, not deskewed");
  assert.equal(raw.scanPoints(raw.trace.frames[0])[0][0],3);
  assert.equal(raw.scanPose()[1][0],2.3);
  assert.match(raw.element("fusion-view-note").textContent,/estimated LiDAR-only/);
  raw.setView("slam");
  assert.equal(raw.element("fusion-view").value,"slam");
  assert.equal(raw.scanPoints(raw.trace.frames[0])[0][0],2);
  assert.equal(raw.scanPose()[1][0],2.5);
  assert.equal(raw.element("fusion-scan-kind").textContent,"Gyro/wheel deskewed");
  raw.element("fusion-projection").value="graph_optimized";
  raw.element("fusion-projection").handlers.change();
  assert.equal(raw.scanPose()[1][0],2.7,
    "graph reprojected hit trail must use optimized pose, not the fused path");
  assert.match(raw.element("fusion-view-note").textContent,/Graph optimized/);
  raw.setFrame(1);
  assert.match(raw.element("fusion-pose-shift").textContent,/0.200 m/);
  raw.setView("lidar");
  assert.equal(raw.scanPose()[1][0],2.3,
    "raw LiDAR always uses LiDAR-only poses, independent of SLAM projection");

  const legacy=await launch(false,"?run=office-continuous-s01&view=lidar");
  assert.equal(legacy.element("fusion-view").value,"lidar");
  assert.equal(legacy.element("fusion-scan-kind").textContent,
    "Legacy deskewed samples");
  assert.equal(legacy.scanPoints(legacy.trace.frames[0])[0][0],2);
  assert.equal(legacy.scanPose()[1][0],2.5,
    "legacy deskewed points must not be drawn at a LiDAR-only pose");
  assert.match(legacy.element("fusion-view-note").textContent,
    /raw hits were not exported/);
  console.log("Synthetic LiDAR/SLAM replay modes, deep links and legacy fallback passed");
}
main().catch(error=>{console.error(error);process.exitCode=1;});
