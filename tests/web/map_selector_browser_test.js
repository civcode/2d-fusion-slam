"use strict";
// Browser DOM smoke test for scene-first fusion maps. Runs locally with Node,
// no npm, browser package, server, network, or GitHub Actions required.
const assert=require("node:assert/strict");
const fs=require("node:fs");
const path=require("node:path");
const vm=require("node:vm");

const app=fs.readFileSync(
  path.resolve(__dirname,"../../apps/demo/web/static/app.js"),"utf8");
const order=[
  "graph_optimized","fused_scan_matched","gyro_wheel","lidar_only","wheel_only"
];
function fixture() {
  const maps=[];
  for(const method of [...order].reverse())
    maps.push({
      id:"fusion@apartment-001@"+method,
      name:"apartment-001 / "+method,role:"fusion",
      run_id:"apartment-001",scene:"synthetic-apartment-v1",method
    });
  for(const method of ["wheel_only","lidar_only","gyro_wheel","fused_scan_matched"])
    maps.push({
      id:"fusion@warehouse-001@"+method,
      name:"warehouse-001 / "+method,role:"fusion",
      run_id:"warehouse-001",scene:"synthetic-warehouse-v1",method
    });
  maps.push({id:"fusion@apartment-002@wheel_only",
    name:"apartment-002 / wheel_only",role:"fusion",
    run_id:"apartment-002",scene:"synthetic-apartment-v1",method:"wheel_only"});
  maps.push({id:"fr079-slam.slammap",name:"fr079-slam.slammap",role:"file"});
  maps.push({id:"sample-synthetic-workshop.slammap",
    name:"sample-synthetic-workshop.slammap",role:"file"});
  return maps;
}
function makeDOM() {
  const nodes=new Map();
  function node(id){
    if(!nodes.has(id)){
      const el={id,children:[],handlers:{},hidden:false,checked:false,
        textContent:"",value:"",className:"",style:{},width:0,height:0,
        append(...children){this.children.push(...children);},
        replaceChildren(...children){this.children=children;},
        addEventListener(name,callback){this.handlers[name]=callback;},
        getBoundingClientRect(){return {width:1000,height:600,left:0,top:0};},
        getContext(){return ctx;},
        querySelector(){return {textContent:""};},
        setAttribute(){},setPointerCapture(){},hasPointerCapture(){return false;}
      };
      nodes.set(id,el);
    }
    return nodes.get(id);
  }
  const ctx=new Proxy({},{get(target,key){return target[key]??function(){}},
    set(target,key,value){target[key]=value;return true;}});
  return {node,document:{
    getElementById:node,
    createElement:tag=>{
      // Distinct DOM nodes, unlike known-ID lookups.
      const el={tagName:tag,children:[],handlers:{},open:false,
        append(...children){this.children.push(...children);},
        replaceChildren(...children){this.children=children;},
        addEventListener(name,callback){this.handlers[name]=callback;},
        setAttribute(){}};
      return el;
    }
  }};
}
async function launch(search="",indexOptions={status:"absent",active:{}}) {
  const {node,document}=makeDOM(),calls=[];
  const listed=fixture();
  const data={
    width:120,height:90,resolution_m:.1,known:1000,occupied:100,
    revision:"5",fingerprint:"12345",
    truth_class:"synthetic_estimated",frame_id:"map",
    source_id:"synthetic-warehouse-v1",
    metric_origin_verified:false,origin:{x:0,y:0,yaw_rad:0}
  };
  const fetch=async url=>{
    calls.push(url);
    if(url==="/api/maps")return {ok:true,json:async()=>({
      maps:listed,registry:indexOptions
    })};
    if(url.endsWith("/image.png"))
      return {ok:true,blob:async()=>({bytes:[]})};
    if(url.startsWith("/api/maps/"))
      return {ok:true,json:async()=>data};
    throw Error("Unexpected endpoint "+url);
  };
  class Image {
    constructor(){this.width=120;this.height=90;}
    set src(value){this.url=value;queueMicrotask(()=>this.onload?.());}
  }
  const scope={
    document,
    window:{devicePixelRatio:1,location:{search},addEventListener(){}},
    fetch,Image,URL:{createObjectURL:()=>"blob:map",
                    revokeObjectURL(){}},
    URLSearchParams
  };
  vm.runInNewContext(app+
    "\nglobalThis.driver={groupMaps,methodDisplay,sceneTitle,selectMap,refresh,"+
    "paintList,selected:()=>selected,expanded:()=>[...expandedRuns],"+
    "getLast:run=>lastRunSelection.get(run)};",
    scope,{timeout:3000});
  for(let i=0;i<60;i++)await Promise.resolve();
  return {...scope.driver,node,calls,listed};
}
function radio(rows,id){
  for(const row of rows){
    if(row.tagName==="label"&&row.children[0]?.value===id)
      return row.children[0];
    const result=radio(row.children||[],id);
    if(result)return result;
  }
  return null;
}
async function settle(){for(let i=0;i<60;i++)await Promise.resolve();}
async function main(){
  const a=await launch();
  const groups=a.groupMaps(a.listed);
  assert.equal(groups.groups.length,3);
  assert.deepEqual(Array.from(groups.groups,x=>x.run),
    ["apartment-001","apartment-002","warehouse-001"]);
  assert.deepEqual(Array.from(groups.groups[0].variants,x=>x.method),order);
  assert.deepEqual(Array.from(groups.groups[2].variants,x=>x.method),
    order.slice(1));
  assert.equal(groups.standalone.length,2);
  assert.equal(a.selected(),"fusion@apartment-001@graph_optimized");
  const nodes=a.node("map-list").children;
  assert.equal(nodes.length,4,"three scene groups and an Other maps section");
  assert.equal(nodes[0].open,true);
  assert.equal(nodes[1].open,false);
  assert.equal(nodes[2].open,false);
  assert.equal(nodes[0].children[0].children[0].children[0].textContent,
    "Apartment");
  assert.equal(nodes[0].children[0].children[0].children[1].textContent,
    "apartment-001");
  assert.equal(a.node("map-title").textContent,"Apartment · Graph optimized");
  assert.equal(radio(nodes,"fusion@apartment-001@graph_optimized").checked,true);
  assert.equal(radio(nodes,"fusion@apartment-001@lidar_only").checked,false);
  assert.equal(a.methodDisplay("graph_optimized").label,"Graph optimized");

  // Opening a run chooses its best *available* variant, not an absent graph.
  nodes[2].children[0].handlers.click({preventDefault(){}});
  await settle();
  assert.equal(a.selected(),"fusion@warehouse-001@fused_scan_matched");
  assert.equal(a.node("map-title").textContent,"Warehouse · Scan-matched fusion");
  const warehouse=a.node("map-list").children;
  assert.equal(warehouse[0].open,false);
  assert.equal(warehouse[2].open,true);
  const lidar=radio(warehouse,"fusion@warehouse-001@lidar_only");
  assert.ok(lidar,"must show LiDAR-only method radio");
  lidar.checked=true;lidar.handlers.change();
  await settle();
  assert.equal(a.selected(),"fusion@warehouse-001@lidar_only");
  assert.equal(a.node("map-title").textContent,"Warehouse · LiDAR only");
  assert.equal(a.getLast("warehouse-001"),a.selected());

  // Going to another run then back should recall its last selection.
  a.node("map-list").children[0].children[0].handlers.click({preventDefault(){}});
  await settle();
  a.node("map-list").children[2].children[0].handlers.click({preventDefault(){}});
  await settle();
  assert.equal(a.selected(),"fusion@warehouse-001@lidar_only");
  assert.ok(a.calls.some(url=>url.includes("fusion%40warehouse-001%40lidar_only")),
    "HTTP must retain the original unchanged canonical map ID");

  // A standalone map stays accessible as its own radio, never classified as
  // one of the synthetic-scene SLAM methods.
  const standalone=radio(a.node("map-list").children,"fr079-slam.slammap");
  assert.ok(standalone);
  standalone.checked=true;standalone.handlers.change();
  await settle();
  assert.equal(a.selected(),"fr079-slam.slammap");
  assert.equal(a.node("map-title").textContent,"fr079-slam.slammap");
  assert.ok(a.node("map-list").children.slice(0,3).every(el=>!el.open));

  const deep=await launch("?map=fusion%40warehouse-001%40wheel_only");
  assert.equal(deep.selected(),"fusion@warehouse-001@wheel_only");
  assert.equal(deep.node("map-title").textContent,"Warehouse · Wheel odometry");
  assert.equal(deep.node("map-list").children[2].open,true);

  const published=await launch("",{
    status:"valid",active:{file:"fr079-slam.slammap",integrity:"valid"}
  });
  assert.equal(published.selected(),"fr079-slam.slammap",
    "published active pointer retains default priority");
  const overwritten=await launch("?map=fusion%40apartment-002%40wheel_only",{
    status:"valid",active:{file:"fr079-slam.slammap",integrity:"valid"}
  });
  assert.equal(overwritten.selected(),"fusion@apartment-002@wheel_only",
    "explicit map deep link overrides published pointer");
  console.log("Scene/method radio grouping, defaults, standalone, deep links and selection passed");
}
main().catch(e=>{console.error(e);process.exitCode=1;});
