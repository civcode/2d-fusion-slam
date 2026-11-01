#include "slam/fusion/pipeline_profiler.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

void expect(bool condition,const char* message){
  if(!condition)throw std::runtime_error(message);
}
int main(){
  slam::fusion::PipelineProfiler profiler;
  for(std::size_t i=0;i<7;++i){
    slam::fusion::PipelineProfiler::Scope timer(profiler,"scan_deskew",i,true);
    // Exercise the timer with distinct observations, without coupling tests
    // to the wall clock resolution or scheduler under test.
    auto amount=std::sin(double(i)+.1);
    if(!std::isfinite(amount))throw std::runtime_error("math failure");
  }
  for(const auto& stage:{"sensor_integration","training_map_build",
                         "heldout_scan_match","fused_map_build","graph_optimize"}){
    slam::fusion::PipelineProfiler::Scope timer(profiler,stage);
  }
  const auto scan=profiler.stats("scan_deskew");
  if(slam::fusion::PipelineProfiler::enabled()){
    expect(scan.calls==7,"scan sample count");
    expect(scan.p50_ms<=scan.p95_ms,"percentile ordering");
    expect(scan.p95_ms<=scan.max_ms,"percentile maximum");
    expect(profiler.fusion_critical_ms()>=scan.total_ms,"critical sample total");
  }else {
    expect(scan.calls==0,"disabled instrumentation collected samples");
    expect(profiler.fusion_critical_ms()==0,"disabled overhead not zero");
  }
  std::ostringstream output;
  profiler.write_json(output,7,300.0,"geometry_reference_fallback","synthetic-apartment-v1");
  const auto body=output.str();
  expect(body.find("\"format\":\"FUSPROFILE1\"")!=std::string::npos,
         "missing JSON format");
  expect(body.find("\"realtime_capability_verified\":false")!=std::string::npos,
         "profile falsely claimed hardware readiness");
  if(slam::fusion::PipelineProfiler::enabled()){
    expect(body.find("\"effective_processing_fps\":null")==std::string::npos,
           "enabled profiler missing throughput");
  }else {
    expect(body.find("\"effective_processing_fps\":null")!=std::string::npos,
           "disabled profiler invented throughput");
  }
  std::cout<<"native fusion profiler instrumentation test passed\n";
}
