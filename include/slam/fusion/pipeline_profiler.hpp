#pragma once
// Portable, low-overhead, *optional* steady-clock instrumentation for offline
// fusion. Stage measurements are wall time, not CPU cycles or hardware timing.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <map>
#include <numeric>
#include <ostream>
#include <string>
#include <vector>

namespace slam::fusion {
class PipelineProfiler {
 public:
  using Clock=std::chrono::steady_clock;
  struct Observation { double ms{}; std::size_t frame{}; bool per_frame{}; };
  class Scope {
   public:
    Scope(PipelineProfiler& owner,std::string stage,std::size_t frame=0,
          bool per_frame=false)
      :owner_(owner),stage_(std::move(stage)),frame_(frame),per_frame_(per_frame)
#ifdef SLAM_PROFILE_FUSION
      ,started_(Clock::now())
#endif
    {}
    Scope(const Scope&)=delete;
    Scope& operator=(const Scope&)=delete;
    ~Scope(){
#ifdef SLAM_PROFILE_FUSION
      const auto end=Clock::now();
      const double elapsed=std::chrono::duration<double,std::milli>(end-started_).count();
      owner_.samples_[stage_].push_back({elapsed,frame_,per_frame_});
#endif
    }
   private:
    PipelineProfiler& owner_;
    std::string stage_;
    std::size_t frame_{};
    bool per_frame_{};
#ifdef SLAM_PROFILE_FUSION
    Clock::time_point started_;
#endif
  };
  [[nodiscard]] static constexpr bool enabled() {
#ifdef SLAM_PROFILE_FUSION
    return true;
#else
    return false;
#endif
  }
  struct Stats {
    std::size_t calls{};
    double total_ms{},mean_ms{},p50_ms{},p95_ms{},max_ms{};
  };
  [[nodiscard]] Stats stats(const std::string& name) const {
    const auto iter=samples_.find(name);
    if(iter==samples_.end()||iter->second.empty())return {};
    std::vector<double> values;
    double total=0;
    values.reserve(iter->second.size());
    for(const auto& value:iter->second){values.push_back(value.ms);total+=value.ms;}
    std::sort(values.begin(),values.end());
    const auto pick=[&](double q) {
      const auto at=static_cast<std::size_t>(std::ceil(q*values.size()))-1;
      return values[at];
    };
    return {values.size(),total,total/values.size(),pick(.5),pick(.95),values.back()};
  }
  // CPU/OS contention, caches and active instrumentation change timing.
  // This *offline critical path proxy* deliberately excludes generation,
  // LiDAR-only and wheel-only baselines, scoring and disk serialization.
  [[nodiscard]] double fusion_critical_ms() const {
    double total=0;
    for(const auto* stage:{"sensor_integration","scan_deskew",
                            "training_map_build","heldout_scan_match",
                            "fused_map_build","graph_optimize"}) {
      total+=stats(stage).total_ms;
    }
    return total;
  }
  void write_json(std::ostream& out,std::size_t frames,double scan_period_ms,
                  const std::string& matcher,const std::string& scene) const {
    out<<std::setprecision(12);
    out<<"{\n\"format\":\"FUSPROFILE1\","
       <<"\n\"scene\":\""<<scene<<"\","
       <<"\n\"matcher\":\""<<matcher<<"\","
       <<"\n\"profiled\":"<<(enabled()?"true":"false")<<","
       <<"\n\"clock\":\"std::chrono::steady_clock\","
       <<"\n\"units\":\"wall_milliseconds\","
       <<"\n\"frames\":"<<frames<<","
       <<"\n\"scan_period_ms\":"<<scan_period_ms<<","
       <<"\n\"input_rate_fps\":"<<(1000./scan_period_ms)<<","
       <<"\n\"realtime_capability_verified\":false,"
       <<"\n\"critical_path_excludes\":\"synthetic_input_generation,baselines,evaluation,artifact_io\","
       <<"\n\"critical_path_includes\":\"sensor_integration,scan_deskew,training_map_build,heldout_scan_match,fused_map_build,graph_optimize\","
       <<"\n\"stage_samples_nested\":true,"
       <<"\n\"stages\":{\n";
    bool first=true;
    for(const auto& [name,values]:samples_){
      (void)values;
      if(!first)out<<",\n";first=false;
      const auto s=stats(name);
      out<<"\""<<name<<"\":{\"calls\":"<<s.calls
         <<",\"total_ms\":"<<s.total_ms
         <<",\"mean_ms\":"<<s.mean_ms
         <<",\"p50_ms\":"<<s.p50_ms
         <<",\"p95_ms\":"<<s.p95_ms
         <<",\"max_ms\":"<<s.max_ms<<"}";
    }
    out<<"\n},\n";
    if(enabled()){
      const auto critical=fusion_critical_ms();
      const auto budget=frames*scan_period_ms;
      out<<"\"fusion_compute_ms\":"<<critical<<","
         <<"\n\"effective_processing_fps\":"<<(critical>0?frames*1000./critical:0)<<","
         <<"\n\"realtime_factor\":"<<(critical>0?budget/critical:0)<<","
         <<"\n\"average_offline_budget_met\":"<<(critical<=budget?"true":"false")<<",";
      // A frame-level workload proxy is not equivalent to the real-time
      // scheduler latency of a streaming SLAM implementation. It allocates
      // one-time integration/training/graph costs uniformly across scans.
      std::vector<double> latency(frames,critical/frames);
      for(std::size_t i=0;i<frames;++i)latency[i]=0;
      for(const auto& [name,observations]:samples_){
        const bool measured=name=="scan_deskew"||
                            name=="heldout_scan_match"||
                            name=="fused_map_insert";
        for(const auto& sample:observations)
          if(measured&&sample.per_frame&&sample.frame<frames)
            latency[sample.frame]+=sample.ms;
      }
      const double fixed=(
        stats("sensor_integration").total_ms+
        stats("training_map_build").total_ms+
        stats("graph_optimize").total_ms+
        std::max(0.,stats("fused_map_build").total_ms-
                    stats("fused_map_insert").total_ms))/frames;
      for(auto& value:latency)value+=fixed;
      std::sort(latency.begin(),latency.end());
      const auto at=[&](double q) {
        const auto index=static_cast<std::size_t>(std::ceil(q*frames))-1;
        return latency[index];
      };
      const double total=std::accumulate(latency.begin(),latency.end(),0.0);
      out<<"\n\"frame_proxy_mean_ms\":"<<total/frames<<","
         <<"\n\"frame_proxy_p50_ms\":"<<at(.5)<<","
         <<"\n\"frame_proxy_p95_ms\":"<<at(.95)<<","
         <<"\n\"frame_proxy_max_ms\":"<<latency.back()<<",";
    }else {
      out<<"\"fusion_compute_ms\":null,"
         <<"\n\"effective_processing_fps\":null,"
         <<"\n\"realtime_factor\":null,"
         <<"\n\"average_offline_budget_met\":null,"
         <<"\n\"frame_proxy_mean_ms\":null,"
         <<"\n\"frame_proxy_p50_ms\":null,"
         <<"\n\"frame_proxy_p95_ms\":null,"
         <<"\n\"frame_proxy_max_ms\":null,";
    }
    out<<"\n\"interpretation\":\"synthetic single-threaded offline throughput estimate; not proof of real-time sensor scheduling\"\n}\n";
  }
 private:
  std::map<std::string,std::vector<Observation>> samples_;
};
} // namespace slam::fusion
