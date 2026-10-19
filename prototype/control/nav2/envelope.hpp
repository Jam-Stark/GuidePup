#pragma once
#include <array>
#include <vector>
#include <cmath>
#include <limits>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <nlohmann/json.hpp>
namespace guidepup {
using State = std::array<double, 6>;
using Controls = std::vector<std::array<double, 2>>;
using json = nlohmann::json;
inline json finite(double v) { return std::isfinite(v) ? json(v) : json(nullptr); }
struct Evaluation {
  bool accepted=true;
  std::string reason="accepted";
  double cost=0, min_geometry=INFINITY, min_nominal_stop=INFINITY, min_empirical_stop=INFINITY, terminal=INFINITY;
  json trace=json::array(), predicted=json::array();
  json metrics() const;
};
using Point = boost::geometry::model::point<double,2,boost::geometry::cs::cartesian>;
using Tree = boost::geometry::index::rtree<Point,boost::geometry::index::quadratic<16>>;
struct Envelope {
  std::vector<std::array<double,2>> centers;
  std::vector<double> radii, times, position_residual, yaw_residual;
  std::array<double,3> stop_tau;
  double velocity_tau, yaw_tau, cell_radius, hard, desired, weight;
  State initial;
  Tree obstacles;
  double model_dt=0, horizon=0, execution_stamp=0;
  size_t time_steps=0;
  bool restoration=false;
  std::string episode_epoch;
  double episode_start=0, deadline=0, deficit_ceiling=0;
  json planning_context, candidate_evaluations=json::array(), candidate_examples=json::object();
  double best_candidate_cost=INFINITY;
  void begin_request(const json &request);
  Evaluation evaluate(const Controls &controls, bool trace=false) const;
  json diagnostics(const json &final_check) const;
  double nominal_stop_clearance(const State &s) const {
    double result=clearance(s);
    for(int k=1;k<=40;++k) result=std::min(result,clearance(stopping(s,k*.05)));
    return result;
  }
  void configure(const nlohmann::json &j) {
    model_dt=j.at("model_dt"); time_steps=j.at("time_steps"); horizon=model_dt*time_steps;
    auto f=j.at("footprint"), s=j.at("stop_model");
    centers=f.at("centers_base_m").get<decltype(centers)>(); radii=f.at("radii_m").get<decltype(radii)>();
    velocity_tau=f.at("velocity_response_time_s"); yaw_tau=f.at("yaw_response_time_s");
    cell_radius=f.at("obstacle_cell_side_m").get<double>()/std::sqrt(2.);
    times=s.at("times_s").get<decltype(times)>(); stop_tau=s.at("velocity_response_time_s").get<decltype(stop_tau)>();
    position_residual=s.at("position_residual_m").get<decltype(position_residual)>();
    yaw_residual=s.at("yaw_residual_rad").get<decltype(yaw_residual)>();
    desired=j.at("preferred_clearance_m"); weight=j.at("clearance_cost_weight");
  }
  State advance(const State &s,double v,double w,double dt) const {
    double ev=std::exp(-dt/velocity_tau), ew=std::exp(-dt/yaw_tau);
    double f=dt*v+(s[3]-v)*velocity_tau*(1-ev), l=s[4]*velocity_tau*(1-ev);
    double heading=s[2]+.5*dt*w+(s[5]-w)*yaw_tau*(1-std::exp(-.5*dt/yaw_tau));
    return {s[0]+f*std::cos(heading)-l*std::sin(heading),s[1]+f*std::sin(heading)+l*std::cos(heading),
      s[2]+dt*w+(s[5]-w)*yaw_tau*(1-ew),v+(s[3]-v)*ev,s[4]*ev,w+(s[5]-w)*ew};
  }
  State stopping(const State &s,double t) const {
    auto o=s; double f=s[3]*stop_tau[0]*(1-std::exp(-t/stop_tau[0]));
    double l=s[4]*stop_tau[1]*(1-std::exp(-t/stop_tau[1]));
    double h=s[2]+s[5]*stop_tau[2]*(1-std::exp(-.5*t/stop_tau[2]));
    o[0]+=f*std::cos(h)-l*std::sin(h); o[1]+=f*std::sin(h)+l*std::cos(h);
    o[2]+=s[5]*stop_tau[2]*(1-std::exp(-t/stop_tau[2])); return o;
  }
  double clearance(const State &s,int stop_index=-1) const {
    double result=std::numeric_limits<double>::infinity();
    if(obstacles.empty()) return result;
    for(size_t k=0;k<centers.size();++k) {
      Point q(s[0]+std::cos(s[2])*centers[k][0]-std::sin(s[2])*centers[k][1],
              s[1]+std::sin(s[2])*centers[k][0]+std::cos(s[2])*centers[k][1]);
      auto nearest=obstacles.qbegin(boost::geometry::index::nearest(q,1));
      double residual=stop_index<0?0:position_residual[stop_index]+yaw_residual[stop_index]*std::hypot(centers[k][0],centers[k][1]);
      result=std::min(result,boost::geometry::distance(q,*nearest)-radii[k]-cell_radius-residual);
    } return result;
  }
  double stop_clearance(const State &s) const {
    double result=std::numeric_limits<double>::infinity();
    for(size_t k=0;k<times.size();++k) result=std::min(result,clearance(stopping(s,times[k]),k));
    return result;
  }
};
Envelope & envelope();
}
