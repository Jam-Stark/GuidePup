#include "envelope.hpp"
#include <nav2_mppi_controller/critic_function.hpp>
#include <pluginlib/class_list_macros.hpp>
namespace guidepup {
Envelope & envelope() { static Envelope value; return value; }
json Evaluation::metrics() const {
  return {{"accepted",accepted},{"reason",reason},{"cost",cost},
    {"minimum_geometry_clearance_m",finite(min_geometry)},
    {"minimum_nominal_stop_clearance_m",finite(min_nominal_stop)},
    {"minimum_empirical_stop_clearance_m",finite(min_empirical_stop)},
    {"terminal_empirical_stop_clearance_m",finite(terminal)}};
}
void Envelope::begin_request(const json &request) {
  execution_stamp=request.at("stamp_s").get<double>()+request.at("execution_delay_s").get<double>();
  const std::string epoch=request.at("epoch");
  const double initial_h=stop_clearance(initial);
  if(epoch!=episode_epoch || initial_h>=hard) restoration=false;
  episode_epoch=epoch;
  // This comparison uses the NAV command response, not the distinct calibrated
  // stopping response. It is never selected as a fallback command sequence.
  auto zero_state=initial;
  double baseline_min=initial_h;
  for(size_t t=0;t<time_steps;++t) {
    zero_state=advance(zero_state,0,0,model_dt);
    baseline_min=std::min(baseline_min,stop_clearance(zero_state));
  }
  const double baseline_deficit=std::max(0.,hard-baseline_min);
  if(initial_h<hard) {
    if(!restoration) {
      restoration=true; episode_start=execution_stamp;
      deadline=episode_start+horizon; deficit_ceiling=baseline_deficit;
    } else deficit_ceiling=std::min(deficit_ceiling,baseline_deficit);
  }
  planning_context={{"mode",restoration?"restoration":"normal"},{"epoch",epoch},
    {"execution_stamp_s",execution_stamp},{"horizon_s",horizon},{"hard_clearance_m",hard},
    {"initial_geometry_clearance_m",finite(clearance(initial))},
    {"initial_nominal_stop_clearance_m",finite(nominal_stop_clearance(initial))},
    {"initial_empirical_stop_clearance_m",finite(initial_h)},
    {"zero_command_baseline_deficit_m",baseline_deficit},
    {"deficit_ceiling_m",restoration?json(deficit_ceiling):json(nullptr)},
    {"episode_start_execution_stamp_s",restoration?json(episode_start):json(nullptr)},
    {"restoration_deadline_s",restoration?json(deadline):json(nullptr)},
    {"restoration_remaining_s",restoration?json(deadline-execution_stamp):json(nullptr)},
    {"restoration_expired",restoration && execution_stamp>=deadline}};
  candidate_evaluations=json::array(); candidate_examples=json::object(); best_candidate_cost=INFINITY;
}
Evaluation Envelope::evaluate(const Controls &controls,bool include_trace) const {
  Evaluation result;
  auto reject=[&](const std::string &reason) {
    if(result.accepted) { result.accepted=false; result.reason=reason; }
  };
  auto geometry=[&](double g) {
    result.min_geometry=std::min(result.min_geometry,g);
    if(g<0) reject("predicted_geometry_overlap");
    else if(g<hard) reject("geometry_margin");
  };
  auto knot=[&](const State &s,size_t index) {
    const double g=clearance(s), nominal=nominal_stop_clearance(s), h=stop_clearance(s);
    geometry(g);
    result.min_nominal_stop=std::min(result.min_nominal_stop,nominal);
    result.min_empirical_stop=std::min(result.min_empirical_stop,h);
    result.terminal=h;
    if(nominal<hard) reject("nominal_stop_geometry");
    const double deficit=std::max(0.,hard-h);
    result.cost+=weight*deficit*deficit/(controls.size()+1);
    const double stamp=execution_stamp+index*model_dt;
    if(restoration) {
      if(execution_stamp>=deadline && h<hard) reject("stop_restoration_expired");
      if(deficit>deficit_ceiling) reject("stop_envelope_nonworsening");
      if((stamp>=deadline || index==controls.size()) && h<hard) reject("stop_restoration_deadline");
    } else if(h<hard) reject("stop_envelope");
    if(include_trace) {
      result.trace.push_back({{"knot",index},{"elapsed_s",index*model_dt},{"execution_stamp_s",stamp},
        {"state",s},{"geometry_clearance_m",finite(g)},{"nominal_stop_clearance_m",finite(nominal)},
        {"empirical_stop_clearance_m",finite(h)},{"empirical_deficit_m",deficit}});
      result.predicted.push_back({s[0],s[1],s[2]});
    }
  };
  auto s=initial; knot(s,0);
  for(size_t t=0;t<controls.size();++t) {
    const auto &u=controls[t];
    for(int j=1;j<=2;++j) {
      const double g=clearance(advance(s,u[0],u[1],model_dt*j/2.));
      geometry(g);
      result.cost+=weight*std::pow(std::max(0.,desired-g),2)/controls.size()/2.;
    }
    s=advance(s,u[0],u[1],model_dt); knot(s,t+1);
  }
  return result;
}
json Envelope::diagnostics(const json &final_check) const {
  json reasons=json::object(); size_t accepted=0;
  for(const auto &item:candidate_evaluations) {
    const std::string reason=item.at("reason");
    reasons[reason]=reasons.value(reason,0)+1;
    if(item.at("accepted").get<bool>()) ++accepted;
  }
  return {{"planning_context",planning_context},{"candidate_evaluations",candidate_evaluations},
    {"candidate_summary",{{"count",candidate_evaluations.size()},{"accepted",accepted},
      {"rejected",candidate_evaluations.size()-accepted},{"counts_by_reason",reasons},
      {"examples",candidate_examples},{"cost_scope","envelope critic only; native path/goal and control costs remain in Nav2"}}},
    {"final_check",final_check}};
}
}
namespace mppi::critics {
class GuidePupEnvelopeCritic : public CriticFunction {
public:
  void initialize() override {}
  void score(CriticData &data) override {
    auto &e=guidepup::envelope(); bool any_valid=false;
    for(size_t b=0;b<data.state.cvx.shape()[0];++b) {
      guidepup::Controls controls;
      for(size_t t=0;t<data.state.cvx.shape()[1];++t) controls.push_back({data.state.cvx(b,t),data.state.cwz(b,t)});
      const auto result=e.evaluate(controls);
      auto metrics=result.metrics(); metrics["index"]=b; metrics["first_command"]=controls.front();
      e.candidate_evaluations.push_back(metrics);
      if(result.accepted && result.cost<e.best_candidate_cost) {
        e.best_candidate_cost=result.cost;
        e.candidate_examples["best_feasible_envelope_cost"]={{"index",b},{"controls",controls}};
      } else if(!result.accepted && !e.candidate_examples.contains(result.reason)) {
        e.candidate_examples[result.reason]={{"index",b},{"controls",controls}};
      }
      data.costs(b)+=result.accepted?result.cost:1.0e8;
      any_valid=any_valid||result.accepted;
    }
    data.fail_flag=!any_valid;
  }
};
}
PLUGINLIB_EXPORT_CLASS(mppi::critics::GuidePupEnvelopeCritic,mppi::critics::CriticFunction)
