#include "envelope.hpp"
#include <iostream>
#include <nav2_mppi_controller/optimizer.hpp>
#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
using json=nlohmann::json;
// Bound the native samples before their normal acceleration propagation. This
// preserves native sampling/weighting and gives every critic the same commands.
class BoundedDiffDrive : public mppi::DiffDriveMotionModel {
public:
  void predict(mppi::models::State &state) override {
    state.cvx=xt::clip(state.cvx,control_constraints_.vx_min,control_constraints_.vx_max);
    state.cwz=xt::clip(state.cwz,-control_constraints_.wz,control_constraints_.wz);
    mppi::DiffDriveMotionModel::predict(state);
    // predict retains the measured initial speed and propagates braking in vx/wz.
    // It also overwrites command samples with those velocities. Restore only the
    // command domain: a zero forward authority means a zero command even while
    // measured forward motion takes time to decay in the prediction.
    state.cvx=xt::clip(state.cvx,control_constraints_.vx_min,control_constraints_.vx_max);
    state.cwz=xt::clip(state.cwz,-control_constraints_.wz,control_constraints_.wz);
  }
};
class NativeOptimizer : public mppi::Optimizer {
public:
  void configure_adapter() {
    settings_.shift_control_sequence=false;
    motion_model_=std::make_shared<BoundedDiffDrive>();
    motion_model_->initialize(settings_.constraints,settings_.model_dt);
  }
  void limits(double v) {
    settings_.constraints=settings_.base_constraints; settings_.constraints.vx_max=v;
    motion_model_->initialize(settings_.constraints,settings_.model_dt);
  }
  void warm_shift() { shiftControlSequence(); }
  geometry_msgs::msg::TwistStamped constrained_output(const builtin_interfaces::msg::Time &stamp) {
    // Native filtering uses prior history, which can cross a newly reduced bound.
    // Reapply Nav2's own sequence constraints before validating or issuing it.
    applyControlSequenceConstraints();
    return getControlFromSequenceAsTwist(stamp);
  }
  json controls() const {
    json result=json::array();
    for(size_t k=0;k<control_sequence_.vx.size();++k) result.push_back({control_sequence_.vx(k),control_sequence_.wz(k)});
    return result;
  }
};
geometry_msgs::msg::PoseStamped pose(double x,double y,double yaw,const builtin_interfaces::msg::Time &stamp) {
  geometry_msgs::msg::PoseStamped p; p.header.frame_id="odom"; p.header.stamp=stamp;
  p.pose.position.x=x; p.pose.position.y=y; p.pose.orientation.z=std::sin(yaw/2); p.pose.orientation.w=std::cos(yaw/2); return p;
}
int main(int argc,char **argv) {
  try {
    rclcpp::init(argc,argv); std::string line;
    if(!std::getline(std::cin,line)) throw std::runtime_error("Missing initialization record");
    auto config=json::parse(line); auto &e=guidepup::envelope(); e.configure(config);
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("guidepup_mppi");
    node->declare_parameter("controller_frequency",10.);
    const std::string ns="FollowPath.";
    for(auto key:{"model_dt","temperature","gamma","vx_std","wz_std"}) node->declare_parameter(ns+key,config.at(key).get<double>());
    for(auto key:{"time_steps","batch_size","iteration_count"}) node->declare_parameter(ns+key,config.at(key).get<int>());
    node->declare_parameter(ns+"vx_max",config.at("max_speed").get<double>());
    node->declare_parameter(ns+"vx_min",0.); node->declare_parameter(ns+"vy_max",0.);
    node->declare_parameter(ns+"wz_max",.6); node->declare_parameter(ns+"ax_max",.4);
    node->declare_parameter(ns+"ax_min",-.4); node->declare_parameter(ns+"az_max",.8);
    node->declare_parameter(ns+"motion_model",std::string("DiffDrive"));
    node->declare_parameter(ns+"retry_attempt_limit",0); node->declare_parameter(ns+"open_loop",false);
    node->declare_parameter(ns+"regenerate_noises",true);
    node->declare_parameter(ns+"critics",std::vector<std::string>{"GuidePupEnvelopeCritic","ConstraintCritic","GoalCritic","GoalAngleCritic","PathFollowCritic","PathAngleCritic"});
    node->declare_parameter(ns+"PathFollowCritic.offset_from_furthest",1);
    node->declare_parameter(ns+"PathFollowCritic.threshold_to_consider",.1);
    node->declare_parameter(ns+"GoalCritic.threshold_to_consider",1.0);
    node->declare_parameter(ns+"GoalAngleCritic.threshold_to_consider",.3);
    rclcpp::NodeOptions map_options;
    map_options.parameter_overrides({rclcpp::Parameter("plugins",std::vector<std::string>{}),
      rclcpp::Parameter("global_frame","odom"),rclcpp::Parameter("robot_base_frame","base_link"),
      rclcpp::Parameter("track_unknown_space",true),rclcpp::Parameter("resolution",.04),
      rclcpp::Parameter("width",20),rclcpp::Parameter("height",20)});
    auto map=std::make_shared<nav2_costmap_2d::Costmap2DROS>(map_options);
    if(map->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS) throw std::runtime_error("Costmap configure failed");
    mppi::ParametersHandler parameters(node); NativeOptimizer optimizer;
    optimizer.initialize(node,"FollowPath",map,&parameters); optimizer.configure_adapter();
    std::cout<<json({{"status","ready"},{"backend","nav2_mppi_controller::Optimizer"},{"nav2_version",NAV2_MPPI_VERSION},{"motion_model","BoundedDiffDrive/native_acceleration"},{"retry_attempt_limit",0},{"collision_scope","observed_depth_voxels"}}).dump()<<std::endl;
    std::string epoch; bool have_solution=false;
    while(std::getline(std::cin,line)) {
      auto request=json::parse(line);
      std::string next_epoch=request.at("epoch");
      if(epoch!=next_epoch) { optimizer.reset(); epoch=next_epoch; have_solution=false; }
      else if(have_solution) optimizer.warm_shift();
      e.initial=request.at("initial").get<guidepup::State>(); e.hard=request.at("hard_clearance_m");
      std::vector<guidepup::Point> points;
      auto grid=map->getCostmap();
      // Unknown remains unknown. The envelope queries exact occupied voxel centres;
      // this grid is only the native path critic's spatial context.
      double minx=e.initial[0]-10,miny=e.initial[1]-10,maxx=e.initial[0]+10,maxy=e.initial[1]+10;
      for(const auto &p:request.at("reference")) { minx=std::min(minx,p[0].get<double>()-1); miny=std::min(miny,p[1].get<double>()-1); maxx=std::max(maxx,p[0].get<double>()+1); maxy=std::max(maxy,p[1].get<double>()+1); }
      grid->resizeMap(std::ceil((maxx-minx)/.04),std::ceil((maxy-miny)/.04),.04,minx,miny);
      grid->resetMap(0,0,grid->getSizeInCellsX(),grid->getSizeInCellsY());
      for(const auto &p:request.at("obstacles")) {
        double x=p[0],y=p[1]; points.emplace_back(x,y); unsigned int mx,my;
        if(grid->worldToMap(x,y,mx,my)) grid->setCost(mx,my,nav2_costmap_2d::LETHAL_OBSTACLE);
      }
      e.obstacles=guidepup::Tree(points.begin(),points.end());
      e.begin_request(request);
      builtin_interfaces::msg::Time stamp=rclcpp::Time(static_cast<int64_t>(request.at("stamp_s").get<double>()*1e9));
      auto current=pose(e.initial[0],e.initial[1],e.initial[2],stamp);
      geometry_msgs::msg::Twist speed; speed.linear.x=e.initial[3]; speed.linear.y=e.initial[4]; speed.angular.z=e.initial[5];
      nav_msgs::msg::Path path; path.header=current.header;
      for(const auto &p:request.at("reference")) path.poses.push_back(pose(p[0],p[1],p[2],stamp));
      optimizer.limits(request.at("speed_limit"));
      try {
        optimizer.evalControl(current,speed,path,path.poses.back().pose,nullptr);
        auto command=optimizer.constrained_output(stamp);
        have_solution=true;
        auto controls=optimizer.controls();
        const auto evaluation=e.evaluate(controls.get<guidepup::Controls>(),true);
        auto final_check=evaluation.metrics();
        final_check["controls"]=controls; final_check["knots"]=evaluation.trace;
        json reply={{"status",evaluation.accepted?"success":"blocked"},
          {"category",evaluation.accepted?"accepted":evaluation.reason},
          {"reason",evaluation.reason},{"minimum_clearance_m",guidepup::finite(evaluation.min_geometry)},
          {"stop_minimum_clearance_m",guidepup::finite(evaluation.min_empirical_stop)},
          {"backend","nav2_mppi_controller::Optimizer"},{"optimization_attempts",1},
          {"diagnostics",e.diagnostics(final_check)}};
        if(evaluation.accepted) {
          reply["command"]={command.twist.linear.x,command.twist.angular.z};
          reply["controls"]=controls; reply["predicted"]=evaluation.predicted;
        } else {
          optimizer.reset(); have_solution=false;
        }
        std::cout<<reply.dump()<<std::endl;
      } catch(const nav2_core::NoValidControl &error) {
        have_solution=false;
        const std::string category=e.restoration && e.execution_stamp>=e.deadline ?
          "stop_restoration_expired":"no_valid_control";
        std::cout<<json({{"status","blocked"},{"category",category},{"reason",category},
          {"native_error",error.what()},{"backend","nav2_mppi_controller::Optimizer"},{"optimization_attempts",1},
          {"diagnostics",e.diagnostics({{"available",false},{"reason","native_no_valid_control_before_weighting_and_filter"},
            {"controls",nullptr},{"knots",nullptr}})}}).dump()<<std::endl;
      }
    }
    optimizer.shutdown(); map->on_cleanup(rclcpp_lifecycle::State()); rclcpp::shutdown(); return 0;
  } catch(const std::exception &error) {
    std::cerr<<"GuidePup native MPPI ERROR: "<<error.what()<<std::endl; return 1;
  }
}
