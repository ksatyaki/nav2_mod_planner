//
// Created by ksatyaki on 15/9/23.
//

#include <nav2_mod_planner/CarStateSpace.hpp>
#include <nav2_mod_planner/mod_planner.hpp>
#include <nav2_util/node_utils.hpp>
#include <rclcpp/logger.hpp>

namespace nav2_mod_planner {

MoDPlanner::MoDPlanner() {}

void MoDPlanner::getParameters() {
  // Initialize planner params using
  // nav2_util::declare_parameter_if_not_declared

  // Get turning radius
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".steering_params.turning_radius",
      rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".steering_params.turning_radius",
                       steering_params_.turning_radius);

  // Get max vehicle speed
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".steering_params.max_vehicle_speed",
      rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".steering_params.max_vehicle_speed",
                       steering_params_.max_vehicle_speed);

  // Get sampler type
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.sampler_type",
      rclcpp::ParameterValue(std::string("hybrid")));
  node_->get_parameter(name_ + ".planner_params.sampler_type",
                       planner_params_.sampler_type);

  // Get objective type
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.objective_type",
      rclcpp::ParameterValue(std::string("MoD-unaware")));
  node_->get_parameter(name_ + ".planner_params.objective_type",
                       planner_params_.objective_type);

  // Get weight euclidean
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.weight_euclidean",
      rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".planner_params.weight_euclidean",
                       planner_params_.weight_euclidean);

  // Get weight quaternion
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.weight_quaternion",
      rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".planner_params.weight_quaternion",
                       planner_params_.weight_quaternion);

  // Get weight mod
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.weight_mod", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".planner_params.weight_mod",
                       planner_params_.weight_mod);

  // Get cliffmap filename
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.cliffmap_filename",
      rclcpp::ParameterValue(std::string("")));
  node_->get_parameter(name_ + ".planner_params.cliffmap_filename",
                       planner_params_.cliffmap_filename);

  // Get gmmtmap filename
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.gmmtmap_filename",
      rclcpp::ParameterValue(std::string("")));
  node_->get_parameter(name_ + ".planner_params.gmmtmap_filename",
                       planner_params_.gmmtmap_filename);

  // Get gmmtmap filename
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.intensitymap_filename",
      rclcpp::ParameterValue(std::string("")));
  node_->get_parameter(name_ + ".planner_params.intensitymap_filename",
                       planner_params_.intensitymap_filename);

  // Get max planning time
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.max_planning_time",
      rclcpp::ParameterValue(30.0));
  node_->get_parameter(name_ + ".planner_params.max_planning_time",
                       planner_params_.max_planning_time);

  // Get path resolution
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.path_resolution",
      rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".planner_params.path_resolution",
                       planner_params_.path_resolution);

  // Get motion plan costs topic
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".motion_plan_costs_topic",
      rclcpp::ParameterValue(std::string("motion_plan_costs")));
  node_->get_parameter(name_ + ".motion_plan_costs_topic",
                       motion_plan_costs_topic_);

  // Get planner type
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".planner_params.planner_type",
      rclcpp::ParameterValue(std::string("rrtstar")));
  node_->get_parameter(name_ + ".planner_params.planner_type",
                       planner_params_.planner_type);
}

void MoDPlanner::initSimpleSetup() {
  // Create a car state space ptr
  ompl::base::StateSpacePtr car_state_space(
      new ompl::base::CarStateSpace(steering_params_.turning_radius));

  // Initialize ompl simple setup.
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(
      ompl::geometric::SimpleSetup(car_state_space));

  ompl::base::RealVectorBounds bounds(2);

  bounds.low[0] = costmap_->getOriginX();
  bounds.high[0] = costmap_->getOriginX() + costmap_->getSizeInMetersX();
  bounds.low[1] = costmap_->getOriginY();
  bounds.high[1] = costmap_->getOriginY() + costmap_->getSizeInMetersY();

  car_state_space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

  // Set-up planner
  ompl::base::SpaceInformationPtr si(simple_setup_->getSpaceInformation());
}

void MoDPlanner::initOptimizationObjectiveAndSampler() {
  // Set the optimization objective based on the objective type parameter
  // cliff-euc = UpstreamCriterionOptimizationObjective with cliffmap
  // gmmt-euc = UpstreamCriterionOptimizationObjective with gmmtmap
  // cliff-dtc = DownTheCLiFFOptimizationObjective with cliffmap
  // path-length = PathLengthOptimizationObjective
  if (planner_params_.objective_type == "cliff-euc") {
    optimization_objective_ =
        std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(
            simple_setup_->getSpaceInformation(), ompl::MoD::MapType::CLiFFMap,
            planner_params_.cliffmap_filename, planner_params_.weight_euclidean,
            planner_params_.weight_quaternion, planner_params_.weight_mod,
            planner_params_.sampler_type, planner_params_.intensitymap_filename,
            planner_params_.sampling_bias, true, false);
  } else if (planner_params_.objective_type == "gmmt-euc") {
    optimization_objective_ =
        std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(
            simple_setup_->getSpaceInformation(), ompl::MoD::MapType::GMMTMap,
            planner_params_.gmmtmap_filename, planner_params_.weight_euclidean,
            planner_params_.weight_quaternion, planner_params_.weight_mod,
            planner_params_.sampler_type, planner_params_.intensitymap_filename,
            planner_params_.sampling_bias, true, false);
  } else if (planner_params_.objective_type == "cliff-dtc") {
    optimization_objective_ =
        std::make_shared<ompl::MoD::DTCOptimizationObjective>(
            simple_setup_->getSpaceInformation(),
            planner_params_.cliffmap_filename,
            planner_params_.intensitymap_filename,
            planner_params_.weight_euclidean, planner_params_.weight_quaternion,
            planner_params_.weight_mod, steering_params_.max_vehicle_speed, 10,
            true, planner_params_.sampler_type, planner_params_.sampling_bias,
            true, false);
  } else if (planner_params_.objective_type == "path-length") {
    optimization_objective_ =
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(
            simple_setup_->getSpaceInformation());
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid objective type");
  }
}

void MoDPlanner::initPlanner() {
  if (planner_params_.planner_type == "rrtstar")
    planner_ = std::make_shared<ompl::geometric::RRTstar>(
        simple_setup_->getSpaceInformation());
  else if (planner_params_.planner_type == "aitstar")
    planner_ = std::make_shared<ompl::geometric::AITstar>(
        simple_setup_->getSpaceInformation());
  else if (planner_params_.planner_type == "prmstar")
    planner_ = std::make_shared<ompl::geometric::PRMstar>(
        simple_setup_->getSpaceInformation());
  else {
    RCLCPP_WARN(node_->get_logger(),
                "Invalid planner type. Choosing PRMstar as default.");
    planner_ = std::make_shared<ompl::geometric::PRMstar>(
        simple_setup_->getSpaceInformation());
  }
}

void MoDPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  getParameters();

  initSimpleSetup();

  initOptimizationObjectiveAndSampler();
  simple_setup_->setOptimizationObjective(optimization_objective_);

  initPlanner();
  simple_setup_->setPlanner(planner_);

  std::shared_ptr<
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
      footprint_collision_checker_ptr =
          std::make_shared<nav2_costmap_2d::FootprintCollisionChecker<
              nav2_costmap_2d::Costmap2D *>>(costmap_);
  state_validity_checker_ = std::make_shared<FootprintStateValidityChecker>(
      simple_setup_->getSpaceInformation(), footprint_collision_checker_ptr,
      costmap_ros->getRobotFootprint());
  simple_setup_->setStateValidityChecker(state_validity_checker_);

  RCLCPP_INFO(node_->get_logger(), "[MoDPlanner]: Configured MoD planner!");
}

void MoDPlanner::activate() {
  RCLCPP_INFO(node_->get_logger(), "[MoDPlanner]: Activating MoD planner!");
}

void MoDPlanner::deactivate() {
  simple_setup_->clear();
  RCLCPP_INFO(node_->get_logger(), "[MoDPlanner]: Deactivating MoD planner!");
}

void MoDPlanner::cleanup() {
  planner_.reset();
  state_validity_checker_.reset();
  optimization_objective_.reset();
  simple_setup_.reset();

  RCLCPP_INFO(node_->get_logger(), "[MoDPlanner]: Cleaning up MoD planner!");
}

nav_msgs::msg::Path MoDPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped &start,
    const geometry_msgs::msg::PoseStamped &goal) {
  nav_msgs::msg::Path solution_path;

  RCLCPP_INFO(node_->get_logger(),
              "[MoDPlanner]: createPlan will now attempt to find a solution "
              "for %lf seconds.",
              planner_params_.max_planning_time);

  // Allocate new start and goal states
  ompl::base::ScopedState<> start_state(simple_setup_->getStateSpace());
  ompl::base::ScopedState<> goal_state(simple_setup_->getStateSpace());

  // Set start and goal states from the ROS2 message types.
  start_state->as<ompl::base::SE2StateSpace::StateType>()->setXY(
      start.pose.position.x, start.pose.position.y);
  start_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(
      tf2::getYaw(start.pose.orientation));
  goal_state->as<ompl::base::SE2StateSpace::StateType>()->setXY(
      goal.pose.position.x, goal.pose.position.y);
  goal_state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(
      tf2::getYaw(goal.pose.orientation));

  // Set the start and goal states in simple setup
  simple_setup_->setStartAndGoalStates(start_state, goal_state);

  // Solve the problem.
  ompl::base::PlannerStatus status =
      simple_setup_->solve(planner_params_.max_planning_time);

  if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
    RCLCPP_INFO(node_->get_logger(), "[MoDPlanner]: Found a solution!");
    // Get the solution path and return it as nav_msgs::Path
    ompl::geometric::PathGeometric solution_path_ompl =
        simple_setup_->getSolutionPath();
    solution_path.header.frame_id = global_frame_;
    solution_path.header.stamp = node_->now();
    solution_path.poses.resize(solution_path_ompl.getStateCount());

    for (std::size_t i = 0; i < solution_path_ompl.getStateCount(); ++i) {
      const auto *state = solution_path_ompl.getState(i)
                              ->as<ompl::base::SE2StateSpace::StateType>();
      solution_path.poses[i].header = solution_path.header;
      solution_path.poses[i].pose.position.x = state->getX();
      solution_path.poses[i].pose.position.y = state->getY();
      solution_path.poses[i].pose.position.z = 0.0;
      solution_path.poses[i].pose.orientation =
          tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), state->getYaw()));
    }
  } else if (status == ompl::base::PlannerStatus::TIMEOUT) {
    RCLCPP_ERROR(node_->get_logger(), "[MoDPlanner 1]: Timed out.");
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "[MoDPlanner]: Status not exact solution from OMPL!");
  }

  return solution_path;
}
}  // namespace nav2_mod_planner

namespace ompl {
namespace base {
CarStateSpace::CarStateSpace(double turning_radius, bool is_symmetric)
    : DubinsStateSpace(turning_radius, is_symmetric) {}

unsigned int CarStateSpace::validSegmentCount(const State *state1,
                                              const State *state2) const {
  return longestValidSegmentCountFactor_ *
         (unsigned int)ceil(distance(state1, state2) / longestValidSegment_);
}
}  // namespace base
}  // namespace ompl

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_mod_planner::MoDPlanner, nav2_core::GlobalPlanner)