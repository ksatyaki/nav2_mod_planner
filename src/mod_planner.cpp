//
// Created by ksatyaki on 15/9/23.
//

#include <nav2_mod_planner/CarStateSpace.hpp>
#include <nav2_mod_planner/mod_planner.hpp>
#include <nav2_util/node_utils.hpp>
#include <rclcpp/logger.hpp>

namespace nav2_mod_planner {

MoDPlanner::MoDPlanner() : footprint_collision_checker_(nullptr) {}

void MoDPlanner::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Initialize footprint collision checker
  footprint_collision_checker_.setCostmap(costmap_);

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

  // Get footprint
  std::string footprint_string;
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".steering_params.footprint",
      rclcpp::ParameterValue(
          std::string("[[0.5,-0.5],[0.5,0.5],[-0.5,0.5],[-0.5,-0.5]]")));
  node_->get_parameter(name_ + ".steering_params.footprint", footprint_string);
  if (nav2_costmap_2d::makeFootprintFromString(footprint_string,
                                               steering_params_.footprint)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse footprint parameter, "
                                      "using a square footprint of size 1x1");
    nav2_costmap_2d::makeFootprintFromString(
        "[[0.5,-0.5],[0.5,0.5],[-0.5,0.5],[-0.5,-0.5]]",
        steering_params_.footprint);
  } else {
    RCLCPP_INFO(node_->get_logger(), "Footprint loaded successfully");
  }

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

  // Create a car state space ptr
  ompl::base::StateSpacePtr car_state_space(
      new ompl::base::CarStateSpace(steering_params_.turning_radius));

  // Initialize ompl simple setup.
  simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(
      ompl::geometric::SimpleSetup(car_state_space));

  ompl::base::RealVectorBounds bounds(2);

  if (!no_map_) {
    bounds.low[0] = costmap_->getOriginX();
    bounds.high[0] = costmap_->getOriginX() + costmap_->getSizeInMetersX();
    bounds.low[1] = costmap_->getOriginY();
    bounds.high[1] = costmap_->getOriginY() + costmap_->getSizeInMetersY();

  } else {
    bounds.low[0] = -10000;
    bounds.low[1] = -10000;
    bounds.high[0] = 10000;
    bounds.high[1] = 10000;
  }
  car_state_space->as<ompl::base::SE2StateSpace>()->setBounds(bounds);

  // Set-up planner
  ompl::base::SpaceInformationPtr si(simple_setup_->getSpaceInformation());
  auto planner = std::make_shared<ompl::geometric::PRMstar>(si);

  // Set the optimization objective based on the objective type parameter
  // cliff-euc = UpstreamCriterionOptimizationObjective with cliffmap
  // gmmt-euc = UpstreamCriterionOptimizationObjective with gmmtmap
  // cliff-dtc = DownTheCLiFFOptimizationObjective with cliffmap
  // path-length = PathLengthOptimizationObjective
  std::shared_ptr<ompl::base::OptimizationObjective> opt_obj_;
  if (planner_params_.objective_type == "cliff-euc") {
    opt_obj_ =
        std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(
            si, planner_params_.cliffmap_filename,
            planner_params_.weight_euclidean);
    simple_setup_->setOptimizationObjective(opt_obj_);
  } else if (planner_params_.objective_type == "gmmt-euc") {
    opt_obj_ =
        std::make_shared<ompl::MoD::UpstreamCriterionOptimizationObjective>(
            si, planner_params_.gmmtmap_filename,
            planner_params_.weight_euclidean);
    simple_setup_->setOptimizationObjective(opt_obj_);
  } else if (planner_params_.objective_type == "cliff-dtc") {
    opt_obj_ = std::make_shared<ompl::MoD::DTCOptimizationObjective>(
        si, planner_params_.cliffmap_filename,
        planner_params_.weight_euclidean);
    simple_setup_->setOptimizationObjective(opt_obj_);
  } else if (planner_params_.objective_type == "path-length") {
    opt_obj_ =
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
    simple_setup_->setOptimizationObjective(opt_obj_);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Invalid objective type");
  }

  simple_setup_->setOptimizationObjective(opt_obj_);
}
} // namespace nav2_mod_planner