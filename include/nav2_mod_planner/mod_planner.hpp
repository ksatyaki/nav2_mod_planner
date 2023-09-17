//
// Created by ksatyaki on 15/9/23.
//
#pragma once

#include <memory>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav_msgs/msg/path.hpp>

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/mod/objectives/DTCOptimizationObjective.h>
#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>
#include <ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h>
#include <ompl/mod/samplers/HybridSampler.h>

namespace nav2_mod_planner {
class MoDPlanner : public nav2_core::GlobalPlanner {
public:
  MoDPlanner();

  ~MoDPlanner() = default;

  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

private:
  struct steering_params {
    double turning_radius;
    double max_vehicle_speed;
    std::vector<geometry_msgs::msg::Point> footprint;
  } steering_params_;

  struct planner_params {
    std::string sampler_type;
    std::string objective_type;
    std::string weight_euclidean, weight_quaternion, weight_mod;
    std::string cliffmap_filename, gmmtmap_filename;
    double max_planning_time;
    double path_resolution;
  } planner_params_;

  // Footprint collision checker
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>
      footprint_collision_checker_;

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D *costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  // Motion plan costs topic
  std::string motion_plan_costs_topic_;

  // An OMPL simple setup ptr
  ompl::geometric::SimpleSetupPtr simple_setup_;

  // Optimization Objective ptr
  std::shared_ptr<ompl::base::OptimizationObjective> optimization_objective_;

  std::shared_ptr<ompl::base::InformedSampler> sampler_;
};

} // namespace nav2_mod_planner
