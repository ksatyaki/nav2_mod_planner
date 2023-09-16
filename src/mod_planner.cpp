//
// Created by ksatyaki on 15/9/23.
//

#include <nav2_mod_planner/mod_planner.hpp>
#include <nav2_util/node_utils.hpp>

namespace nav2_mod_planner {

void MoDPlanner::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Initialize all the parameters
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".cliffmap_filename", rclcpp::ParameterValue(""));

  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".gmmtmap_filename", rclcpp::ParameterValue(""));

  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".motion_plan_costs_topic",
      rclcpp::ParameterValue("plan_costs"));
}
} // namespace nav2_mod_planner