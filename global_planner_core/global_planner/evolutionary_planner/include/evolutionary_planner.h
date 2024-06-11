/**
 * *********************************************************
 *
 * @file: evolutionary_planner.h
 * @brief: Contains the evolutionary planner ROS wrapper class
 * @author: Yang Haodong
 * @date: 2023-7-16
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef EVOLUTIONARY_PLANNER_H
#define EVOLUTIONARY_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <nav_msgs/GetPlan.h>

#include "global_planner.h"

namespace evolutionary_planner
{
class EvolutionaryPlanner : public nav_core::BaseGlobalPlanner
{
public:
  /**
   * @brief Construct a new Evolutionary Planner object
   */
  EvolutionaryPlanner();

  /**
   * @brief Construct a new Evolutionary Planner object
   * @param name      planner name
   * @param costmap_ros   the cost map to use for assigning costs to trajectories
   */
  EvolutionaryPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Destroy the Evolutionary Planner object
   */
  ~EvolutionaryPlanner() = default;

  /**
   * @brief Planner initialization
   * @param name        planner name
   * @param costmapRos  costmap ROS wrapper
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmapRos);

  /**
   * @brief Planner initialization
   * @param name      planner name
   * @param costmap   costmap pointer
   * @param frame_id  costmap frame ID
   */
  void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

  /**
   * @brief Plan a path given start and goal in world map
   * @param start start in world map
   * @param goal  goal in world map
   * @param plan  plan
   * @return true if find a path successfully, else false
   */
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Plan a path given start and goal in world map
   * @param start     start in world map
   * @param goal      goal in world map
   * @param plan      plan
   * @param tolerance error tolerance
   * @return true if find a path successfully, else false
   */
  bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, double tolerance,
                std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Publish planning path
   * @param path  planning path
   */
  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief Regeister planning service
   * @param req   request from client
   * @param resp  response from server
   */
  bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

protected:
  /**
   * @brief Calculate plan from planning path
   * @param path  path generated by global planner
   * @param plan  plan transfromed from path, i.e. [start, ..., goal]
   * @return  bool true if successful, else false
   */
  bool _getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan);

  /**
   * @brief  publish expand zone
   * @param  expand  set of expand nodes
   */
  void _publishExpand(std::vector<Node>& expand);

protected:
  bool initialized_;                                          // initialization flag
  std::shared_ptr<global_planner::GlobalPlanner> g_planner_;  // global graph planner
  std::string frame_id_;                                      // costmap frame ID
  ros::Publisher plan_pub_;                                   // path planning publisher
  ros::ServiceServer make_plan_srv_;                          // planning service
  ros::Publisher expand_pub_;                                 // nodes explorer publisher

private:
  double tolerance_;  // tolerance
  double factor_;     // obstacle inflation factor
  bool is_outline_;   // whether outline the boudary of map
  bool is_expand_;    // whether publish expand map or not
};
}  // namespace evolutionary_planner
#endif