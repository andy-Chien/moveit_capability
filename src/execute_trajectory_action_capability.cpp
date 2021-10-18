/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Kentaro Wada */

#include "moveit_capability/execute_trajectory_action_capability.h"

#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{

moveit_controller_manager::ExecutionStatus MoveGroupExecuteTrajectoryActionT::execute_status_ = moveit_controller_manager::ExecutionStatus::UNKNOWN;

MoveGroupExecuteTrajectoryActionT::MoveGroupExecuteTrajectoryActionT() : MoveGroupCapability("ExecuteTrajectoryAction")
{
}


void MoveGroupExecuteTrajectoryActionT::initialize()
{
  // start the move action server
  execute_action_server_.reset(new actionlib::SimpleActionServer<moveit_msgs::ExecuteTrajectoryAction>(
      root_node_handle_, EXECUTE_ACTION_NAME,
      boost::bind(&MoveGroupExecuteTrajectoryActionT::executePathCallback, this, _1), false));
  execute_action_server_->registerPreemptCallback(
      boost::bind(&MoveGroupExecuteTrajectoryActionT::preemptExecuteTrajectoryCallback, this));
  execute_action_server_->start();
}

void MoveGroupExecuteTrajectoryActionT::executeCallback(const moveit_controller_manager::ExecutionStatus& status)
{ 
  MoveGroupExecuteTrajectoryActionT::execute_status_ = status; 
}

void MoveGroupExecuteTrajectoryActionT::executePathCallback(const moveit_msgs::ExecuteTrajectoryGoalConstPtr& goal)
{
  moveit_msgs::ExecuteTrajectoryResult action_res;
  if (!context_->trajectory_execution_manager_)
  {
    const std::string response = "Cannot execute trajectory since ~allow_trajectory_execution was set to false";
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    execute_action_server_->setAborted(action_res, response);
    return;
  }

  executePath(goal, action_res);

  const std::string response = getActionResultString(action_res.error_code, false, false);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    execute_action_server_->setSucceeded(action_res, response);
  }
  else if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::PREEMPTED)
  {
    execute_action_server_->setPreempted(action_res, response);
  }
  else
  {
    execute_action_server_->setAborted(action_res, response);
  }

  setExecuteTrajectoryState(IDLE);
}

void MoveGroupExecuteTrajectoryActionT::executePath(const moveit_msgs::ExecuteTrajectoryGoalConstPtr& goal,
                                                   moveit_msgs::ExecuteTrajectoryResult& action_res)
{
  ROS_INFO("executePath, executePath, executePath, executePath, executePath");
  ROS_INFO_NAMED(getName(), "Execution request received");

  context_->trajectory_execution_manager_->clear();
  if (context_->trajectory_execution_manager_->push(goal->trajectory))
  {
    setExecuteTrajectoryState(MONITOR);
    MoveGroupExecuteTrajectoryActionT::execute_status_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    trajectory_execution_manager::TrajectoryExecutionManager::ExecutionCompleteCallback execution_callback;
    execution_callback = MoveGroupExecuteTrajectoryActionT::executeCallback;
    context_->trajectory_execution_manager_->execute(execution_callback);
    planning_scene::PlanningSceneConstPtr planning_scene = context_->planning_scene_monitor_->getPlanningScene();
    collision_detection::CollisionRequest req;
    robot_trajectory::RobotTrajectory t(planning_scene->getRobotModel(), "");
    robot_state::RobotState start(planning_scene->getCurrentState());
    // robot_state::robotStateMsgToRobotState(planning_scene->getTransforms(), start_state, start);
    t.setRobotTrajectoryMsg(start, goal->trajectory);
    ros::Rate r(40);
    std::size_t wpc = t.getWayPointCount();
    std::pair<int, int> path_segment;
    try
    {
      while(node_handle_.ok() && execute_status_ == moveit_controller_manager::ExecutionStatus::RUNNING)
      {
        path_segment = context_->trajectory_execution_manager_->getCurrentExpectedTrajectoryIndex();
        for (std::size_t i = std::max(path_segment.second - 1, 0); i < std::min(path_segment.second + 10, int(wpc)); ++i)
        {
          planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
          if (path_segment.second == -1)
            break;
          collision_detection::CollisionResult res;
          // planning_scene->checkCollisionUnpadded(req, res, t.getWayPoint(i));
          planning_scene->checkCollision(req, res, t.getWayPoint(i));
          if (res.collision)
          {
            ROS_INFO("!!!!!!Collision detected during execution!!!!!!");
            context_->trajectory_execution_manager_->stopExecution();
            execute_status_ = moveit_controller_manager::ExecutionStatus::FAILED;
          }
        }
        r.sleep();
      }
    }
    catch (...)
    {
      ROS_ERROR("!!!!!!!Collision detected during execution Failed!!!!!!!");
    }
    moveit_controller_manager::ExecutionStatus status = context_->trajectory_execution_manager_->waitForExecution();
    if (execute_status_ == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
    }
    else if (execute_status_ == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    }
    else if (execute_status_ == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
    }
    else
    {
      action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    }
    ROS_INFO_STREAM_NAMED(getName(), "Execution completed: " << execute_status_.asString());
  }
  else
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
  }
}

void MoveGroupExecuteTrajectoryActionT::preemptExecuteTrajectoryCallback()
{
  context_->trajectory_execution_manager_->stopExecution(true);
}

void MoveGroupExecuteTrajectoryActionT::setExecuteTrajectoryState(MoveGroupState state)
{
  moveit_msgs::ExecuteTrajectoryFeedback execute_feedback;
  execute_feedback.state = stateToStr(state);
  execute_action_server_->publishFeedback(execute_feedback);
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupExecuteTrajectoryActionT, move_group::MoveGroupCapability)
