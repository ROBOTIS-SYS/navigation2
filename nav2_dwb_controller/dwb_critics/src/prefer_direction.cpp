/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "dwb_critics/prefer_direction.hpp"
#include <math.h>
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/node_utils.hpp"

PLUGINLIB_EXPORT_CLASS(dwb_critics::PreferDirectionCritic, dwb_core::TrajectoryCritic)

using nav2_util::declare_parameter_if_not_declared;

namespace dwb_critics
{

void PreferDirectionCritic::onInit()
{
  declare_parameter_if_not_declared(
    nh_,
    dwb_plugin_name_ + "." + name_ + ".penalty", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    nh_,
    dwb_plugin_name_ + "." + name_ + ".prefer_right", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    nh_, dwb_plugin_name_ + "." + name_ + ".scale",
    rclcpp::ParameterValue(1.0));

  nh_->get_parameter(dwb_plugin_name_ + "." + name_ + ".penalty", penalty_);
  nh_->get_parameter(dwb_plugin_name_ + "." + name_ + ".prefer_right", prefer_right_);
  nh_->get_parameter(dwb_plugin_name_ + "." + name_ + ".scale", scale_);
}

double PreferDirectionCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  std::cout << "Tra vel | x : " << traj.velocity.x << ", y : " << traj.velocity.y <<
    ", theta : " << traj.velocity.theta << std::endl;

  if (prefer_right_ && traj.velocity.theta > 0.0) {
//    std::cout << "[PreferDirection]|Right| theta : " << traj.velocity.theta << ", penalty : " <<
//      penalty_ << std::endl;
    return penalty_;
  }

  if (!prefer_right_ && traj.velocity.theta < 0.0) {
//    std::cout << "[PreferDirection]|Left| theta : " << traj.velocity.theta << ", penalty : " <<
//      penalty_ << std::endl;
    return penalty_;
  }

  return 0.0;
}

}  // namespace dwb_critics
