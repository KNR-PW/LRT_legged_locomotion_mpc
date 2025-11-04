// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 * Based on rgrandia on 02.03.20 (https://github.com/leggedrobotics/ocs2)
 */


#ifndef __CONTACT_FORCE_WRENCH_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__
#define __CONTACT_FORCE_WRENCH_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/common/Utils.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {
    class ContactForceWrenchTrajectoryPlanner
    {
      public:

        /**
         * Constructor for a BaseTrajectoryPlanner.
         * @param [in] modelInfo: FloatingBase model info
         */
        ContactForceWrenchTrajectoryPlanner(
          floating_base_model::FloatingBaseModelInfo modelInfo);
        
        /** 
         * Update target trajectories with force and wrench trajectory
         * @param [in] contactFlagsTrajectory: trajectory of contact flags
         * @param [out] targetTrajectories: target trajectories
         * 
         * @remark Every element of contactFlagsTrajectory should be generated at every 
         * time point in targetTrajectories.timeTrajectory
         * (contactFlagsTrajectory[i]  at targetTrajectories.timeTrajectory[i]).
         * Generate contact trajectory with GaitPlanner::getContactFlagsAtTimes() 
         * using targetTrajectories.timeTrajectory.
         */
        void updateTargetTrajectory(std::vector<contact_flags_t> contactFlagsTrajectory,
          ocs2::TargetTrajectories& targetTrajectories);

      private:

        floating_base_model::FloatingBaseModelInfo modelInfo_;
    };
  }; // namespace planners
}; // namespace legged_locomotion_mpc

#endif