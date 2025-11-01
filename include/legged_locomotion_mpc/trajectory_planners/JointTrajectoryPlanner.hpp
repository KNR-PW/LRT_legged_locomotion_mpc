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
 */


#ifndef __JOINT_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__
#define __JOINT_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/SwingTrajectoryPlanner.hpp>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <ocs2_core/reference/TargetTrajectories.h>

#include <multi_end_effector_kinematics/MultiEndEffectorKinematics.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {
    class JointTrajectoryPlanner
    {
      public:

        JointTrajectoryPlanner(floating_base_model::FloatingBaseModelInfo modelInfo,
          multi_end_effector_kinematics::MultiEndEffectorKinematics&& kinematicsSolver);
        
        using position_trajectories = std::vector<std::vector<vector3_t>>;
        using velocity_trajectories = std::vector<std::vector<vector3_t>>;
        void updateTrajectory(const state_vector_t& currentState,
          ocs2::TargetTrajectories& targetTrajectories, 
          const position_trajectories& endEffectorPositionTrajectories,
          const velocity_trajectories& endEffectorVelocityTrajectories);

        ocs2::vector_t computeJointPositions(const ocs2::vector_t& actualJointPositions, 
          const vector6_t& basePose, 
          const std::vector<vector3_t>& endEffectorPositions);

        ocs2::vector_t computeJointVelocities(const ocs2::vector_t& actualJointPositions,
          const vector6_t& basePose, const vector6_t& baseVelocity,
          const std::vector<vector3_t>& endEffectorVelocities);
    
      private:
  
        floating_base_model::FloatingBaseModelInfo modelInfo_;

        multi_end_effector_kinematics::MultiEndEffectorKinematics kinematicsSolver_;
    };
    
    multi_end_effector_kinematics::KinematicsModelSettings loadKinematicsModelSettings(
      const std::string &filename, bool verbose = true);

    multi_end_effector_kinematics::InverseSolverSettings loadInverseSolverSettings(
      const std::string &filename, bool verbose = true);

    std::string loadInverseSolverName(const std::string &filename, bool verbose = true);
  };
}; // namespace legged_locomotion_mpc

#endif