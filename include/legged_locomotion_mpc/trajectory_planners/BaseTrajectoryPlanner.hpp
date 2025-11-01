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


#ifndef __BASE_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__
#define __BASE_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>

#include <terrain_model/core/TerrainModel.hpp>
#include <terrain_model/core/TerrainPlane.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {
    class BaseTrajectoryPlanner
    {
      public:

        struct StaticSettings
        {
          // Time between trajectory points
          ocs2::scalar_t deltaTime = 0.1;

          // Initial base height
          ocs2::scalar_t initialBaseHeight;

          // Minimum base height
          ocs2::scalar_t minimumBaseHeight;

          // Maximumbase height
          ocs2::scalar_t maximumBaseHeight;

          // Nominal lateral base width
          ocs2::scalar_t nominalBaseWidtLateral;

          // Nominal heading base width
          ocs2::scalar_t nominalBaseWidthHeading;
        };

        struct BaseReferenceCommand
        {
          ocs2::scalar_t baseHeadingVelocity;
          ocs2::scalar_t baseLateralVelocity;
          ocs2::scalar_t baseVerticalVelocity;
          ocs2::scalar_t yawRate;
        };

        /**
         * Constructor for a BaseTrajectoryPlanner.
         * @param [in] modelInfo: FloatingBase model info
         * @param [in] settings: Planner settings
         */
        BaseTrajectoryPlanner(floating_base_model::FloatingBaseModelInfo modelInfo,
          StaticSettings settings);
        
        /** 
         * Update terrain model 
         * @param [in] baseHeight: New base height
         */
        void updateBaseHeight(ocs2::scalar_t baseHeight);

        /** 
         * Update terrain model 
         * @param [in] terrainModel: New terrain model
         */
        void updateTerrain(const terrain_model::TerrainModel& terrainModel);
        
        /** 
         * Update target trajectories with new base trajectory:
         * - time points from initTime to finalTime
         * - 3D base position in world frame
         * - ZYX euler base orientation in world frame
         * - 3D linear base velocity in base frame
         * - 3D angular base velocity in base frame
         * @param [in] initTime: initial time
         * @param [in] finalTime: final time
         * @param [in] command: base command
         * @param [in] initialState: current robot state (taken from sensors)
         * @param [out] targetTrajectories: target trajectories
         */
        void updateTargetTrajectory(ocs2::scalar_t initTime, 
          ocs2::scalar_t finalTime, const BaseReferenceCommand& command, 
          const state_vector_t& initialState, ocs2::TargetTrajectories& targetTrajectories);

      private:

        terrain_model::TerrainPlane projectedHeadingPlane(vector2_t baseXYPositionInWorld, 
          ocs2::scalar_t yawOnPlane);
        
        ocs2::scalar_t currentBaseHeight_;
        floating_base_model::FloatingBaseModelInfo modelInfo_;
        StaticSettings settings_;
        const terrain_model::TerrainModel* terrainModel_;
    };
  }; // namespace planners
}; // namespace legged_locomotion_mpc

#endif