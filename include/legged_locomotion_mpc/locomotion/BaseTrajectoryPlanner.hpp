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
 * Based on rgrandia on 02.03.20.
 */


#ifndef __BASE_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__
#define __BASE_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>

#include <terrain_model/core/ConvexTerrain.hpp>
#include <terrain_model/core/TerrainModel.hpp>
#include <terrain_model/core/TerrainPlane.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    class BaseTrajectoryPlanner
    {
      public:

        struct StaticSettings
        {
          // Time between trajectory points
          ocs2::scalar_t deltaTime = 0.1;

          // Minimum base height
          ocs2::scalar_t minimumBaseHeight = 0.4;

          // Maximumbase height
          ocs2::scalar_t maximumBaseHeight = 0.6;
        };

        struct BaseReferenceCommand
        {
          /**
           * IMPORTANT: baseVerticalVelocity is only taken into consideration if and 
           * only if other velocities are 0!
           *  
           */ 
          ocs2::scalar_t baseHeadingVelocity;
          ocs2::scalar_t baseLateralVelocity;
          ocs2::scalar_t baseVerticalVelocity;
          ocs2::scalar_t yawRate;
        };

        struct BaseFlatReferenceTrajectory 
        {
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          std::vector<ocs2::scalar_t> time;
          std::vector<ocs2::scalar_t> yaw;
          std::vector<vector3_t> positionInWorld;
        };

        struct BaseReferenceTrajectory 
        {
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          std::vector<ocs2::scalar_t> time;
          std::vector<vector3_t> eulerZyx;
          std::vector<vector3_t> positionInWorld;
          std::vector<vector3_t> linearVelocityInWorld;
          std::vector<vector3_t> angularVelocityInWorld;
        };

        BaseTrajectoryPlanner(floating_base_model::FloatingBaseModelInfo modelInfo,
          StaticSettings settings);

        /** Update terrain model */
        void updateTerrain(const terrain_model::TerrainModel& terrainModel);

        void updateTargetTrajectory(ocs2::scalar_t initTime, 
          ocs2::scalar_t finalTime, const BaseReferenceCommand& command, 
          const state_vector_t& initialState, ocs2::TargetTrajectories& targetTrajectories);

      private:
          
        Eigen::Vector2d map2DVelocityCommandToWorld(ocs2::scalar_t headingVelocity, 
          ocs2::scalar_t lateralVelocity, ocs2:scalar_t yaw);

        BaseFlatReferenceTrajectory generate2DExtrapolatedBaseReference(ocs2::scalar_t initTime, 
          ocs2::scalar_t finalTime, const state_vector_t& initialState,
          const BaseReferenceCommand& command);

        BaseReferenceTrajectory generateExtrapolatedBaseReference(ocs2::scalar_t initTime, 
          ocs2::scalar_t finalTime, const state_vector_t& initialState,
          const BaseReferenceCommand& command);

        // BaseReferenceTrajectory generateExtrapolatedBaseReference(
        //   const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
        //   const BaseReferenceCommand& command, const grid_map::GridMap& gridMap,
        //   double nominalStanceWidthInHeading, double nominalStanceWidthLateral);
        
        floating_base_model::FloatingBaseModelInfo modelInfo_;
        StaticSettings settings_;
        
        const terrain_model::TerrainModel* terrainModel_;


    };
  }; // namespace locomotion
}; // namespace legged_locomotion_mpc

#endif