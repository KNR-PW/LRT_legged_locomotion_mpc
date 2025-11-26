// Copyright (c) 2025, Bartłomiej Krajewski
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

#ifndef __TERRAIN_AVOIDANCE_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __TERRAIN_AVOIDANCE_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/collision/PinocchoCollisionInterface.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

namespace legged_locomotion_mpc
{
  /**
   * Constraint that ensures that end effectors do not go through terrain:
   * min(terrain_sdf(pos_ee)) - r_e + relax > 0 where:
   * terrain_sdf - terrain sdf 
   * pos_ee - all sphere approximation positons of end effector or collision link
   * r_e - radius of end effector or collision link sphere
   * relax - relaxation constraint value
   */
  class TerrainAvoidanceSoftConstraint final: public ocs2::StateCost 
  {
    public:

      /**
       * Constructor
       * @param [in] info: Floating Base model info.
       * @param [in] sphereInterface: interface for sphere approximation
       * @param [in] referenceManager: Legged Reference Manager
       * @param [in] collisionIndices: Indexes of end effectors (0 : endEffectorNum - 1)
       * or collision links (endEffectorNum : endEffectorNum + collisionNum - 1).
       * @param [in] relaxations: Relax constraint values (for end effectors and 
       * collision links).
       * @param [in] settings: Relaxed barrier penalty settings
       * 
       * @warning All end effectors ale by default added, in collisionIndices include 
       * only additional links defined as collision links
       */
      TerrainAvoidanceSoftConstraint(
        floating_base_model::FloatingBaseModelInfo info,
        const collision::PinocchioCollisionInterface& collisionInterface,
        const LeggedReferenceManager& referenceManager,
        std::vector<size_t> collisionIndices,
        std::vector<ocs2::scalar_t> relaxations,
        ocs2::RelaxedBarrierPenalty::Config settings);

      ~TerrainAvoidanceSoftConstraint() override = default;

      TerrainAvoidanceSoftConstraint* clone() const override;

      /** Get cost term value */
      ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::TargetTrajectories& targetTrajectories, 
        const ocs2::PreComputation& preComp) const override;

      /** Get cost term quadratic approximation */
      ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::TargetTrajectories& targetTrajectories, 
        const ocs2::PreComputation& preComp) const override;

    private:
      TerrainAvoidanceSoftConstraint(const TerrainAvoidanceSoftConstraint& rhs);

      const size_t threeDofEndEffectorNum_;
      const size_t sixDofEndEffectorNum_;
      const size_t endEffectorNum_;
      const std::vector<size_t> collisionLinkIndicies_;

      const std::vector<ocs2::scalar_t> relaxations_;

      const LeggedReferenceManager& referenceManager_;
      const collision::PinocchioCollisionInterface& collisionInterface_;
      
      std::unique_ptr<ocs2::RelaxedBarrierPenalty> terrainAvoidancePenaltyPtr_;
  };
} // namespace legged_locomotion_mpc

#endif