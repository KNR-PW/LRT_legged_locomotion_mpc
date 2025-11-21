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

#ifndef __TERRAIN_AVOIDANCE_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __TERRAIN_AVOIDANCE_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  /**
   * Constraint that ensures that end effectors do not go through terrain:
   * max(terrain_sdf(pos_ee)) - r_e + relax > 0 where:
   * terrain_sdf - terrain sdf 
   * pos_ee - all end effector sphere approximation positons
   * r_e - radius of end effector sphere
   * relax - relaxation constraint value
   */
  class TerrainAvoidanceSoftConstraint final: public ocs2::StateInputCost 
  {
    public:

      /**
       * Constructor
       * @param [in] info: Floating Base model info.
       * @param [in] sphereInterface: interface for sphere approximation
       * @param [in] relaxation: Relax constraint values.
       * @param [in] jointVelocityLimits: Maximum joint velocities.
       * @param [in] settings: Relaxed barrier penalty settings 
       * 
       */
     TerrainAvoidanceSoftConstraint(floating_base_model::FloatingBaseModelInfo info,
        ocs2::vector_t jointPositionUpperLimits,
        ocs2::vector_t jointPositionLowerLimits,
        ocs2::vector_t jointVelocityLimits,
        ocs2::RelaxedBarrierPenalty::Config settings);

      ~JointLimitsSoftConstraint() override = default;

     TerrainAvoidanceSoftConstraint* clone() const override;

      /** Get cost term value */
      ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
        const ocs2::vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
        const ocs2::PreComputation& preComp) const override;

      /** Get cost term quadratic approximation */
      ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::vector_t& input,
        const ocs2::TargetTrajectories& targetTrajectories,
        const ocs2::PreComputation& preComp) const override;

    private:
     TerrainAvoidanceSoftConstraint(constTerrainAvoidanceSoftConstraint &rhs);
        
      const floating_base_model::FloatingBaseModelInfo info_;
      const ocs2::vector_t jointPositionUpperLimits_;
      const ocs2::vector_t jointPositionLowerLimits_;
      const ocs2::vector_t jointVelocityLimits_;

      std::unique_ptr<ocs2::RelaxedBarrierPenalty> jointRelaxedBarrierPenaltyPtr_;

      /**
       * Get partial derivatives of rotation matrix times vector 
       * with respect to euler angles
       */
      std::unique_ptr<ocs2::CppAdInterface> rotationMatrixVectorAdInterfacePtr_;

    };

} // namespace legged_locomotion_mpc

#endif