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

#ifndef __JOINT_LIMITS_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __JOINT_LIMITS_SOFT_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  /**
   * Constraint that satisfies joint position and velocity limits (not torques)
   */
  class JointLimitsSoftConstraint final: public ocs2::StateInputCost 
  {
    public:

      /**
       * Constructor
       * @param [in] info: Floating Base model info.
       * @param [in] jointPositionUpperLimits: Maximum joint positions.
       * @param [in] jointPositionLowerLimits: Minimum joint positions.
       * @param [in] jointVelocityLimits: Maximum joint velocities.
       * @param [in] settings: Relaxed barrier penalty settings 
       * 
       */
      JointLimitsSoftConstraint(floating_base_model::FloatingBaseModelInfo info,
        ocs2::vector_t jointPositionUpperLimits,
        ocs2::vector_t jointPositionLowerLimits,
        ocs2::vector_t jointVelocityLimits,
        ocs2::RelaxedBarrierPenalty::Config settings);

      ~JointLimitsSoftConstraint() override = default;

      JointLimitsSoftConstraint* clone() const override;

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
      JointLimitsSoftConstraint(const JointLimitsSoftConstraint &rhs);
        
      const floating_base_model::FloatingBaseModelInfo info_;
      const ocs2::vector_t jointPositionUpperLimits_;
      const ocs2::vector_t jointPositionLowerLimits_;
      const ocs2::vector_t jointVelocityLimits_;

      std::unique_ptr<ocs2::RelaxedBarrierPenalty> jointRelaxedBarrierPenaltyPtr_;

    };

} // namespace legged_locomotion_mpc

#endif