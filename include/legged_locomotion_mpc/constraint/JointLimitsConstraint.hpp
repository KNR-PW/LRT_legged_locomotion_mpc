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

#ifndef __JOINT_LIMITS_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __JOINT_LIMITS_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>

#include <legged_locomotion_mpc/common/ModelSettings.hpp>

namespace legged_locomotion_mpc
{
  /**
   * Constraint that satisfies joint position and velocity limits (not torques)
   */
  class JointLimitsConstraint final : public ocs2::StateInputConstraint 
  {
    public:

      /**
       * Constructor
       * @param [in] info : Floating Base Model info.
       * @param [in] jointPositionUpperLimits : Maximum joint positions (angles).
       * @param [in] jointPositionLowerLimits : Minimum joint positions (angles).
       * @param [in] jointVelocityLimits : Maximum joint velocities.
       * 
       */
      JointLimitsConstraint(const floating_base_model::FloatingBaseModelInfo& info,
        const ocs2::vector_t& jointPositionUpperLimits,
        const ocs2::vector_t& jointPositionLowerLimits,
        const ocs2::vector_t& jointVelocityLimits);

      ~JointLimitsConstraint() override = default;

      JointLimitsConstraint* clone() const override;

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override;

      ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t &state,
        const ocs2::vector_t &input, const PreComputation &preComp) const override;

      ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time,
        const ocs2::vector_t &state, const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

    private:
      JointLimitsConstraint(const JointLimitsConstraint &rhs);
        
      floating_base_model::FloatingBaseModelInfo info_;
      ocs2::vector_t jointPositionUpperLimits_;
      ocs2::vector_t jointPositionLowerLimits_;
      ocs2::vector_t jointVelocityLimits_;

      size_t numConstraints_;
      
      // Cached constant gradients
      ocs2::matrix_t cachedStateGradient_;
      ocs2::matrix_t cachedInputGradient_;

    };

} // namespace legged_locomotion_mpc

#endif