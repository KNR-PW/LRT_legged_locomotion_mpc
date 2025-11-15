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

#ifndef __ZERO_3_DOF_VELOCITY_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __ZERO_3_DOF_VELOCITY_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

namespace legged_locomotion_mpc
{
  class Zero3DofVelocityConstraint final : public ocs2::StateInputConstraint {
    public:

      /**
       * Constructor
       * @param [in] referenceManager : Legged model ReferenceManager
       * @param [in] endEffectorIndex : The 3 DoF end effector index.
       */
      Zero3DofVelocityConstraint(const LeggedReferenceManager& referenceManager,
        size_t endEffectorIndex);

      ~Zero3DofVelocityConstraint() override = default;

      Zero3DofVelocityConstraint *clone() const override;

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override;

      ocs2::vector_t getValue(ocs2::scalar_t time,
        const ocs2::vector_t &state, const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

      ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time,
        const ocs2::vector_t &state, const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

    private:
      Zero3DofVelocityConstraint(const Zero3DofVelocityConstraint &rhs) = default;

      const LeggedReferenceManager& referenceManager_;
      const size_t endEffectorIndex_;
    };

} // namespace legged_locomotion_mpc

#endif