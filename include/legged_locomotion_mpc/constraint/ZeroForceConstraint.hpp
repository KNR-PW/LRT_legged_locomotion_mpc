/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2), 2025

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __ZERO_FORCE_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __ZERO_FORCE_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

namespace legged_locomotion_mpc
{
  class ZeroForceConstraint final : public ocs2::StateInputConstraint 
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:

      /** 
       * Constructor
       * @param [in] referenceManager: Legged model ReferenceManager.
       * @param [in] info: info of FloatingBase model.
       * @param [in] endEffectorIndex: The 3 DoF end effector index.
       */
      ZeroForceConstraint(const LeggedReferenceManager& referenceManager,
        floating_base_model::FloatingBaseModelInfo info,
        size_t endEffectorIndex);

      ~ZeroForceConstraint() override = default;

      ZeroForceConstraint* clone() const override;

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override;

      ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
        const ocs2::vector_t& input, const ocs2::PreComputation& preComp) const override;

      ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, 
        const ocs2::vector_t& state, const ocs2::vector_t& input,
        const ocs2::PreComputation& preComp) const override;

    private:

      ZeroForceConstraint(const ZeroForceConstraint &other) = default;

      const LeggedReferenceManager& referenceManager_;
      const floating_base_model::FloatingBaseModelInfo info_;
      const size_t endEffectorIndex_;

  };

} // namespace legged_locomotion_mpc

#endif