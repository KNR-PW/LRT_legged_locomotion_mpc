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


#ifndef __DYNAMICS_LEGGED_LOCOMOTION_MPC__
#define __DYNAMICS_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

#include <floating_base_model/PinocchioFloatingBaseDynamicsAD.hpp>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/logic/LeggedSynchronizedModule.hpp>

namespace legged_locomotion_mpc 
{
  class LeggedDynamicsAD final : public ocs2::SystemDynamicsBase 
  {
    public:
      LeggedDynamicsAD(const ocs2::PinocchioInterface &pinocchioInterface,
        const floating_base_model::FloatingBaseModelInfo &info,
        const std::string &modelName,
        const ModelSettings &modelSettings,
        const LeggedSynchronizedModule& synchronizedModule);

      ~LeggedDynamicsAD() override = default;

      LeggedDynamicsAD *clone() const override { return new LeggedDynamicsAD(*this); }

      ocs2::vector_t computeFlowMap(ocs2::scalar_t time, 
        const ocs2::vector_t &state, const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) override;

      ocs2::VectorFunctionLinearApproximation linearApproximation(ocs2::scalar_t time,
        const ocs2::vector_t &state, const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) override;

    private:
      LeggedDynamicsAD(const LeggedDynamicsAD &rhs) = default;

      floating_base_model::PinocchioFloatingBaseDynamicsAD dynamicsAdPtr_;
      const LeggedSynchronizedModule* leggedSynchronizedModule_;

      Eigen::Matrix<ocs2::scalar_t, 6, 1> floatingBaseDisturbance_ = Eigen::Matrix<ocs2::scalar_t, 6, 1>::Zero();
    };

}; // namespace legged_locomotion_mpc

#endif