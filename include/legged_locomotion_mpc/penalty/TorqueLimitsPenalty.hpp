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
   Based on: rgrandia (https://github.com/leggedrobotics/ocs2)
 */


#ifndef __TORQUE_LIMITS_PENALTY_LEGGED_LOCOMOTION_MPC__
#define __TORQUE_LIMITS_PENALTY_LEGGED_LOCOMOTION_MPC__


#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_core/penalties/Penalties.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>


namespace legged_locomotion_mpc
{
  namespace penalty
  {
    class TorqueLimitsPenalty final: public ocs2::StateInputCost
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using TorqueApproximatorAD = legged_locomotion_mpc::PinocchioTorqueApproximationCppAd;
    
        TorqueLimitsPenalty(const floating_base_model::FloatingBaseModelInfo& info,
          const ocs2::vector_t& torqueLimits,
          ocs2::RelaxedBarrierPenalty::Config settings,
          TorqueApproximatorAD& torqueApproximator);

        /** Get cost term value */
        ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
          const ocs2::vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
          const ocs2::PreComputation& preComp) const override;
        
        /** Get cost term quadratic approximation */
        ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
          ocs2::scalar_t time, const ocs2::vector_t& state, const vector_t& input,
          const ocs2::TargetTrajectories& targetTrajectories,
          const ocs2::PreComputation& preComp) const override;

      private:

        floating_base_model::FloatingBaseModelInfo info_;

        std::unique_ptr<TorqueApproximatorAD> torqueApproximatorPtr_; // TODO DODAJ W PRECOMPUTATION
        
        std::unique_ptr<ocs2::RelaxedBarrierPenalty> torqueRelaxedBarrierPenaltyPtr_;

        ocs2::vector_t torqueLimits_;

        ocs2::ScalarFunctionQuadraticApproximation quadraticApproximation_;
    };
  }; //namespace penalty
}; // namespace legged_locomotion_mpc

#endif