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

#ifndef __QUADRATIC_TRACKING_COST_LEGGED_LOCOMOTION_MPC__
#define __QUADRATIC_TRACKING_COST_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/Utils.hpp>
#include <legged_locomotion_mpc/reference_manager/SwitchedModelReferenceManager.hpp>

namespace legged_locomotion_mpc 
{
  /**
  * State-input tracking cost used for intermediate times
  */
  class QuadraticTrackingCost final : public StateInputCost 
  {
    public:
     /**
      * Constructor for the quadratic cost function defined as the following:
      * \f$ L = 0.5(x-x_{n})' Q (x-x_{n}) + 0.5(u-u_{n})' R (u-u_{n}) \f$
      * @param [in] Q: \f$ Q \f$
      * @param [in] R: \f$ R \f$
      */
      QuadraticTrackingCost(ocs2::vector_t Q, ocs2::vector_t R,
        floating_base_model::FloatingBaseModelInfo& info,
        const LeggedSynchronizedModule &leggedSynchronizedModule
        const SwitchedModelReferenceManager& referenceManager);

      ~QuadraticTrackingCost() override = default;

      QuadraticTrackingCost *clone() const override;

      /** Get cost term value */
      ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
        const ocs2::vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
        const ocs2::PreComputation&) final;

      /** Get cost term quadratic approximation */
      ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(ocs2::scalar_t time,
        const ocs2::vector_t& state, const vector_t& input,
        const ocs2::TargetTrajectories& targetTrajectories,
        const ocs2::PreComputation&) final;

  private:

      QuadraticTrackingCost(const QuadraticTrackingCost &rhs) = default;

      std::pair<vector_t, vector_t> getStateInputDeviation(scalar_t time, const vector_t &state,
        const vector_t &input,
        const TargetTrajectories &targetTrajectories) const;


      const floating_base_model::FloatingBaseModelInfo* info_;
      const LeggedSynchronizedModule* leggedSynchronizedModulePtr_; //TODO DODAJ TEN MODUL
      const SwitchedModelReferenceManager *referenceManagerPtr_;

      /** Diagonal matrixes defined as vectors */
      ocs2::vector_t Q_;
      ocs2::vector_t R_;
      
  };
  /**
  * State tracking cost used for the final time
  */
  class QuadraticFinalTrackingCost final : public QuadraticStateCost 
  {
  public:
      QuadraticFinalTrackingCost(matrix_t Q, CentroidalModelInfo info,
        const SwitchedModelReferenceManager &referenceManager)
        
      ~QuadraticFinalTrackingCost() override = default;

      QuadraticFinalTrackingCost *clone() const override;

  private:
    matrix_t Q_;
    matrix_t R_;

      QuadraticFinalTrackingCost(const QuadraticFinalTrackingCost &rhs) = default;


      const floating_base_model::FloatingBaseModelInfo* info_;
      const LeggedSynchronizedModule* leggedSynchronizedModule_; //TODO DODAJ TEN MODUL
      const SwitchedModelReferenceManager *referenceManagerPtr_;
  };
};

#endif