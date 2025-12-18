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

#include <legged_locomotion_mpc/cost/TerminalTrackingCost.hpp>

namespace legged_locomotion_mpc 
{
  namespace cost
  {

    using namespace ocs2;
    using namespace floating_base_model;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    TerminalTrackingCost::TerminalTrackingCost(matrix_t Q,
      FloatingBaseModelInfo info): 
      StateCost(), Q_(std::move(Q)) 
    {
      if(Q_.rows() != Q_.cols() || Q_.rows() != info.stateDim)
      {
        throw std::invalid_argument("[TerminalTrackingCost]: Wrong size of Q matrix!");
      }
    }

    TerminalTrackingCost* TerminalTrackingCost::clone() const
    {
      return new TerminalTrackingCost(*this);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    scalar_t TerminalTrackingCost::getValue(scalar_t time, const vector_t& state,
      const TargetTrajectories& targetTrajectories,
      const PreComputation&) const
    {
      const vector_t stateDeviation = getStateDeviation(time, state, targetTrajectories);
      const scalar_t cost = 0.5 * stateDeviation.dot(Q_ * stateDeviation);

      return cost;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    ScalarFunctionQuadraticApproximation TerminalTrackingCost::getQuadraticApproximation(
      scalar_t time, const vector_t& state, 
      const TargetTrajectories& targetTrajectories, const PreComputation&) const
    {
      ScalarFunctionQuadraticApproximation cost;

      vector_t stateDeviation = getStateDeviation(time, state, targetTrajectories);
      
      const vector_t weightedStateDeviation = Q_ * stateDeviation;
      cost.f = 0.5 * stateDeviation.dot(weightedStateDeviation);
      cost.dfdx = weightedStateDeviation;
      cost.dfdxx = Q_;
  
      return cost;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    vector_t TerminalTrackingCost::getStateDeviation(scalar_t time,
      const vector_t &state, const TargetTrajectories &targetTrajectories) const
    {
      const vector_t xNominal = targetTrajectories.getDesiredState(time);
      return {state - xNominal};
    }
  } // namespace cost
} // namespace legged_locomotion_mpc