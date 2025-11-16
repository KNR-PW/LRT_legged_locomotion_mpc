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
 * Based on: rgrandia (https://github.com/leggedrobotics/ocs2)
 */

#include <legged_locomotion_mpc/penalty/TorqueLimitsPenalty.hpp>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TorqueLimitsPenalty::TorqueLimitsPenalty(
    FloatingBaseModelInfo info,
    const vector_t torqueLimits,
    RelaxedBarrierPenalty::Config settings,
    const PinocchioTorqueApproximationCppAd& torqueApproximator):
      StateInputCost(),
      info_(std::move(info)),
      torqueApproximator_(torqueApproximator),
      torqueRelaxedBarrierPenaltyPtr_(new RelaxedBarrierPenalty(settings)),
      torqueLimits_(std::move(torqueLimits))
  {
    // checking size of torqueLimits vector (complicated way :/)
    const vector_t sampleState = vector_t::Random(info_.stateDim);
    const vector_t sampleInput = vector_t::Random(info_.inputDim);

    const vector_t sampleTorques = torqueApproximator_.getValue(sampleState, sampleInput);
    assert(torqueLimits_.rows() == sampleTorques.rows());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TorqueLimitsPenalty *TorqueLimitsPenalty::clone() const
  {
    return new TorqueLimitsPenalty(*this);
  }
    
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t TorqueLimitsPenalty::getValue(scalar_t time, const vector_t& state,
    const vector_t& input, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const vector_t& torqueApprox = leggedPrecomputation.getApproximatedJointTorques();
    
    const vector_t upperBoundTorqueOffset = torqueLimits_ - torqueApprox;
    const vector_t lowerBoundTorqueOffset = torqueLimits_ + torqueApprox;

    return upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
    {
      return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
    }).sum() + lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
    {
      return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
    }).sum();
  }
    
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ScalarFunctionQuadraticApproximation TorqueLimitsPenalty::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const VectorFunctionLinearApproximation& torqueApproxDerivative = 
      leggedPrecomputation.getApproximatedJointTorquesDerivatives();

    const vector_t upperBoundTorqueOffset = torqueLimits_ - torqueApproxDerivative.f;
    const vector_t lowerBoundTorqueOffset = torqueLimits_ + torqueApproxDerivative.f;

    const size_t forceSize = 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;
    
    const auto dTorquedQ = torqueApproxDerivative.dfdx.block(0, 6, info_.actuatedDofNum, info_.generalizedCoordinatesNum);
    const auto dTorquedF = torqueApproxDerivative.dfdu.block(0, 0, info_.actuatedDofNum, forceSize);
    
    ScalarFunctionQuadraticApproximation cost;

    cost.dfdx = vector_t::Zero(info_.stateDim);
    cost.dfdu = vector_t::Zero(info_.inputDim);

    cost.f = upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
    {
      return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
    }).sum() + lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi)
    {
      return torqueRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
    }).sum();

    // Penalty derivatives w.r.t. the constraint
    const vector_t penaltyDerivatives = lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi)
    {
      return torqueRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
    }) - upperBoundTorqueOffset.unaryExpr([&](scalar_t hi)
    { 
      return torqueRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi); 
    });

    const vector_t penaltySecondDerivatives = lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi)
    {
        return torqueRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
    }) + upperBoundTorqueOffset.unaryExpr([&](scalar_t hi)
    {
        return torqueRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
    });

    const auto [hessianStateState, hessianInputState] = torqueApproximator_.getWeightedHessians(penaltyDerivatives, state, input);

    cost.dfdx.block(6, 0, info_.generalizedCoordinatesNum, 1).noalias() =  dTorquedQ.transpose() * penaltyDerivatives;
    cost.dfdu.block(0, 0, forceSize, 1).noalias() =  dTorquedF.transpose() * penaltyDerivatives;
    cost.dfdxx.block(6, 6, info_.generalizedCoordinatesNum, 
      info_.generalizedCoordinatesNum).noalias() = hessianStateState + 
        dTorquedQ.transpose() * penaltySecondDerivatives.asDiagonal() * dTorquedQ;

    cost.dfduu.block(0, 0, forceSize, forceSize).noalias() = 
      dTorquedF.transpose() * penaltySecondDerivatives.asDiagonal() * dTorquedF; // torque is linearly dependent on forces, so its hessian is zero
    
    cost.dfdux.block(0, 6, forceSize, info_.generalizedCoordinatesNum).noalias() = 
      hessianInputState + 
        dTorquedF.transpose() * penaltySecondDerivatives.asDiagonal() * dTorquedQ;

    return cost;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TorqueLimitsPenalty::TorqueLimitsPenalty(const TorqueLimitsPenalty &rhs):
    info_(rhs.info_), torqueApproximator_(rhs.torqueApproximator_),
    torqueRelaxedBarrierPenaltyPtr_(rhs.torqueRelaxedBarrierPenaltyPtr_->clone()),
    torqueLimits_(rhs.torqueLimits_) {}
} // namespace legged_locomotion_mpc