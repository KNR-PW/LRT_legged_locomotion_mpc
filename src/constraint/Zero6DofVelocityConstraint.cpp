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

#include <legged_locomotion_mpc/constraint/Zero6DofVelocityConstraint.hpp>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Zero6DofVelocityConstraint::Zero6DofVelocityConstraint(
    const LeggedReferenceManager &referenceManager,
    size_t endEffectorIndex): 
      StateInputConstraint(ConstraintOrder::Linear),
      referenceManager_(referenceManager),
      endEffectorIndex_(endEffectorIndex) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool Zero6DofVelocityConstraint::isActive(scalar_t time) const
  {
    return referenceManager_.getContactFlags(time)[endEffectorIndex_]; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Zero6DofVelocityConstraint* Zero6DofVelocityConstraint::clone() const 
  { 
    return new Zero6DofVelocityConstraint(*this); 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t Zero6DofVelocityConstraint::getNumConstraints(scalar_t time) const 
  { 
    return 6; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t Zero6DofVelocityConstraint::getValue(scalar_t time, 
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const auto linearVelocity = leggedPrecomputation.getEndEffectorLinearVelocity(endEffectorIndex_);
    const auto angularVelocity = leggedPrecomputation.getEndEffectorAngularVelocity(endEffectorIndex_);
    vector_t velocity(6);
    velocity << linearVelocity, angularVelocity;
    return velocity;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation Zero6DofVelocityConstraint::getLinearApproximation(
    scalar_t time,
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {

    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    VectorFunctionLinearApproximation velocityApprox;
    velocityApprox.f = vector_t::Zero(6);
    velocityApprox.dfdx = matrix_t(6, state.size());
    velocityApprox.dfdu = matrix_t(6, input.size());

    const auto linearVelocityApprox = leggedPrecomputation.getEndEffectorLinearVelocityDerivatives(endEffectorIndex_);
    const auto angularVelocityApprox = leggedPrecomputation.getEndEffectorAngularVelocityDerivatives(endEffectorIndex_);
    
    velocityApprox.f << linearVelocityApprox.f, angularVelocityApprox.f;
    velocityApprox.dfdx << linearVelocityApprox.dfdx,  angularVelocityApprox.dfdx;
    velocityApprox.dfdu << linearVelocityApprox.dfdu, angularVelocityApprox.dfdu;

    return velocityApprox;
  } 
} // namespace legged_locomotion_mpc