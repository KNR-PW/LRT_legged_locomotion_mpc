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

#include <legged_locomotion_mpc/constraint/NormalVelocityConstraint.hpp>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  NormalVelocityConstraint::NormalVelocityConstraint(
    const LeggedReferenceManager& referenceManager,
    size_t endEffectorIndex): 
      StateInputConstraint(ConstraintOrder::Linear),
      referenceManager_(referenceManager),
      endEffectorIndex_(endEffectorIndex) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  NormalVelocityConstraint* NormalVelocityConstraint::clone() const
  { 
    return new NormalVelocityConstraint(*this); 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool NormalVelocityConstraint::isActive(scalar_t time) const
  {
    return !referenceManager_.getContactFlags(time)[endEffectorIndex_]; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t NormalVelocityConstraint::getNumConstraints(scalar_t time) const 
  { 
    return 1; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t NormalVelocityConstraint::getValue(scalar_t time, 
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const vector3_t& surfaceNormal = leggedPrecomputation.getSurfaceNormal(
      endEffectorIndex_);

    const vector3_t& velocity = 
      leggedPrecomputation.getEndEffectorLinearVelocity(endEffectorIndex_);
    
    const vector3_t& referenceVelocity = 
      leggedPrecomputation.getReferenceEndEffectorLinearVelocity(endEffectorIndex_);

    return (vector_t(1) << surfaceNormal.dot(velocity - referenceVelocity)).finished();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation NormalVelocityConstraint::getLinearApproximation(
    scalar_t time,
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const vector3_t& surfaceNormal = leggedPrecomputation.getSurfaceNormal(
      endEffectorIndex_);

    const vector3_t& velocity = 
      leggedPrecomputation.getEndEffectorLinearVelocity(endEffectorIndex_);
    
    const vector3_t& referenceVelocity = 
      leggedPrecomputation.getReferenceEndEffectorLinearVelocity(endEffectorIndex_);

    const auto& velocityDerivatives = 
      leggedPrecomputation.getEndEffectorLinearVelocityDerivatives(endEffectorIndex_);

    VectorFunctionLinearApproximation constraint;
    
    constraint.f = (vector_t(1) << surfaceNormal.dot(velocity - referenceVelocity)).finished();
    constraint.dfdx = surfaceNormal.transpose() * velocityDerivatives.dfdx;
    constraint.dfdu = surfaceNormal.transpose() * velocityDerivatives.dfdu;

    return constraint;
  }
} // namespace legged_locomotion_mpc