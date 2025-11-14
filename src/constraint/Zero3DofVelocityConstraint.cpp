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

#include <legged_locomotion_mpc/constraint/Zero3DofVelocityConstraint.hpp>
#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Zero3DofVelocityConstraint::Zero3DofVelocityConstraint(
    const LeggedReferenceManager& referenceManager,
    size_t endEffectorIndex): 
      StateInputConstraint(ConstraintOrder::Linear),
      referenceManager_(referenceManager),
      endEffectorIndex_(endEffectorIndex) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool Zero3DofVelocityConstraint::isActive(scalar_t time) const
  {
    return referenceManager_.getContactFlags(time)[endEffectorIndex_]; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Zero3DofVelocityConstraint* Zero3DofVelocityConstraint::clone() const
  { 
    return new Zero3DofVelocityConstraint(*this); 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t Zero3DofVelocityConstraint::getNumConstraints(scalar_t time) const
  { 
    return 3; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t Zero3DofVelocityConstraint::getValue(scalar_t time, 
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    return leggedPrecomputation.getEndEffectorPosition(endEffectorIndex_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation Zero3DofVelocityConstraint::getLinearApproximation(
    scalar_t time,
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    return leggedPrecomputation.getEndEffectorPositionDerivatives(endEffectorIndex_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Zero3DofVelocityConstraint::Zero3DofVelocityConstraint(
    const Zero3DofVelocityConstraint &rhs):
    StateInputConstraint(rhs),
    referenceManager_(rhs.referenceManager_),
    endEffectorIndex_(rhs.endEffectorIndex_) {}
    
} // namespace legged_locomotion_mpc