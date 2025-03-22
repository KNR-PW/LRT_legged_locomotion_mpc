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

#include "legged_locomotion_mpc/constraint/ForceFrictionConeConstraint.hpp"

namespace legged_locomotion_mpc
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  explicit ForceFrictionConeConstraint::Config::Config(
    ocs2::scalar_t frictionCoefficientParam,
    ocs2::scalar_t regularizationParam,
    ocs2::scalar_t gripperForceParam,
    ocs2::scalar_t hessianDiagonalShiftParam): 
      frictionCoefficient(frictionCoefficientParam),
      regularization(regularizationParam),
      gripperForce(gripperForceParam),
      hessianDiagonalShift(hessianDiagonalShiftParam) 
    {
      assert(frictionCoefficient > 0.0);
      assert(regularization > 0.0);
      assert(hessianDiagonalShift >= 0.0);
    }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeConstraint::ForceFrictionConeConstraint(const SwitchedModelReferenceManager &referenceManager,
    Config config,
    size_t contactPointIndex,
    FloatingBaseModelInfo& info): 
      StateInputConstraint(ocs2::ConstraintOrder::Quadratic),
      referenceManagerPtr_(&referenceManager),
      config_(config),
      contactPointIndex_(contactPointIndex),
      info_(&info) {
  }

  // TODO: Dodaj jak będzie ogarnięty interface od Kuby!
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void ForceFrictionConeConstraint::setSurfaceNormalInWorld(const vector3_t &surfaceNormalInWorld) 
  {
    rotationWorldToTerrain_.setIdentity();
    throw std::runtime_error("[ForceFrictionConeConstraint] setSurfaceNormalInWorld() is not implemented!");
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool ForceFrictionConeConstraint::isActive(ocs2::scalar_t time) const 
  {
    return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::vector_t ForceFrictionConeConstraint::getValue(scalar_t time,
    const ocs2::vector_t &state,
    const ocs2::vector_t &input,
    const ocs2::PreComputation &preComp) const 
  {
    const auto forcesInWorldFrame = access_helper_functions::getContactForces(input, contactPointIndex_, *info_);
    const vector3_t localForce = rotationWorldToTerrain_ * forcesInWorldFrame;
    return coneConstraint(localForce);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::VectorFunctionLinearApproximation ForceFrictionConeConstraint::getLinearApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t &state,
    const ocs2::vector_t &input,
    const ocs2::PreComputation &preComp) const 
  {
    const vector3_t forcesInWorldFrame = access_helper_functions::getContactForces(input, contactPointIndex_, *info_);
    const vector3_t localForce = rotationWorldToTerrain_ * forcesInWorldFrame;
    const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame);
    const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
    const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);
    ocs2::VectorFunctionLinearApproximation linearApproximation;
    linearApproximation.f = coneConstraint(localForce);
    linearApproximation.dfdx = ocs2::matrix_t::Zero(1, state.size());
    linearApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
    return linearApproximation;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::VectorFunctionQuadraticApproximation ForceFrictionConeConstraint::getQuadraticApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t &state,
    const ocs2::vector_t &input,
    const ocs2::PreComputation &preComp) const 
  {
    const vector3_t forcesInWorldFrame = access_helper_functions::getContactForces(input, contactPointIndex_, *info_);
    const vector3_t localForce = rotationWorldToTerrain_ * forcesInWorldFrame;
    const auto localForceDerivatives = computeLocalForceDerivatives(forcesInWorldFrame);
    const auto coneLocalDerivatives = computeConeLocalDerivatives(localForce);
    const auto coneDerivatives = computeConeConstraintDerivatives(coneLocalDerivatives, localForceDerivatives);
    ocs2::VectorFunctionQuadraticApproximation quadraticApproximation;
    quadraticApproximation.f = coneConstraint(localForce);
    quadraticApproximation.dfdx = ocs2::matrix_t::Zero(1, state.size());
    quadraticApproximation.dfdu = frictionConeInputDerivative(input.size(), coneDerivatives);
    quadraticApproximation.dfdxx.emplace_back(frictionConeSecondDerivativeState(state.size(), coneDerivatives));
    quadraticApproximation.dfduu.emplace_back(frictionConeSecondDerivativeInput(input.size(), coneDerivatives));
    quadraticApproximation.dfdux.emplace_back(ocs2::matrix_t::Zero(input.size(), state.size()));
    return quadraticApproximation;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeConstraint::LocalForceDerivatives ForceFrictionConeConstraint::computeLocalForceDerivatives(
      const vector3_t &forcesInWorldFrame) const 
  {
    LocalForceDerivatives localForceDerivatives{};
    localForceDerivatives.dF_du = rotationWorldToTerrain_;
    return localForceDerivatives;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeConstraint::ConeLocalDerivatives ForceFrictionConeConstraint::computeConeLocalDerivatives(
    const vector3_t &localForces) const 
  {
    const auto F_x_square = localForces.x() * localForces.x();
    const auto F_y_square = localForces.y() * localForces.y();
    const auto F_tangent_square = F_x_square + F_y_square + config_.regularization;
    const auto F_tangent_norm = sqrt(F_tangent_square);
    const auto F_tangent_square_pow32 = F_tangent_norm * F_tangent_square; // = F_tangent_square ^ (3/2)

    ConeLocalDerivatives coneDerivatives{};
    coneDerivatives.dCone_dF(0) = -localForces.x() / F_tangent_norm;
    coneDerivatives.dCone_dF(1) = -localForces.y() / F_tangent_norm;
    coneDerivatives.dCone_dF(2) = config_.frictionCoefficient;

    coneDerivatives.d2Cone_dF2(0, 0) = -(F_y_square + config_.regularization) / F_tangent_square_pow32;
    coneDerivatives.d2Cone_dF2(0, 1) = localForces.x() * localForces.y() / F_tangent_square_pow32;
    coneDerivatives.d2Cone_dF2(0, 2) = 0.0;
    coneDerivatives.d2Cone_dF2(1, 0) = coneDerivatives.d2Cone_dF2(0, 1);
    coneDerivatives.d2Cone_dF2(1, 1) = -(F_x_square + config_.regularization) / F_tangent_square_pow32;
    coneDerivatives.d2Cone_dF2(1, 2) = 0.0;
    coneDerivatives.d2Cone_dF2(2, 0) = 0.0;
    coneDerivatives.d2Cone_dF2(2, 1) = 0.0;
    coneDerivatives.d2Cone_dF2(2, 2) = 0.0;

    return coneDerivatives;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::vector_t ForceFrictionConeConstraint::coneConstraint(const vector3_t &localForces) const 
  {
    const auto F_tangent_square = localForces.x() * localForces.x() + localForces.y() * localForces.y() + 
      config_.regularization;

    const auto F_tangent_norm = sqrt(F_tangent_square);

    const scalar_t coneConstraint = config_.frictionCoefficient * (localForces.z() + config_.gripperForce) -
      F_tangent_norm;
    return (ocs2::vector_t(1) << coneConstraint).finished();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeConstraint::ConeDerivatives ForceFrictionConeConstraint::computeConeConstraintDerivatives(
      const ConeLocalDerivatives &coneLocalDerivatives,
      const LocalForceDerivatives &localForceDerivatives) const 
  {
    ConeDerivatives coneDerivatives;
    // First order derivatives
    coneDerivatives.dCone_du.noalias() = coneLocalDerivatives.dCone_dF.transpose() * localForceDerivatives.dF_du;

    // Second order derivatives
    coneDerivatives.d2Cone_du2.noalias() =
      localForceDerivatives.dF_du.transpose() * coneLocalDerivatives.d2Cone_dF2 * localForceDerivatives.dF_du;

    return coneDerivatives;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::matrix_t ForceFrictionConeConstraint::frictionConeInputDerivative(size_t inputDim,
    const ConeDerivatives &coneDerivatives) const 
  {
    ocs2::matrix_t dhdu = ocs2::matrix_t::Zero(1, inputDim);
    dhdu.block<1, 3>(0, 3 * contactPointIndex_) = coneDerivatives.dCone_du;
    return dhdu;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::matrix_t ForceFrictionConeConstraint::frictionConeSecondDerivativeInput(size_t inputDim,
    const ConeDerivatives &coneDerivatives) const
  {
    ocs2::matrix_t ddhdudu = ocs2::matrix_t::Zero(inputDim, inputDim);
    ddhdudu.block<3, 3>(3 * contactPointIndex_, 3 * contactPointIndex_) = coneDerivatives.d2Cone_du2;
    ddhdudu.diagonal().array() -= config_.hessianDiagonalShift;
    return ddhdudu;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::matrix_t ForceFrictionConeConstraint::frictionConeSecondDerivativeState(size_t stateDim,
    const ConeDerivatives &coneDerivatives) const 
  {
    ocs2::matrix_t ddhdxdx = ocs2::matrix_t::Zero(stateDim, stateDim);
    ddhdxdx.diagonal().array() -= config_.hessianDiagonalShift;
    return ddhdxdx;
  }

} // namespace legged_locomotion_mpc