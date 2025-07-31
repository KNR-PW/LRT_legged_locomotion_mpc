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
      frictionCoefficient_(frictionCoefficientParam),
      regularization_(regularizationParam),
      gripperForce_(gripperForceParam),
      hessianDiagonalShift_(hessianDiagonalShiftParam) 
  {
    assert(frictionCoefficient_ > 0.0);
    assert(regularization_ > 0.0);
    assert(hessianDiagonalShift_ >= 0.0);
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
      info_(&info) 
  {

    const size_t stateDim = info.stateDim.size();
    const size_t inputDim = info.inputDim.size();

    linearApproximation_.f = ocs2::matrix_t::Zero(1,1);
    linearApproximation_.dfdx = ocs2::matrix_t::Zero(1, stateDim);
    linearApproximation_.dfdu = ocs2::matrix_t::Zero(1, inputDim);

    quadraticApproximation_.f = ocs2::matrix_t::Zero(1,1);
    quadraticApproximation_.dfdx = ocs2::matrix_t::Zero(1, stateDim);
    quadraticApproximation_.dfdu = ocs2::matrix_t::Zero(1, inputDim);

    quadraticApproximation_.dfdxx.emplace_back(ocs2::matrix_t::Zero(stateDim, stateDim));
    quadraticApproximation_.dfdxx.diagonal().array() -= config_.hessianDiagonalShift;

    quadraticApproximation_.dfduu.emplace_back(ocs2::matrix_t::Zero(inputDim, inputDim));

    quadraticApproximation_.dfdux.emplace_back(ocs2::matrix_t::Zero(inputDim, stateDim));
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
  void ForceFrictionConeConstraint::setFrictionCoefficient(const double frictionCoefficientParam)
  {
    config_.frictionCoefficient_ = frictionCoefficientParam;
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

    const scalar_t tangentForceInverse = 1 / (sqrt(localForces.x() * localForces.x() + 
      localForces.y() * localForces.y() + config_.regularization_));

    const vector3_t dConeDF;
    dConeDF(0) = -localForces.x() * tangentForceInverse;
    dConeDF(1) = -localForces.y() * tangentForceInverse;
    dConeDF(2) = config_.frictionCoefficient_;

    linearApproximation_.f = coneConstraint(localForce);
    linearApproximation_.dfdu.block<1, 3>(0, 3 * contactPointIndex_) = dConeDF.transpose() * rotationWorldToTerrain_;
    
    return linearApproximation_;
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

    const scalar_t forceXSquared = localForces.x() * localForces.x();
    const scalar_t forceYSquared = localForces.y() * localForces.y();
    const scalar_t forceXforceY  = localForces.x() * localForces.y();

    const scalar_t tangentForceSuaredInverse = 1 / (forceXSquared + 
      forceYSquared + config_.regularization_);
    
    const scalar_t tangentForceInverse32 = tangentForceSuaredInverse * sqrt(tangentForceSuaredInverse);



    const vector3_t dConeDF;
    dConeDF(0) = -localForces.x() * tangentForceInverse;
    dConeDF(1) = -localForces.y() * tangentForceInverse;
    dConeDF(2) = config_.frictionCoefficient_;

    
    const matrix3_t d2ConeDF2 = matrix3_t::Zero();

    d2ConeDF2(0, 0) = -(forceYSquared + config_.regularization_) * tangentForceInverse32;
    d2ConeDF2(1, 1) = -(forceXSquared + config_.regularization_) * tangentForceInverse32;
    d2ConeDF2(0, 1) =  forceXforceY * * tangentForceInverse32;
    d2ConeDF2(1, 0) =  d2ConeDF2(0, 1);
    
    quadraticApproximation_.f = coneConstraint(localForce);

    quadraticApproximation_.dfdu.block<1, 3>(0, 3 * contactPointIndex_) = dConeDF.transpose() * rotationWorldToTerrain_;

    const matrix3_t d2ConeD2u = rotationWorldToTerrain_.transpose() * d2ConeDF2 * rotationWorldToTerrain_;
    quadraticApproximation_.dfduu[0].block<3, 3>(3 * contactPointIndex_, 3 * contactPointIndex_) = d2ConeD2u;
    quadraticApproximation_.dfduu[0].diagonal().array() -= config_.hessianDiagonalShift_;

    return quadraticApproximation_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::vector_t ForceFrictionConeConstraint::coneConstraint(const vector3_t &localForces) const 
  {
    const scalar_t tangentForce = sqrt(localForces.x() * localForces.x() + 
      localForces.y() * localForces.y() + config_.regularization_);

    const scalar_t frictionForce = config_.frictionCoefficient_
      * (localForces.z() + config_.gripperForce_);
      
    const scalar_t coneConstraint = frictionForce - tangentForce;
    return (ocs2::vector_t(1) << coneConstraint).finished();
  }

} // namespace legged_locomotion_mpc