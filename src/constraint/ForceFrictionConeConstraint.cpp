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

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  explicit ForceFrictionConeConstraint::Config::Config(
    scalar_t frictionCoefficientParam,
    scalar_t regularizationParam,
    scalar_t gripperForceParam,
    scalar_t hessianDiagonalShiftParam): 
      frictionCoefficient_(frictionCoefficientParam),
      regularization_(regularizationParam),
      hessianDiagonalShift_(hessianDiagonalShiftParam) 
  {
    assert(frictionCoefficient_ > 0.0);
    assert(regularization_ > 0.0);
    assert(hessianDiagonalShift_ >= 0.0);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeConstraint::ForceFrictionConeConstraint(
    const SwitchedModelReferenceManager &referenceManager, Config config,
    size_t contactPointIndex, FloatingBaseModelInfo& info): 
      StateInputConstraint(ConstraintOrder::Quadratic),
      referenceManagerPtr_(&referenceManager),
      config_(config),
      contactPointIndex_(contactPointIndex),
      info_(std::move(info)) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool ForceFrictionConeConstraint::isActive(scalar_t time) const 
  {
    return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t ForceFrictionConeConstraint::getValue(scalar_t time,
    const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const 
  {
    const auto forcesInWorldFrame = access_helper_functions::getContactForces(input, contactPointIndex_, *info_);
    const matrix3_t rotationMatrixToTerrain = referenceManagerPtr_->
    const vector3_t localForce = rotationWorldToTerrain_ * forcesInWorldFrame;
    return coneConstraint(localForce);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation ForceFrictionConeConstraint::getLinearApproximation(
    scalar_t time,
    const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const 
  {
    const vector3_t forcesInWorldFrame = access_helper_functions::getContactForces(input, contactPointIndex_, *info_);
    const vector3_t localForce = rotationWorldToTerrain_ * forcesInWorldFrame;

    const scalar_t tangentForceInverse = 1 / (sqrt(localForces.x() * localForces.x() + 
      localForces.y() * localForces.y() + config_.regularization_));

    const vector3_t dConeDF;
    dConeDF(0) = -localForces.x() * tangentForceInverse;
    dConeDF(1) = -localForces.y() * tangentForceInverse;
    dConeDF(2) = config_.frictionCoefficient_;


    VectorFunctionLinearApproximation linearApprox;

    const size_t stateDim = info.stateDim.size();
    const size_t inputDim = info.inputDim.size();
    linearApprox.dfdx = matrix_t::Zero(1, stateDim);
    linearApprox.dfdu = matrix_t::Zero(1, inputDim);

    linearApprox.f = coneConstraint(localForce);
    linearApprox.dfdu.block<1, 3>(0, 3 * contactPointIndex_) = dConeDF.transpose() * rotationWorldToTerrain_;
    
    return linearApprox;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionQuadraticApproximation ForceFrictionConeConstraint::getQuadraticApproximation(
    scalar_t time,
    const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const 
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
    d2ConeDF2(0, 1) =  forceXforceY * tangentForceInverse32;
    d2ConeDF2(1, 0) =  d2ConeDF2(0, 1);


    VectorFunctionQuadraticApproximation quadraticApprox;

    quadraticApprox.f = coneConstraint(localForce);
    
    const size_t stateDim = info.stateDim.size();
    const size_t inputDim = info.inputDim.size();

    quadraticApprox.dfdx = matrix_t::Zero(1, stateDim);
    quadraticApprox.dfdu = matrix_t::Zero(1, inputDim);

    quadraticApprox.dfdxx.emplace_back(matrix_t::Zero(stateDim, stateDim));
    quadraticApprox.dfdxx.diagonal().array() -= config_.hessianDiagonalShift;

    quadraticApprox.dfduu.emplace_back(matrix_t::Zero(inputDim, inputDim));

    quadraticApprox.dfdux.emplace_back(matrix_t::Zero(inputDim, stateDim));
  

    quadraticApprox.dfdu.block<1, 3>(0, 3 * contactPointIndex_) = dConeDF.transpose() * rotationWorldToTerrain_;

    const matrix3_t d2ConeD2u = rotationWorldToTerrain_.transpose() * d2ConeDF2 * rotationWorldToTerrain_;
    quadraticApprox.dfduu[0].block<3, 3>(3 * contactPointIndex_, 3 * contactPointIndex_) = d2ConeD2u;
    quadraticApprox.dfduu[0].diagonal().array() -= config_.hessianDiagonalShift_;

    return quadraticApprox;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t ForceFrictionConeConstraint::coneConstraint(const vector3_t &localForces) const 
  {
    const scalar_t tangentForce = sqrt(localForces.x() * localForces.x() + 
      localForces.y() * localForces.y() + config_.regularization_);

    const scalar_t frictionForce = config_.frictionCoefficient_
      * (localForces.z() + config_.gripperForce_);
      
    const scalar_t coneConstraint = frictionForce - tangentForce;
    return (vector_t(1) << coneConstraint).finished();
  }

} // namespace legged_locomotion_mpc