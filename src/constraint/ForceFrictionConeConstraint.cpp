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

#include <legged_locomotion_mpc/constraint/ForceFrictionConeConstraint.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeConstraint::Config::Config(
    scalar_t frictionCoefficientParam,
    scalar_t regularizationParam,
    scalar_t hessianDiagonalShiftParam): 
      frictionCoefficient(frictionCoefficientParam),
      regularization(regularizationParam),
      hessianDiagonalShift(hessianDiagonalShiftParam) 
  {
    assert(frictionCoefficient > 0.0);
    assert(regularization > 0.0);
    assert(hessianDiagonalShift >= 0.0);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeConstraint::ForceFrictionConeConstraint(
    const LeggedReferenceManager &referenceManager, Config config,
    floating_base_model::FloatingBaseModelInfo info, size_t endEffectorIndex): 
      StateInputConstraint(ConstraintOrder::Quadratic),
      referenceManager_(referenceManager),
      config_(std::move(config)),
      endEffectorIndex_(endEffectorIndex),
      info_(std::move(info)) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool ForceFrictionConeConstraint::isActive(scalar_t time) const 
  {
    return referenceManager_.getContactFlags(time)[endEffectorIndex_];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ForceFrictionConeConstraint* ForceFrictionConeConstraint::clone() const
  { 
    return new ForceFrictionConeConstraint(*this); 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t ForceFrictionConeConstraint::getNumConstraints(scalar_t time) const
  {
    return 1;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t ForceFrictionConeConstraint::getValue(scalar_t time,
    const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const 
  {
    const auto forcesInWorldFrame = access_helper_functions::getContactForces(input, 
      endEffectorIndex_, info_);

    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const matrix3_t& rotationMatrixToTerrain = 
      leggedPrecomputation.getRotationWorldToTerrain(endEffectorIndex_);

    const vector3_t localForce = rotationMatrixToTerrain * forcesInWorldFrame;

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
    const auto forcesInWorldFrame = access_helper_functions::getContactForces(input, 
      endEffectorIndex_, info_);

    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const matrix3_t& rotationMatrixToTerrain = 
      leggedPrecomputation.getRotationWorldToTerrain(endEffectorIndex_);

    const vector3_t localForce = rotationMatrixToTerrain * forcesInWorldFrame;

    const scalar_t tangentForceInverse = 1 / (sqrt(localForce.x() * localForce.x() + 
      localForce.y() * localForce.y() + config_.regularization));

    vector3_t dConeDF;
    dConeDF(0) = -localForce.x() * tangentForceInverse;
    dConeDF(1) = -localForce.y() * tangentForceInverse;
    dConeDF(2) = config_.frictionCoefficient;


    VectorFunctionLinearApproximation linearApprox;

    const size_t stateDim = info_.stateDim;
    const size_t inputDim = info_.inputDim;
    linearApprox.dfdx = matrix_t::Zero(1, stateDim);
    linearApprox.dfdu = matrix_t::Zero(1, inputDim);

    linearApprox.f = coneConstraint(localForce);
    linearApprox.dfdu.block<1, 3>(0, 3 * endEffectorIndex_) = dConeDF.transpose() * rotationMatrixToTerrain;
    
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
    const auto forcesInWorldFrame = access_helper_functions::getContactForces(input, 
      endEffectorIndex_, info_);

    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const matrix3_t& rotationMatrixToTerrain = 
      leggedPrecomputation.getRotationWorldToTerrain(endEffectorIndex_);

    const vector3_t localForce = rotationMatrixToTerrain * forcesInWorldFrame;

    const scalar_t forceXSquared = localForce.x() * localForce.x();
    const scalar_t forceYSquared = localForce.y() * localForce.y();
    const scalar_t forceXforceY  = localForce.x() * localForce.y();

    const scalar_t tangentForceSquaredInverse = 1 / (forceXSquared + 
      forceYSquared + config_.regularization);

    const scalar_t tangentForceInverse = sqrt(tangentForceSquaredInverse);
    
    const scalar_t tangentForceInverse32 = tangentForceSquaredInverse * tangentForceInverse;

    vector3_t dConeDF;
    dConeDF(0) = -localForce.x() * tangentForceInverse;
    dConeDF(1) = -localForce.y() * tangentForceInverse;
    dConeDF(2) = config_.frictionCoefficient;

    
    matrix3_t d2ConeDF2 = matrix3_t::Zero();

    d2ConeDF2(0, 0) = -(forceYSquared + config_.regularization) * tangentForceInverse32;
    d2ConeDF2(1, 1) = -(forceXSquared + config_.regularization) * tangentForceInverse32;
    d2ConeDF2(0, 1) =  forceXforceY * tangentForceInverse32;
    d2ConeDF2(1, 0) =  d2ConeDF2(0, 1);


    VectorFunctionQuadraticApproximation quadraticApprox;

    quadraticApprox.f = coneConstraint(localForce);
    
    const size_t stateDim = info_.stateDim;
    const size_t inputDim = info_.inputDim;

    quadraticApprox.dfdx = matrix_t::Zero(1, stateDim);
    quadraticApprox.dfdu = matrix_t::Zero(1, inputDim);

    quadraticApprox.dfdxx.emplace_back(matrix_t::Zero(stateDim, stateDim));
    quadraticApprox.dfdxx[0].diagonal().array() -= config_.hessianDiagonalShift;

    quadraticApprox.dfduu.emplace_back(matrix_t::Zero(inputDim, inputDim));

    quadraticApprox.dfdux.emplace_back(matrix_t::Zero(inputDim, stateDim));
  

    quadraticApprox.dfdu.block<1, 3>(0, 3 * endEffectorIndex_) = dConeDF.transpose() * 
      rotationMatrixToTerrain;

    const matrix3_t d2ConeD2u = rotationMatrixToTerrain.transpose() * d2ConeDF2 * 
      rotationMatrixToTerrain;

    quadraticApprox.dfduu[0].block<3, 3>(3 * endEffectorIndex_, 
      3 * endEffectorIndex_) = d2ConeD2u;

    quadraticApprox.dfduu[0].diagonal().array() -= config_.hessianDiagonalShift;

    return quadraticApprox;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t ForceFrictionConeConstraint::coneConstraint(const vector3_t &localForce) const 
  {
    const scalar_t tangentForce = sqrt(localForce.x() * localForce.x() + 
      localForce.y() * localForce.y() + config_.regularization);

    const scalar_t frictionForce = config_.frictionCoefficient * localForce.z();
      
    const scalar_t coneConstraint = frictionForce - tangentForce;
    return (vector_t(1) << coneConstraint).finished();
  }
} // namespace legged_locomotion_mpc