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

#include <legged_locomotion_mpc/cost/TrajectoryTrackingCost.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/log.hpp>
#include <pinocchio/codegen/cppadcg.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <legged_locomotion_mpc/common/Utils.hpp>
#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  namespace cost
  {
    using namespace ocs2;
    using namespace floating_base_model;

    TrajectoryTrackingCost::TrajectoryTrackingCost(FloatingBaseModelInfo info,
      const LeggedReferenceManager& referenceManager,
      BaseWeights baseWeights, JointWeights jointWeights,
      EndEffectorWeights endEffectorWeights, const std::string& modelFolder,
      bool recompileLibraries, bool verbose):
      info_(std::move(info)), referenceManager_(referenceManager),
      baseWeights_(std::move(baseWeights)), jointWeights_(std::move(jointWeights)),
      endEffectorWeights_(std::move(endEffectorWeights)) 
    {
      assert(jointWeights_.positions.size() == info_.actuatedDofNum);
      assert(jointWeights_.velocities.size() == info_.actuatedDofNum);
      
      const size_t endEffectorNum = info_.numThreeDofContacts + info_.numSixDofContacts;
      
      assert(endEffectorWeights_.positions.size() == endEffectorNum);
      assert(endEffectorWeights_.linearVelocities.size() == endEffectorNum);
      assert(endEffectorWeights_.forces.size() == endEffectorNum);

      auto systemFlowMapFunc = [&](const ad_vector_t& x, const ad_vector_t& p, 
        ad_vector_t& y) 
      {
        const Eigen::Matrix<ad_scalar_t, 3, 1> currentEulerAnglesAD = x;
        const Eigen::Matrix<ad_scalar_t, 3, 1> targetEulerAnglesAD = p;
        y = getLog3CppAd(currentEulerAnglesAD, targetEulerAnglesAD);
      };
    
      log3AdInterfacePtr_.reset(
          new CppAdInterface(systemFlowMapFunc, 3, 3, "log3", modelFolder));
    
      if (recompileLibraries) {
        log3AdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
      } else {
        log3AdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
      }
    }

    TrajectoryTrackingCost* TrajectoryTrackingCost::clone() const
    {
      return new TrajectoryTrackingCost(*this);
    }

    scalar_t TrajectoryTrackingCost::getValue(scalar_t time, const vector_t& state, 
      const vector_t& input, const TargetTrajectories& targetTrajectories, 
      const PreComputation& preComp) const
    {
      scalar_t cost = 0.0;

      const auto targetState = targetTrajectories.getDesiredState(time);
      const auto targetInput = targetTrajectories.getDesiredInput(time);

      const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

      // Base current data
      const auto baseCurrentPosition = floating_base_model::
        access_helper_functions::getBasePosition(state, info_);

      const vector3_t baseCurrentEulerAngles = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(state, info_);

      const auto baseCurrentLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(state, info_);

      const auto baseCurrentAngularVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(state, info_);

      // Base target data
      const auto baseTargetPosition = floating_base_model::
        access_helper_functions::getBasePosition(targetState, info_);

      const vector3_t baseTargetEulerAngles = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(targetState, info_);

      const auto baseTargetLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(targetState, info_);

      const auto baseTargetAngularVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(targetState, info_);
      
      // Base error
      const auto basePositionError = baseTargetPosition - baseCurrentPosition;

      const auto baseRotationError = log3AdInterfacePtr_->getFunctionValue(
        baseCurrentEulerAngles, baseTargetEulerAngles);

      const auto baseLinearVelocityError = baseTargetLinearVelocity 
        - baseCurrentLinearVelocity;

      const auto baseAngularVelocityError = baseTargetAngularVelocity 
        - baseCurrentAngularVelocity;
      
      // Add base weighted squared cost
      cost += basePositionError.dot(baseWeights_.position.asDiagonal() * basePositionError);
      cost += baseRotationError.dot(baseWeights_.rotation.asDiagonal() * baseRotationError);
      cost += baseLinearVelocityError.dot(baseWeights_.linearVelocity.asDiagonal() 
        * baseLinearVelocityError);
      cost += baseAngularVelocityError.dot(baseWeights_.angularVelocity.asDiagonal() 
        * baseAngularVelocityError);

      const size_t endEffectorNum = info_.numThreeDofContacts + info_.numSixDofContacts;

      const auto& endEffectorTargets = referenceManager_.getEndEffectorTrajectoryPoint(time);
      
      // In order to get true contact forces, not interpolated ones (when contact is changed for example)
      const contact_flags_t targetContactFlags = referenceManager_.getContactFlags(time);
      const vector_t forcesInInput = utils::weightCompensatingInput(info_, targetContactFlags);

      for(size_t i = 0; i < endEffectorNum; ++i)
      {
        // End effectors current data
        const vector3_t& endEffectorCurrentPositon = leggedPrecomputation.getEndEffectorPosition(i);
        const vector3_t& endEffectorCurrentVelocity = leggedPrecomputation.getEndEffectorLinearVelocity(i);
        const auto endEffectorCurrentForce = floating_base_model::
          access_helper_functions::getContactForces(input, i, info_);

        // End effectors target data
        const vector3_t& endEffectorTargetPositon = endEffectorTargets.positions[i];
        const vector3_t& endEffectorTargetVelocity = endEffectorTargets.velocities[i];
        const auto endEffectorTargetForce = floating_base_model::
          access_helper_functions::getContactForces(forcesInInput, i, info_);
        
        // End effectors error
        const vector3_t& endEffectorPositionError = endEffectorTargetPositon - endEffectorCurrentPositon;
        const vector3_t& endEffectorVelocityError = endEffectorTargetVelocity - endEffectorCurrentVelocity;
        const auto endEffectorForceError = endEffectorTargetForce - endEffectorCurrentForce;
        
        // Add end effectors weighted squared cost
        cost += endEffectorPositionError.dot(endEffectorWeights_.positions[i].asDiagonal() 
          * endEffectorPositionError);
        cost += endEffectorVelocityError.dot(endEffectorWeights_.velocities[i].asDiagonal() 
          * endEffectorVelocityError);
        cost += endEffectorForceError.dot(endEffectorWeights_.forces[i].asDiagonal() 
          * endEffectorForceError);
      }

      // Joint current data
      const auto jointCurrentPositions = floating_base_model::
        access_helper_functions::getJointPositions(state, info_);
      
      const auto jointCurrentVelocities = floating_base_model::
        access_helper_functions::getJointVelocities(input, info_);

      // Joint target data
      const auto jointTargetPositions = floating_base_model::
        access_helper_functions::getJointPositions(targetState, info_);
      
      const auto jointTargetVelocities = floating_base_model::
        access_helper_functions::getJointVelocities(targetInput, info_);

      // Joint error
      const auto jointPositionError = jointTargetPositions - jointCurrentPositions;
      const auto jointVelocityError = jointTargetVelocities - jointCurrentVelocities;

      // Add joint weighted squared cost
      cost += jointPositionError.dot(jointWeights_.positions.asDiagonal() 
        * jointPositionError);
      cost += jointVelocityError.dot(jointWeights_.positions.asDiagonal() 
        * jointVelocityError);
      
      // 1 / 2 of sum
      return 0.5 * cost;
    }

    ScalarFunctionQuadraticApproximation TrajectoryTrackingCost::getQuadraticApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const TargetTrajectories& targetTrajectories,
      const PreComputation& preComp) const
    {
      ScalarFunctionQuadraticApproximation cost;

      cost.f = 0.0;
      cost.dfdx = vector_t::Zero(info_.stateDim);
      cost.dfdu = vector_t::Zero(info_.inputDim);
      cost.dfdxx = vector_t::Zero(info_.stateDim, info_.stateDim);
      cost.dfduu = vector_t::Zero(info_.inputDim, info_.inputDim);
      cost.dfdux = vector_t::Zero(info_.inputDim, info_.stateDim);
      
      const auto targetState = targetTrajectories.getDesiredState(time);
      const auto targetInput = targetTrajectories.getDesiredInput(time);

      const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

      // Base current data
      const auto baseCurrentPosition = floating_base_model::
        access_helper_functions::getBasePosition(state, info_);

      const vector3_t baseCurrentEulerAngles = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(state, info_);

      const auto baseCurrentLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(state, info_);

      const auto baseCurrentAngularVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(state, info_);

      // Base target data
      const auto baseTargetPosition = floating_base_model::
        access_helper_functions::getBasePosition(targetState, info_);

      const vector3_t baseTargetEulerAngles = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(targetState, info_);

      const auto baseTargetLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(targetState, info_);

      const auto baseTargetAngularVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(targetState, info_);
      
      // Base error
      const auto basePositionError = baseTargetPosition - baseCurrentPosition;

      const auto baseRotationError = log3AdInterfacePtr_->getFunctionValue(
        baseCurrentEulerAngles, baseTargetEulerAngles);

      const auto baseLinearVelocityError = baseTargetLinearVelocity 
        - baseCurrentLinearVelocity;

      const auto baseAngularVelocityError = baseTargetAngularVelocity 
        - baseCurrentAngularVelocity;
      
      // Add base weighted squared cost and its derivatives
      const vector3_t weightedBasePositonError = baseWeights_.position.asDiagonal() * basePositionError;
      cost.f += basePositionError.dot(weightedBasePositonError);
      cost.dfdx.block<3, 1>(6, 0) += -weightedBasePositonError;
      cost.dfdxx.block<3, 3>(6, 6) += baseWeights_.position.asDiagonal();
      
      const vector3_t weightedBaseRotationError = baseWeights_.rotation.asDiagonal() 
        * baseRotationError;
      cost.f += baseRotationError.dot(weightedBaseRotationError);
      const matrix_t log3Derivative = log3AdInterfacePtr_->getJacobian(
        baseCurrentEulerAngles, baseTargetEulerAngles);
      cost.dfdx.block<3, 1>(9, 0) += log3Derivative.transpose() * weightedBaseRotationError;
      cost.dfdx.block<3, 3>(9, 9) += log3Derivative.transpose() 
        * baseWeights_.rotation.asDiagonal() * log3Derivative;
      
      const vector3_t weightedBaseLinearVelocityError = 
        baseWeights_.linearVelocity.asDiagonal() * baseLinearVelocityError;
      cost.f += baseLinearVelocityError.dot(weightedBaseLinearVelocityError);
      cost.dfdx.block<3, 1>(0, 0) += -weightedBaseLinearVelocityError;
      cost.dfdxx.block<3, 3>(0, 0) += baseWeights_.linearVelocity.asDiagonal();
      
      const vector3_t weightedBaseAngularVelocityError = 
        baseWeights_.angularVelocity.asDiagonal() * baseAngularVelocityError;
      cost.f += baseAngularVelocityError.dot(weightedBaseAngularVelocityError);
      cost.dfdx.block<3, 1>(3, 0) += -weightedBaseAngularVelocityError;
      cost.dfdxx.block<3, 3>(3, 3) += baseWeights_.angularVelocity.asDiagonal();

      const size_t endEffectorNum = info_.numThreeDofContacts + info_.numSixDofContacts;

      const auto& endEffectorTargets = referenceManager_.getEndEffectorTrajectoryPoint(time);
      
      // In order to get true contact forces, not interpolated ones (when contact is changed for example)
      const contact_flags_t targetContactFlags = referenceManager_.getContactFlags(time);
      const vector_t forcesInInput = utils::weightCompensatingInput(info_, targetContactFlags);

      const size_t forceIndexOffset = 3 * info_.numThreeDofContacts 
          + 6 * info_.numSixDofContacts;

      for(size_t i = 0; i < endEffectorNum; ++i)
      {
        // End effectors current data
        const vector3_t& endEffectorCurrentPositon = leggedPrecomputation.getEndEffectorPosition(i);
        const auto& endEffectorCurrentPositonDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(i);

        const vector3_t& endEffectorCurrentVelocity = leggedPrecomputation.getEndEffectorLinearVelocity(i);
        const auto& endEffectorCurrentVelocityDerivative = leggedPrecomputation.getEndEffectorLinearVelocityDerivatives(i);

        const auto endEffectorCurrentForce = floating_base_model::
          access_helper_functions::getContactForces(input, i, info_);

        // End effectors target data
        const vector3_t& endEffectorTargetPositon = endEffectorTargets.positions[i];
        const vector3_t& endEffectorTargetVelocity = endEffectorTargets.velocities[i];
        const auto endEffectorTargetForce = floating_base_model::
          access_helper_functions::getContactForces(forcesInInput, i, info_);
        
        // End effectors error
        const vector3_t& endEffectorPositionError = endEffectorTargetPositon - endEffectorCurrentPositon;
        const vector3_t& endEffectorVelocityError = endEffectorTargetVelocity - endEffectorCurrentVelocity;
        const auto endEffectorForceError = endEffectorTargetForce - endEffectorCurrentForce;

        const vector3_t weightedEndEffectorPositionError = 
          endEffectorWeights_.positions[i].asDiagonal() * endEffectorPositionError;

        const vector3_t weightedEndEffectorVelocityError = 
          endEffectorWeights_.velocities[i].asDiagonal() * endEffectorVelocityError;

        const vector3_t weightedEndEffectorForceError = 
          endEffectorWeights_.forces[i].asDiagonal() * endEffectorForceError;

        const auto positionDerivativeBlock = endEffectorCurrentPositonDerivative.dfdx.block(0, 
          6, 3, info_.stateDim - 6);

        const auto velocityDerivativeBlock = endEffectorCurrentVelocityDerivative.dfdu.block(0, 
          forceIndexOffset, 3, info_.inputDim - forceIndexOffset);

        // Add end effectors weighted squared cost and its derivatives
        cost.f += endEffectorPositionError.dot(weightedEndEffectorPositionError);
        cost.dfdx.block(6, 0, info_.stateDim - 6, 1) += 
          -positionDerivativeBlock.transpose() * weightedEndEffectorPositionError;
        cost.dfdxx.block(6, 6, info_.stateDim - 6, info_.stateDim - 6) += 
          positionDerivativeBlock.transpose() * endEffectorWeights_.positions[i].asDiagonal() 
          * positionDerivativeBlock;

        cost.f += endEffectorVelocityError.dot(weightedEndEffectorVelocityError);
        cost.dfdx += -endEffectorCurrentVelocityDerivative.dfdx.transpose() 
          * weightedEndEffectorVelocityError;
        cost.dfdu.block(forceIndexOffset, 0, info_.inputDim - forceIndexOffset, 1) +=
          -velocityDerivativeBlock.transpose() * weightedEndEffectorVelocityError;
        cost.dfdxx += endEffectorCurrentVelocityDerivative.dfdx.transpose() * 
          endEffectorWeights_.velocities[i].asDiagonal() * endEffectorCurrentVelocityDerivative.dfdx;
        cost.dfduu.block(forceIndexOffset, forceIndexOffset, 
          info_.inputDim - forceIndexOffset, info_.inputDim - forceIndexOffset) += 
            velocityDerivativeBlock.transpose() 
            * endEffectorWeights_.velocities[i].asDiagonal() * velocityDerivativeBlock;

        cost.f += endEffectorForceError.dot(weightedEndEffectorForceError);
        if(i < info_.numThreeDofContacts)
        {
          cost.dfdu.block<3, 1>(3 * i, 0) += -weightedEndEffectorForceError;
          cost.dfduu.block<3, 3>(3 * i, 3 * i) += endEffectorWeights_.forces[i].asDiagonal();
        }
        else
        {
          const size_t startIndex = 6 * i - 3 * info_.numThreeDofContacts;
          cost.dfdu.block<3, 1>(startIndex, 0) += -weightedEndEffectorForceError;
          cost.dfduu.block<3, 3>(startIndex, startIndex) += 
            endEffectorWeights_.forces[i].asDiagonal();
        }
      }

      // Joint current data
      const auto jointCurrentPositions = floating_base_model::
        access_helper_functions::getJointPositions(state, info_);
      
      const auto jointCurrentVelocities = floating_base_model::
        access_helper_functions::getJointVelocities(input, info_);

      // Joint target data
      const auto jointTargetPositions = floating_base_model::
        access_helper_functions::getJointPositions(targetState, info_);
      
      const auto jointTargetVelocities = floating_base_model::
        access_helper_functions::getJointVelocities(targetInput, info_);

      // Joint error
      const auto jointPositionError = jointTargetPositions - jointCurrentPositions;
      const auto jointVelocityError = jointTargetVelocities - jointCurrentVelocities;

      const auto weightedJointPositionError = jointWeights_.positions.asDiagonal() 
        * jointPositionError;

      const auto weightedJointVelocityError = jointWeights_.velocities.asDiagonal() 
        * jointVelocityError;

      // Add joint weighted squared cost and its derivatives
      cost.f += jointPositionError.dot(weightedJointPositionError);
      cost.dfdx.block(12, 0, info_.actuatedDofNum, 1) += -weightedJointPositionError;
      cost.dfdxx.block(12, 12, info_.actuatedDofNum, info_.actuatedDofNum) += 
        jointWeights_.positions.asDiagonal();

      cost.f += jointVelocityError.dot(weightedJointVelocityError);
      cost.dfdu.block(forceIndexOffset, 0, info_.actuatedDofNum, 1) += 
        -weightedJointVelocityError;
      cost.dfduu.block(forceIndexOffset, forceIndexOffset, info_.actuatedDofNum, 
        info_.actuatedDofNum) += jointWeights_.velocities.asDiagonal();
      
      // 1 / 2 of sum
      cost.f *= 0.5;

      return cost;
    }

    TrajectoryTrackingCost::TrajectoryTrackingCost(const TrajectoryTrackingCost& rhs):
      info_(rhs.info_), referenceManager_(rhs.referenceManager_), 
      baseWeights_(rhs.baseWeights_), jointWeights_(rhs.jointWeights_),
      endEffectorWeights_(rhs.endEffectorWeights_), 
      log3AdInterfacePtr_(new CppAdInterface(*rhs.log3AdInterfacePtr_)) {}

    ad_vector_t TrajectoryTrackingCost::getLog3CppAd(
      const Eigen::Matrix<ad_scalar_t, 3, 1>& currentEulerAngles, 
      const Eigen::Matrix<ad_scalar_t, 3, 1>& targetEulerAngles)
    {
      const auto targetRotation = getRotationMatrixFromZyxEulerAngles(targetEulerAngles);
      const auto currentRotation = getRotationMatrixFromZyxEulerAngles(targetEulerAngles);
      return pinocchio::log3(targetRotation.transpose() * currentRotation);
    }
  } // namespace cost
} // namespace legged_locomotion_mpc