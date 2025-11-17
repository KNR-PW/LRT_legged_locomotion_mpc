#include <legged_locomotion_mpc/soft_constraint/JointLimitsSoftConstraint.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointLimitsSoftConstraint::JointLimitsSoftConstraint(
    floating_base_model::FloatingBaseModelInfo info,
    vector_t jointPositionUpperLimits,
    vector_t jointPositionLowerLimits,
    vector_t jointVelocityLimits,
    RelaxedBarrierPenalty::Config settings):
      StateInputCost(),
      jointPositionLowerLimits_(std::move(jointPositionLowerLimits)), 
      jointPositionUpperLimits_(std::move(jointPositionUpperLimits)),
      jointVelocityLimits_(std::move(jointVelocityLimits)),
      info_(std::move(info)),
      jointRelaxedBarrierPenaltyPtr_(new RelaxedBarrierPenalty(settings)) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointLimitsSoftConstraint* JointLimitsSoftConstraint::clone() const
  {
    return new JointLimitsSoftConstraint(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t JointLimitsSoftConstraint::getValue(scalar_t time, const vector_t& state,
    const vector_t& input, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto jointPositions = access_helper_functions::getJointPositions(state, info_);
    const auto jointVelocities = access_helper_functions::getJointVelocities(input, info_);
    const vector_t upperBoundJointPositionOffset = jointPositionUpperLimits_ - jointPositions;
    const vector_t lowerBoundJointPositionOffset = jointPositions - jointPositionLowerLimits_;
    const vector_t upperBoundJointVelocitiesOffset = jointVelocityLimits_ - jointVelocities;
    const vector_t lowerBoundJointVelocitiesOffset = jointVelocityLimits_ + jointVelocities;

    const scalar_t cost = upperBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + upperBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum();

    return cost;
  }

  ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const vector_t& input,
    const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const
  {
    const auto jointPositions = access_helper_functions::getJointPositions(state, info_);
    const auto jointVelocities = access_helper_functions::getJointVelocities(input, info_);
    const vector_t upperBoundJointPositionOffset = jointPositionUpperLimits_ - jointPositions;
    const vector_t lowerBoundJointPositionOffset = jointPositions - jointPositionLowerLimits_;
    const vector_t upperBoundJointVelocitiesOffset = jointVelocityLimits_ - jointVelocities;
    const vector_t lowerBoundJointVelocitiesOffset = jointVelocityLimits_ + jointVelocities;

    ScalarFunctionQuadraticApproximation cost;

    cost.f = upperBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + upperBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum() + lowerBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum();

    const vector_t penaltyDerivativesState = 
      -upperBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      }) + lowerBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      });
    
    const vector_t penaltyDerivativesInput = 
      - upperBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      }) + lowerBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      });
    
    // Should be diagonal matrix, but will be stored as vector for faster computation
    const vector_t penaltySecondDerivativesState = 
      upperBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      }) + lowerBoundJointPositionOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      });
    
     const vector_t penaltySecondDerivativesInput = 
      upperBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      }) + lowerBoundJointVelocitiesOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      });

    cost.dfdx = vector_t::Zero(info_.stateDim);
    cost.dfdu = vector_t::Zero(info_.inputDim);

    const size_t forceSize = 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;

    cost.dfdx.block(12, 0, info_.actuatedDofNum, 1) = penaltyDerivativesState;
    cost.dfdu.block(forceSize, 0, info_.actuatedDofNum, 1) = penaltyDerivativesInput;

    cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);
    cost.dfduu = matrix_t::Zero(info_.inputDim, info_.inputDim);
    cost.dfdux = matrix_t::Zero(info_.inputDim, info_.stateDim);

    cost.dfdxx.block(12, 12, info_.actuatedDofNum, info_.actuatedDofNum) = 
      penaltySecondDerivativesState.asDiagonal();

    cost.dfduu.block(forceSize, forceSize, info_.actuatedDofNum, info_.actuatedDofNum) = 
      penaltyDerivativesInput.asDiagonal();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointLimitsSoftConstraint::JointLimitsSoftConstraint(const JointLimitsSoftConstraint &rhs):
    StateInputCost(), info_(rhs.info_),
    jointPositionLowerLimits_(rhs.jointPositionLowerLimits_),
    jointPositionUpperLimits_(rhs.jointPositionUpperLimits_),
    jointVelocityLimits_(rhs.jointVelocityLimits_),
    jointRelaxedBarrierPenaltyPtr_(rhs.jointRelaxedBarrierPenaltyPtr_->clone())
    {}


} // namespace legged_locomotion_mpc