#include <legged_locomotion_mpc/constraint/JointLimitsConstraint.hpp>


namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointLimitsConstraint::JointLimitsConstraint(
    const floating_base_model::FloatingBaseModelInfo& info,
    const vector_t& jointPositionUpperLimits,
    const vector_t& jointPositionLowerLimits,
    const vector_t& jointVelocityLimits):
      StateInputConstraint(ConstraintOrder::Linear),
      jointPositionLowerLimits_(jointPositionLowerLimits), 
      jointPositionUpperLimits_(jointPositionUpperLimits),
      jointVelocityLimits_(jointVelocityLimits),
      info_(info)
  {
    numConstraints_ = 4 * info_.actuatedDofNum;

    size_t jointVelocitiesStartIndex = 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;

    cachedStateGradient_ = matrix_t(2 * info_.actuatedDofNum, info_.stateDim);

    cachedStateGradient_.block(0, 12,
      info_.actuatedDofNum, info_.actuatedDofNum).diagonal() = -vector_t::Ones(info_.actuatedDofNum);

    cachedStateGradient_.block(info_.actuatedDofNum, 12,
      info_.actuatedDofNum, info_.actuatedDofNum).diagonal() = vector_t::Ones(info_.actuatedDofNum);

    cachedInputGradient_ = matrix_t(2 * info_.actuatedDofNum, info_.inputDim);

    cachedInputGradient_.block(0, jointVelocitiesStartIndex,
      info_.actuatedDofNum, info_.actuatedDofNum).diagonal() = -vector_t::Ones(info_.actuatedDofNum);

    cachedInputGradient_.block(info_.actuatedDofNum, jointVelocitiesStartIndex,
      info_.actuatedDofNum, info_.actuatedDofNum).diagonal() = vector_t::Ones(info_.actuatedDofNum);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointLimitsConstraint* JointLimitsConstraint::clone() const
  {
    return JointLimitsConstraint(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool JointLimitsConstraint::isActive(scalar_t time) const
  {
    return true;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t JointLimitsConstraint::getNumConstraints(scalar_t time) const
  {
    return 4 * info_.actuatedDofNum;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t JointLimitsConstraint::getValue(scalar_t time, const vector_t &state,
    const vector_t &input, const PreComputation &preComp) const
  {
    const vector_t jointPositions = access_helper_functions::getJointAngles(state, info_);
    const vector_t jointVelocities = access_helper_functions::getJointVelocities(input, info_);
    const vector_t upperBoundJointPositionOffset = jointPositionUpperLimits_ - jointPositions;
    const vector_t lowerBoundJointPositionOffset = jointPositions - jointPositionLowerLimits_;
    const vector_t upperBoundJointVelocitiesOffset = jointVelocityLimits_ - jointVelocities;
    const vector_t lowerBoundJointVelocitiesOffset = jointVelocityLimits_ + jointVelocities;

    vector_t constraintValue;
    constraintValue << upperBoundJointPositionOffset, lowerBoundJointPositionOffset, 
      upperBoundJointVelocitiesOffset, lowerBoundJointVelocitiesOffset;

    return constraintValue;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation JointLimitsConstraint::getLinearApproximation(scalar_t time,
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    VectorFunctionLinearApproximation constraintLinearApprox;

    constraintLinearApprox.f = getValue(time, state, input);
    constraintLinearApprox.dfdx = cachedStateGradient_;
    constraintLinearApprox.dfdu = cachedInputGradient_;

    return constraintLinearApprox;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  JointLimitsConstraint::JointLimitsConstraint(const JointLimitsConstraint &rhs):
    StateInputConstraint(rhs), info_(rhs.info_),
    jointPositionLowerLimits_(rhs.jointPositionLowerLimits_),
    jointPositionUpperLimits_(rhs.jointPositionUpperLimits_),
    jointVelocityLimits_(rhs.jointVelocityLimits_),
    numConstraints_(rhs.numConstraints_),
    linearApproximation_(rhs.linearApproximation_){}


} // namespace legged_locomotion_mpc