#include "legged_locomotion_mpc/constraint/Zero3DofVelocityConstraint.hpp"


namespace legged_locomotion_mpc
{
  using namespace ocs2;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Zero3DofVelocityConstraint::Zero3DofVelocityConstraint(
    const SwitchedModelReferenceManager &referenceManager,
    const PinocchioEndEffectorKinematicsCppAd &endEffectorKinematics,
    size_t contactPointIndex): 
      StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(new PinocchioEndEffectorKinematicsCppAd(endEffectorKinematics)),
      contactPointIndex_(contactPointIndex) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool Zero3DofVelocityConstraint::isActive(scalar_t time) const
  {
    return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_]; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t Zero3DofVelocityConstraint::getValue(scalar_t time, 
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    return vector_t(endEffectorKinematicsPtr_->getVelocity(state, input).front());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation Zero3DofVelocityConstraint::getLinearApproximation(
    scalar_t time,
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    return endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Zero3DofVelocityConstraint::Zero3DofVelocityConstraint(
    const Zero3DofVelocityConstraint &rhs):
    StateInputConstraint(rhs),
    referenceManagerPtr_(rhs.referenceManagerPtr_),
    endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
    contactPointIndex_(rhs.contactPointIndex_) {}
    
} // namespace legged_locomotion_mpc