#include "legged_locomotion_mpc/constraint/Zero6DofVelocityConstraint.hpp"

namespace legged_locomotion_mpc
{
  using namespace ocs2;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Zero6DofVelocityConstraint::Zero6DofVelocityConstraint(
    const SwitchedModelReferenceManager &referenceManager,
    const Pinocchio6DofEndEffectorKinematicsCppAd &endEffectorKinematics,
    size_t contactFeetIndex): 
      StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      endEffectorKinematicsPtr_(new Pinocchio6DofEndEffectorKinematicsCppAd(endEffectorKinematics)),
      contactFeetIndex_(contactFeetIndex) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool Zero6DofVelocityConstraint::isActive(scalar_t time) const
  {
    return referenceManagerPtr_->getContactFlags(time)[contactFeetIndex_]; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t Zero6DofVelocityConstraint::getValue(scalar_t time, 
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    const auto linearVelocity = endEffectorKinematicsPtr_->getLinearVelocity(state, input).front();
    const auto angularVelocity = endEffectorKinematicsPtr_->getAngularVelocity(state, input).front();
    vector_t velocity(6);
    velocity << linearVelocity, angularVelocity;
    return velocity;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation Zero6DofVelocityConstraint::getLinearApproximation(
    scalar_t time,
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    VectorFunctionLinearApproximation velocityApprox;

    const auto linearVelocityApprox = endEffectorKinematicsPtr_->getLinearVelocityLinearApproximation(state, input).front();
    const auto angularVelocityApprox = endEffectorKinematicsPtr_->getAngularVelocityLinearApproximation(state, input).front();

    velocityApprox.f << linearVelocityApprox.f << angularVelocityApprox.f;
    velocityApprox.dfdx << linearVelocityApprox.dfdx << angularVelocityApprox.dfdx;
    velocityApprox.dfdu << linearVelocityApprox.dfdu << angularVelocityApprox.dfdu;

    return velocityApprox;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Zero6DofVelocityConstraint::Zero6DofVelocityConstraint(
    const Zero6DofVelocityConstraint &rhs):
    StateInputConstraint(rhs),
    referenceManagerPtr_(rhs.referenceManagerPtr_),
    endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
    contactFeetIndex_(rhs.contactFeetIndex_) {}
    
} // namespace legged_locomotion_mpc