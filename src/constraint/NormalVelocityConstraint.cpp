#include "legged_locomotion_mpc/constraint/NormalVelocityConstraint.hpp"

namespace legged_locomotion_mpc
{
  using namespace ocs2;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  NormalVelocityConstraint::NormalVelocityConstraint(
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
  bool NormalVelocityConstraint::isActive(scalar_t time) const
  {
    return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_]; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t NormalVelocityConstraint::getValue(scalar_t time, 
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    return surfaceNormalInWorld_.dot(endEffectorKinematicsPtr_->getVelocity(state, input).front());
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation NormalVelocityConstraint::getLinearApproximation(
    scalar_t time,
    const vector_t &state, const vector_t &input,
    const PreComputation &preComp) const
  {
    VectorFunctionLinearApproximation constraint;

    const auto velocityLinearApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
    
    constraint.f = surfaceNormalInWorld_.dot(velocityLinearApprox);
    constraint.dfdx = surfaceNormalInWorld_.transpose() * velocityLinearApprox.dfdx;
    constraint.dfdu = surfaceNormalInWorld_.transpose() * velocityLinearApprox.dfdu;

    return constraint;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void NormalVelocityConstraint::setSurfaceNormalInWorld(const vector3_t &surfaceNormalInWorld)
  {
    throw std::runtime_error("[NormalVelocityConstraint] setSurfaceNormalInWorld() is not implemented!");
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  NormalVelocityConstraint::NormalVelocityConstraint(
    const NormalVelocityConstraint &rhs):
    StateInputConstraint(rhs),
    referenceManagerPtr_(rhs.referenceManagerPtr_),
    endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
    contactPointIndex_(rhs.contactPointIndex_) {}
    
} // namespace legged_locomotion_mpc