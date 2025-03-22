#include "legged_locomotion_mpc/constraint/ZeroForceConstraint.hpp"

using namespace floating_base_model;

namespace legged_locomotion_mpc
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ZeroForceConstraint::ZeroForceConstraint(const SwitchedModelReferenceManager &referenceManager,
    size_t contactPointIndex,
    FloatingBaseModelInfo& info,
    size_t stateDim,
    size_t inputDim): 
      StateInputConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      contactPointIndex_(contactPointIndex),
      info_(&info) 
  {
    approx_.dfdx = ocs2::matrix_t::Zero(3, stateDim);
    approx_.dfdu = ocs2::matrix_t::Zero(3, inputDim);
    approx_.dfdu.middleCols<3>(3 * contactPointIndex_).diagonal() = ocs2::vector_t::Ones(3);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool ZeroForceConstraint::isActive(ocs2::scalar_t time) const 
  {
    return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t ZeroForceConstraint::getValue(ocs2::scalar_t time,
    const ocs2::vector_t &state,
    const ocs2::vector_t &input,
    const ocs2::PreComputation &preComp) const 
  {
    return access_helper_functions::getContactForces(input, contactPointIndex_, *info_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(ocs2::scalar_t time,
    const ocs2::vector_t &state,
    const ocs2::vector_t &input,
    const ocs2::PreComputation &preComp) const 
  {
    approx_.f = getValue(time, state, input, preComp);
    return approx_;
  }

} // namespace legged_locomotion_mpc