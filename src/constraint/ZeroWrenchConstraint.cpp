#include "legged_locomotion_mpc/constraint/ZeroWrenchConstraint.hpp"

using namespace floating_base_model;

namespace legged_locomotion_mpc
{

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ZeroWrenchConstraint::ZeroWrenchConstraint(const SwitchedModelReferenceManager &referenceManager,
    size_t contactPointIndex,
    FloatingBaseModelInfo& info,
    size_t stateDim,
    size_t inputDim): 
      StateInputConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      contactPointIndex_(contactPointIndex),
      info_(&info) 
  {
    const FloatingBaseModelInfo& info = *info;
    const size_t startRow = 3 * info.numThreeDofContacts + 6 * (contactPointIndex_ - info.numThreeDofContacts);
    approx_.dfdx = ocs2::matrix_t::Zero(6, stateDim);
    approx_.dfdu = ocs2::matrix_t::Zero(6, inputDim);
    approx_.dfdu.middleCols<6>(startRow).diagonal() = ocs2::vector_t::Ones(6);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool ZeroWrenchConstraint::isActive(ocs2::scalar_t time) const 
  {
    return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t ZeroWrenchConstraint::getValue(ocs2::scalar_t time,
    const ocs2::vector_t &state,
    const ocs2::vector_t &input,
    const ocs2::PreComputation &preComp) const 
  {
    return access_helper_functions::getContactWrenches(input, contactPointIndex_, *info_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation ZeroWrenchConstraint::getLinearApproximation(ocs2::scalar_t time,
    const ocs2::vector_t &state,
    const ocs2::vector_t &input,
    const ocs2::PreComputation &preComp) const 
  {
    approx_.f = getValue(time, state, input, preComp);
    return approx_;
  }

} // namespace legged_locomotion_mpc