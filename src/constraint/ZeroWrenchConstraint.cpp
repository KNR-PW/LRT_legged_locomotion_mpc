#include <legged_locomotion_mpc/constraint/ZeroWrenchConstraint.hpp>

using namespace floating_base_model;

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ZeroWrenchConstraint::ZeroWrenchConstraint(const LeggedReferenceManager& referenceManager,
    FloatingBaseModelInfo info,
    size_t endEffectorIndex): 
      StateInputConstraint(ConstraintOrder::Linear),
      referenceManager_(referenceManager),
      endEffectorIndex_(endEffectorIndex),
      info_(std::move(info)) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool ZeroWrenchConstraint::isActive(scalar_t time) const 
  {
    return !referenceManager_.getContactFlags(time)[endEffectorIndex_];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ZeroWrenchConstraint* ZeroWrenchConstraint::clone() const
  { 
    return new ZeroWrenchConstraint(*this); 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t ZeroWrenchConstraint::getNumConstraints(scalar_t time) const
  { 
    return 6; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t ZeroWrenchConstraint::getValue(scalar_t time,
    const vector_t& state,
    const vector_t& input,
    const PreComputation& preComp) const 
  {
    return access_helper_functions::getContactWrenches(input, endEffectorIndex_, info_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation ZeroWrenchConstraint::getLinearApproximation(scalar_t time,
    const vector_t& state,
    const vector_t& input,
    const PreComputation& preComp) const 
  {
    VectorFunctionLinearApproximation linearApproximation;

    const size_t startRow = 3 * info_.numThreeDofContacts + 6 * (endEffectorIndex_ - info_.numThreeDofContacts);

    linearApproximation.f = getValue(time, state, input, preComp);
    linearApproximation.dfdx = ocs2::matrix_t::Zero(6, info_.stateDim);
    linearApproximation.dfdu = ocs2::matrix_t::Zero(6, info_.inputDim);
    linearApproximation.dfdu.middleCols<6>(startRow).diagonal() = ocs2::vector_t::Ones(6);

    return linearApproximation;
  }

} // namespace legged_locomotion_mpc