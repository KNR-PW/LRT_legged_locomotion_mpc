#include <legged_locomotion_mpc/constraint/ZeroForceConstraint.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>

using namespace floating_base_model;

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model; 
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ZeroForceConstraint::ZeroForceConstraint(const LeggedReferenceManager& referenceManager,
    FloatingBaseModelInfo info,
    size_t endEffectorIndex): 
      StateInputConstraint(ConstraintOrder::Linear),
      referenceManager_(referenceManager),
      endEffectorIndex_(endEffectorIndex),
      info_(std::move(info)) { }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool ZeroForceConstraint::isActive(scalar_t time) const 
  {
    return !referenceManager_.getContactFlags(time)[endEffectorIndex_];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ZeroForceConstraint* ZeroForceConstraint::clone() const
  { 
    return new ZeroForceConstraint(*this); 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t ZeroForceConstraint::getNumConstraints(scalar_t time) const
  { 
    return 3; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const 
  {
    return access_helper_functions::getContactForces(input, endEffectorIndex_, info_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time,
    const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const 
  {
    const size_t stateDim = info_.stateDim;
    const size_t inputDim = info_.inputDim;

    VectorFunctionLinearApproximation linearApproximation;
    
    linearApproximation.f = getValue(time, state, input, preComp);
    linearApproximation.dfdx = matrix_t::Zero(3, stateDim);
    linearApproximation.dfdu = matrix_t::Zero(3, inputDim);
    linearApproximation.dfdu.middleCols<3>(3 * endEffectorIndex_).diagonal() = vector_t::Ones(3);
    
    return linearApproximation;
  }

} // namespace legged_locomotion_mpc