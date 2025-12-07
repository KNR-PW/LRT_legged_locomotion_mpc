#include <legged_locomotion_mpc/soft_constraint/EndEffectorPlacementSoftConstraint.hpp>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  EndEffectorPlacementSoftConstraint::EndEffectorPlacementSoftConstraint(
    floating_base_model::FloatingBaseModelInfo info,
    const LeggedReferenceManager& referenceManager,
    vector_t endEffectorRadiuses,
    RelaxedBarrierPenalty::Config settings):
      StateCost(),
      referenceManager_(referenceManager),
      endEffectorNum_(info.numThreeDofContacts + info.numSixDofContacts),
      info_(std::move(info)),
      endEffectorRadiuses_(std::move(endEffectorRadiuses)), 
      placementRelaxedBarrierPenaltyPtr_(new RelaxedBarrierPenalty(settings)) 
  {
    assert(endEffectorRadiuses_.size() == endEffectorNum_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  EndEffectorPlacementSoftConstraint* EndEffectorPlacementSoftConstraint::clone() const
  {
    return new EndEffectorPlacementSoftConstraint(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  scalar_t EndEffectorPlacementSoftConstraint::getValue(scalar_t time, 
    const vector_t& state, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);
    const auto& constraints = referenceManager_.getEndEffectorConstraintMatrixes(time);

    scalar_t cost = 0.0;

    for(size_t i = 0; i < endEffectorNum_; ++i)
    {
      if(!contactFlags[i]) continue;
      const auto& position = leggedPrecomputation.getEndEffectorPosition(i);
      const auto& constraint = constraints[i];
      vector_t constraintVector = constraint.A * position + constraint.b;
      constraintVector.array() -= endEffectorRadiuses_[i];
      
      cost += constraintVector.unaryExpr([&](scalar_t hi) 
      {
        return placementRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum();
    }
    return cost;
  }

  ScalarFunctionQuadraticApproximation EndEffectorPlacementSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories, 
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);
    const auto& constraints = referenceManager_.getEndEffectorConstraintMatrixes(time);

    ScalarFunctionQuadraticApproximation cost;
    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(info_.stateDim);
    cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);


    for(size_t i = 0; i < endEffectorNum_; ++i)
    {
      if(!contactFlags[i]) continue;
      const auto& position = leggedPrecomputation.getEndEffectorPosition(i);
      const auto& constraint = constraints[i];
      vector_t constraintVector = constraint.A * position + constraint.b;
      constraintVector.array() -= endEffectorRadiuses_[i];
      
      cost.f += constraintVector.unaryExpr([&](scalar_t hi) 
      {
        return placementRelaxedBarrierPenaltyPtr_->getValue(0.0, hi);
      }).sum();

      const auto& positionDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(i);

      const vector_t penaltyDerivative = constraintVector.unaryExpr([&](scalar_t hi) 
      {
        return placementRelaxedBarrierPenaltyPtr_->getDerivative(0.0, hi);
      });

      const matrix_t constraintDerivative = constraint.A * positionDerivative.dfdx;

      cost.dfdx += (constraintDerivative).transpose() * penaltyDerivative;

      const vector_t penaltySecondDerivative = constraintVector.unaryExpr([&](scalar_t hi) 
      {
        return placementRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      });

      cost.dfdxx += (constraintDerivative).transpose() * penaltySecondDerivative * constraintDerivative;
    }
    return cost;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  EndEffectorPlacementSoftConstraint::EndEffectorPlacementSoftConstraint(const EndEffectorPlacementSoftConstraint &rhs):
    StateCost(),
    referenceManager_(rhs.referenceManager_),
    endEffectorNum_(rhs.endEffectorNum_),
    info_(rhs.info_),
    endEffectorRadiuses_(rhs.endEffectorRadiuses_), 
    placementRelaxedBarrierPenaltyPtr_(rhs.placementRelaxedBarrierPenaltyPtr_->clone()) {}
} // namespace legged_locomotion_mpc