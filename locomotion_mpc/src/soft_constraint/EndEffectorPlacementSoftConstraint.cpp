#include <legged_locomotion_mpc/soft_constraint/EndEffectorPlacementSoftConstraint.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

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
    EndEffectorPlacementSoftConstraint::Settings settings):
      StateCost(),
      referenceManager_(referenceManager),
      endEffectorNum_(info.numThreeDofContacts + info.numSixDofContacts),
      info_(std::move(info)),
      endEffectorRadiuses_(std::move(settings.endEffectorSafetyRadiuses)), 
      placementRelaxedBarrierPenaltyPtr_(new RelaxedBarrierPenalty(settings.barrierSettings)) 
  {
    if(endEffectorRadiuses_.size() != endEffectorNum_)
    {
      throw std::invalid_argument("[EndEffectorPlacementSoftConstraint]: endEffectorRadiuses " 
        "have wrong size!");
    }
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
      const auto& constraint = constraints[i];
      if(!contactFlags[i] || !constraint.isActive()) continue;
      const auto& position = leggedPrecomputation.getEndEffectorPosition(i);
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
      const auto& constraint = constraints[i];
      if(!contactFlags[i] || !constraint.isActive()) continue;
      const auto& position = leggedPrecomputation.getEndEffectorPosition(i);
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

      cost.dfdx.noalias() += constraintDerivative.transpose() * penaltyDerivative;

      const vector_t penaltySecondDerivative = constraintVector.unaryExpr([&](scalar_t hi) 
      {
        return placementRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, hi);
      });

      cost.dfdxx.noalias() += constraintDerivative.transpose() 
        * penaltySecondDerivative.asDiagonal() * constraintDerivative;
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
    
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  using Settings = EndEffectorPlacementSoftConstraint::Settings;
  Settings loadEndEffectorPlacementSoftConstraintSettings(const std::string& filename, 
    const ModelSettings modelSettings, const std::string& fieldName, bool verbose)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    Settings settings;

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC End Effector Placement Soft Constraint Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    std::vector<std::string> endEffectorNames(modelSettings.endEffectorThreeDofNames);
    endEffectorNames.insert(endEffectorNames.end(), 
      modelSettings.endEffectorSixDofNames.begin(), modelSettings.endEffectorSixDofNames.end());

    size_t index = 0;
    settings.endEffectorSafetyRadiuses.resize(endEffectorNames.size());
    for(const auto& endEffectorName: endEffectorNames)
    {
      loadData::loadPtreeValue(pt, settings.endEffectorSafetyRadiuses[index], 
        fieldName + "." + endEffectorName + ".safetyRadius", verbose);
      if(settings.endEffectorSafetyRadiuses[index] < 0.0)
      {
        std::string message = "[EndEffectorPlacementSoftConstraint]: " + endEffectorName + " safety radius smaller than 0.0!";
        throw std::invalid_argument(message);
      }
      index++;
    }

    loadData::loadPtreeValue(pt, settings.barrierSettings.mu, 
        fieldName + ".mu", verbose);

    if(settings.barrierSettings.mu < 0.0)
    {
      throw std::invalid_argument("[EndEffectorPlacementSoftConstraint]: Relaxed barrier penalty mu smaller than 0.0!");
    }

    loadData::loadPtreeValue(pt, settings.barrierSettings.delta, 
        fieldName + ".delta", verbose);

    if(settings.barrierSettings.delta < 0.0)
    {
      throw std::invalid_argument("[EndEffectorPlacementSoftConstraint]: Relaxed barrier penalty delta smaller than 0.0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return settings;
  }
} // namespace legged_locomotion_mpc