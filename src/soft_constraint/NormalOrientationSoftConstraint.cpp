#include <legged_locomotion_mpc/soft_constraint/NormalOrientationSoftConstraint.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  using namespace floating_base_model;

  NormalOrientationSoftConstraint::NormalOrientationSoftConstraint(
    const LeggedReferenceManager& referenceManager,
    NormalOrientationSoftConstraint::Settings settings, FloatingBaseModelInfo info, 
    const std::string& modelFolder, bool recompileLibraries, bool verbose):
      referenceManager_(referenceManager),info_(std::move(info)), 
      normalRelaxedBarrierPenaltyPtr_(new QuadraticPenalty(settings.barrierMu)) 
  {
    // Create CppAD function
    auto systemFlowMapFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) 
    {
      const Eigen::Matrix<ad_scalar_t, 3, 1> eulerAnglesAD = x;
      y = getNormalFromEulerAnglesCppAd(eulerAnglesAD);
    };
    
    normalFromEulerAnglesAdInterfacePtr_.reset(
      new CppAdInterface(systemFlowMapFunc, 3, "normal_from_euler", modelFolder));
    
    if(recompileLibraries) 
    {
      normalFromEulerAnglesAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    } 
    else 
    {
      normalFromEulerAnglesAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    }
  }

  NormalOrientationSoftConstraint* NormalOrientationSoftConstraint::clone() const
  {
    return new NormalOrientationSoftConstraint(*this);
  }

  scalar_t NormalOrientationSoftConstraint::getValue(scalar_t time, const vector_t& state,
    const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    scalar_t cost = 0.0;

    for(size_t i = 0; i < info_.numSixDofContacts; ++i)
    {
      const size_t endEffectorIndex = info_.numThreeDofContacts + i;

      if(!contactFlags[endEffectorIndex]) continue;

      const auto& eulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        endEffectorIndex);

      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(eulerAngles);

      const vector3_t& surfaceNormal = leggedPrecomputation.getSurfaceNormal(
        endEffectorIndex);

      const vector3_t endEffectorNormal = rotationMatrix.col(2);

      const scalar_t value = 1.0 - endEffectorNormal.dot(surfaceNormal);

      cost += normalRelaxedBarrierPenaltyPtr_->getValue(0.0, value);
    }
    return cost;
  }
        
  ScalarFunctionQuadraticApproximation NormalOrientationSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    const contact_flags_t contactFlags = referenceManager_.getContactFlags(time);

    ScalarFunctionQuadraticApproximation cost;

    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(info_.stateDim);
    cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);

    for(size_t i = 0; i < info_.numSixDofContacts; ++i)
    {
      const size_t endEffectorIndex = info_.numThreeDofContacts + i;

      if(!contactFlags[endEffectorIndex]) continue;

      const auto& eulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        endEffectorIndex);

      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(eulerAngles);

      const vector3_t& surfaceNormal = leggedPrecomputation.getSurfaceNormal(
        endEffectorIndex);

      const vector3_t endEffectorNormal = rotationMatrix.col(2);

      const scalar_t value = 1.0 - endEffectorNormal.dot(surfaceNormal);

      cost.f += normalRelaxedBarrierPenaltyPtr_->getValue(0.0, value);

      const scalar_t penaltyDerivative = normalRelaxedBarrierPenaltyPtr_->getDerivative(
        0.0, value);

      const matrix3_t normalGradient = matrix3_t::Map(
        normalFromEulerAnglesAdInterfacePtr_->getJacobian(eulerAngles).data());

      const matrix_t eulerAnglesDerivative = leggedPrecomputation.getEndEffectorOrientationDerivatives(
        endEffectorIndex).dfdx.block(0, 6, 3, info_.generalizedCoordinatesNum);

      const vector_t dValueDx = eulerAnglesDerivative.transpose() * normalGradient.transpose() * surfaceNormal;

      cost.dfdx.block(6, 0, info_.generalizedCoordinatesNum, 1).noalias() 
        += penaltyDerivative * dValueDx;

      const scalar_t secondPenaltyDerivative = 
        normalRelaxedBarrierPenaltyPtr_->getSecondDerivative(0.0, value);

      cost.dfdxx.block(6, 6, info_.generalizedCoordinatesNum, 
        info_.generalizedCoordinatesNum).noalias() 
          += secondPenaltyDerivative * dValueDx * dValueDx.transpose();
    }

    return cost;
  }

  NormalOrientationSoftConstraint::NormalOrientationSoftConstraint(
    const NormalOrientationSoftConstraint &other): referenceManager_(other.referenceManager_), 
      info_(other.info_), 
      normalRelaxedBarrierPenaltyPtr_(other.normalRelaxedBarrierPenaltyPtr_->clone()), 
      normalFromEulerAnglesAdInterfacePtr_(new CppAdInterface(
        *other.normalFromEulerAnglesAdInterfacePtr_)) {}

  ad_vector_t NormalOrientationSoftConstraint::getNormalFromEulerAnglesCppAd(
    const Eigen::Matrix<ad_scalar_t, 3, 1>& eulerAnglesAD)
  {
    return getRotationMatrixFromZyxEulerAngles(eulerAnglesAD).col(2);
  }

  NormalOrientationSoftConstraint::Settings loadNormalOrientationSoftConstraintSettings(
    const std::string& filename, const std::string& fieldName, bool verbose)
  {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    NormalOrientationSoftConstraint::Settings settings;

    if(verbose) 
    {
      std::cerr << "\n #### Legged Locomotion MPC Normal Orientation Soft Constraintt Settings:";
      std::cerr << "\n #### =============================================================================\n";
    }

    loadData::loadPtreeValue(pt, settings.barrierMu, 
        fieldName + ".mu", verbose);

    if(settings.barrierMu < 0.0)
    {
      throw std::invalid_argument("[NormalOrientationSoftConstraint]: Relaxed barrier penalty mu smaller than 0.0!");
    }

    if(verbose) 
    {
      std::cerr << " #### =============================================================================" <<
      std::endl;
    }

    return settings;
  }
} // namespace legged_locomotion_mpc