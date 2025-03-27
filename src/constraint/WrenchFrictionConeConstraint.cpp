

#include "legged_locomotion_mpc/constraint/WrenchFrictionConeConstraint.hpp"



namespace legged_locomotion_mpc
{
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  explicit WrenchFrictionConeConstraint::Config::Config(
    ocs2::scalar_t frictionCoefficientParam,
    ocs2::scalar_t footLengthX,
    ocs2::scalar_t footLengthY): 
      frictionCoefficient_(frictionCoefficientParam),
      footHalfLengthX_(footLengthX / 2),
      footHalfLengthY_(footLengthY / 2),
  {
    assert(frictionCoefficient_ > 0.0);
    assert(footHalfLengthX_ > 0.0);
    assert(footHalfLengthY_>= 0.0);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WrenchFrictionConeConstraint::WrenchFrictionConeConstraint(const SwitchedModelReferenceManager &referenceManager,
    Config config,
    size_t contactFeetIndex,
    FloatingBaseModelInfo& info): 
      StateInputConstraint(ocs2::ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      config_(config),
      contactFeetIndex_(contactFeetIndex),
      info_(&info) 
  {

    const size_t stateDim = info.stateDim.size();
    const size_t inputDim = info.inputDim.size();

    linearApproximation_.f = ocs2::matrix_t::Zero(16,1);
    linearApproximation_.dfdx = ocs2::matrix_t::Zero(16, stateDim);
    linearApproximation_.dfdu = ocs2::matrix_t::Zero(16, inputDim);

    coneConstraintMatrix_ = generateConeConstraintMatrix(config_);

  }

  // TODO: Dodaj jak będzie ogarnięty interface od Kuby!
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void WrenchFrictionConeConstraint::setSurfaceNormalInWorld(const vector3_t &surfaceNormalInWorld) 
  {
    rotationWorldToTerrain_.setIdentity();
    throw std::runtime_error("[WrenchFrictionConeConstraint] setSurfaceNormalInWorld() is not implemented!");
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool WrenchFrictionConeConstraint::isActive(ocs2::scalar_t time) const 
  {
    return referenceManagerPtr_->getContactFlags(time)[contactFeetIndex_];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::vector_t WrenchFrictionConeConstraint::getValue(scalar_t time,
    const ocs2::vector_t &state,
    const ocs2::vector_t &input,
    const ocs2::PreComputation &preComp) const 
  {
    const auto wrenchInWorldFrame = access_helper_functions::getContactWrenches(input, contactFeetIndex_, *info_);
    
    const vector6_t localWrench;
    localWrench << rotationWorldToTerrain_ * wrenchInWorldFrame.block<3, 1>(0, 0),
      rotationWorldToTerrain_ * wrenchInWorldFrame.block<3, 1>(3, 0);

    ocs2::vector_t coneConstraint = coneConstraintMatrix_ * localWrench;

    return coneConstraint;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ocs2::VectorFunctionLinearApproximation WrenchFrictionConeConstraint::getLinearApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t &state,
    const ocs2::vector_t &input,
    const ocs2::PreComputation &preComp) const 
  {
    const auto& info = *info_;

    const auto wrenchInWorldFrame = access_helper_functions::getContactWrenches(input, contactFeetIndex_, *info_);
    
    const vector6_t localWrench;
    localWrench << rotationWorldToTerrain_ * wrenchInWorldFrame.block<3, 1>(0, 0),
      rotationWorldToTerrain_ * wrenchInWorldFrame.block<3, 1>(3, 0);

    ocs2::vector_t coneConstraint = coneConstraintMatrix_ * localWrench;

    linearApproximation_.f = coneConstraint;

    const size_t startIndex = 3 * info.numThreeDofContacts + 6 * (contactFeetIndex_ - info.numThreeDofContacts);

    matrix6_t rotation6x6 = matrix6_t::Zero();
    rotation6x6.block<3, 3>(0, 0) = rotationWorldToTerrain_;
    rotation6x6.block<3, 3>(3, 3) = rotationWorldToTerrain_;

    linearApproximation_.dfdu.block<16, 6>(0, startIndex) = coneConstraintMatrix_ * rotation6x6;
    
    return linearApproximation_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::Matrix<ocs2::scalar_t, 16, 6> WrenchFrictionConeConstraint::generateConeConstraintMatrix(const Config& config)
  {
    Eigen::Matrix<ocs2::scalar_t, 16, 6> coneConstraintMatrix;

    // u * f_z + f_x >= 0
    coneConstraintMatrix(0, 0) = 1.0;
    coneConstraintMatrix(0, 2) = config.frictionCoefficient_;

    // u * f_z - f_x >= 0
    coneConstraintMatrix(1, 0) = -1.0;
    coneConstraintMatrix(1, 2) = config.frictionCoefficient_;

    // u * f_z + f_y >= 0
    coneConstraintMatrix(2, 1) = 1.0;
    coneConstraintMatrix(2, 2) = config.frictionCoefficient_;

    // u * f_z - f_y >= 0
    coneConstraintMatrix(3, 1) = -1.0;
    coneConstraintMatrix(3, 2) = config.frictionCoefficient_;

    // Y * f_z + tau_x >= 0
    coneConstraintMatrix(4, 2) = config.footHalfLengthY_;
    coneConstraintMatrix(4, 3) = config.frictionCoefficient_;

    // Y * f_z - tau_x >= 0
    coneConstraintMatrix(5, 2) = config.footHalfLengthY_;
    coneConstraintMatrix(5, 3) = -config.frictionCoefficient_;

    // X * f_z + tau_y >= 0
    coneConstraintMatrix(6, 2) = config.footHalfLengthX_;
    coneConstraintMatrix(7, 4) = config.frictionCoefficient_;

    // X * f_z - tau_y >= 0
    coneConstraintMatrix(7, 2) = config.footHalfLengthX_;
    coneConstraintMatrix(7, 4) = -config.frictionCoefficient_;

    // Y * f_x + X * f_y + u * (X + Y) * f_z - u * tau_x - u * tau_y + tau_z >= 0
    coneConstraintMatrix(8, 0) = config.footHalfLengthY_;
    coneConstraintMatrix(8, 1) = config.footHalfLengthX_;
    coneConstraintMatrix(8, 2) = (config.footHalfLengthY_ + config.foothHalfLengthX_) * config.frictionCoefficient_;
    coneConstraintMatrix(8, 3) = -config.frictionCoefficient_;
    coneConstraintMatrix(8, 4) = -config.frictionCoefficient_;
    coneConstraintMatrix(8, 5) = 1;

    // Y * f_x - X * f_y + u * (X + Y) * f_z - u * tau_x + u * tau_y + tau_z >= 0
    coneConstraintMatrix(9, 0) = config.footHalfLengthY_;
    coneConstraintMatrix(9, 1) = -config.footHalfLengthX_;
    coneConstraintMatrix(9, 2) = (config.footHalfLengthY_ + config.foothHalfLengthX_) * config.frictionCoefficient_;
    coneConstraintMatrix(9, 3) = -config.frictionCoefficient_;
    coneConstraintMatrix(9, 4) = config.frictionCoefficient_;
    coneConstraintMatrix(9, 5) = 1;

    // -Y * f_x + X * f_y + u * (X + Y) * f_z - u * tau_x - u * tau_y + tau_z >= 0
    coneConstraintMatrix(10, 0) = -config.footHalfLengthY_;
    coneConstraintMatrix(10, 1) = config.footHalfLengthX_;
    coneConstraintMatrix(10, 2) = (config.footHalfLengthY_ + config.foothHalfLengthX_) * config.frictionCoefficient_;
    coneConstraintMatrix(10, 3) = config.frictionCoefficient_;
    coneConstraintMatrix(10, 4) = -config.frictionCoefficient_;
    coneConstraintMatrix(10, 5) = 1;

    // -Y * f_x - X * f_y + u * (X + Y) * f_z + u * tau_x + u * tau_y + tau_z >= 0
    coneConstraintMatrix(11, 0) = -config.footHalfLengthY_;
    coneConstraintMatrix(11, 1) = -config.footHalfLengthX_;
    coneConstraintMatrix(11, 2) = (config.footHalfLengthY_ + config.foothHalfLengthX_) * config.frictionCoefficient_;
    coneConstraintMatrix(11, 3) = config.frictionCoefficient_;
    coneConstraintMatrix(11, 4) = config.frictionCoefficient_;
    coneConstraintMatrix(11, 5) = 1;

    // -Y * f_x - X * f_y + u * (X + Y) * f_z - u * tau_x - u * tau_y - tau_z >= 0
    coneConstraintMatrix(12, 0) = -config.footHalfLengthY_;
    coneConstraintMatrix(12, 1) = -config.footHalfLengthX_;
    coneConstraintMatrix(12, 2) = (config.footHalfLengthY_ + config.foothHalfLengthX_) * config.frictionCoefficient_;
    coneConstraintMatrix(12, 3) = -config.frictionCoefficient_;
    coneConstraintMatrix(12, 4) = -config.frictionCoefficient_;
    coneConstraintMatrix(12, 5) = -1;

    // -Y * f_x + X * f_y + u * (X + Y) * f_z - u * tau_x + u * tau_y - tau_z >= 0
    coneConstraintMatrix(13, 0) = -config.footHalfLengthY_;
    coneConstraintMatrix(13, 1) = config.footHalfLengthX_;
    coneConstraintMatrix(13, 2) = (config.footHalfLengthY_ + config.foothHalfLengthX_) * config.frictionCoefficient_;
    coneConstraintMatrix(13, 3) = -config.frictionCoefficient_;
    coneConstraintMatrix(13, 4) = config.frictionCoefficient_;
    coneConstraintMatrix(13, 5) = -1;

    // Y * f_x - X * f_y + u * (X + Y) * f_z + u * tau_x - u * tau_y - tau_z >= 0
    coneConstraintMatrix(14, 0) = config.footHalfLengthY_;
    coneConstraintMatrix(14, 1) = -config.footHalfLengthX_;
    coneConstraintMatrix(14, 2) = (config.footHalfLengthY_ + config.foothHalfLengthX_) * config.frictionCoefficient_;
    coneConstraintMatrix(14, 3) = config.frictionCoefficient_;
    coneConstraintMatrix(14, 4) = -config.frictionCoefficient_;
    coneConstraintMatrix(14, 5) = -1;

    // Y * f_x + X * f_y + u * (X + Y) * f_z + u * tau_x + u * tau_y - tau_z >= 0
    coneConstraintMatrix(15, 0) = config.footHalfLengthY_;
    coneConstraintMatrix(15, 1) = config.footHalfLengthX_;
    coneConstraintMatrix(15, 2) = (config.footHalfLengthY_ + config.foothHalfLengthX_) * config.frictionCoefficient_;
    coneConstraintMatrix(15, 3) = config.frictionCoefficient_;
    coneConstraintMatrix(15, 4) = config.frictionCoefficient_;
    coneConstraintMatrix(15, 5) = -1;

    return coneConstraintMatrix;
  }

} // namespace legged_locomotion_mpc