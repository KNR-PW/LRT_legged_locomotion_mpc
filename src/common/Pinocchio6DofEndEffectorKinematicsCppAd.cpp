#include <legged_locomotion_mpc/common/Pinocchio6DofEndEffectorKinematicsCppAd.hpp>

namespace {

void defaultUpdatePinocchioInterface(const ocs2::ad_vector_t&, ocs2::PinocchioInterfaceTpl<ocs2::ad_scalar_t>&) {}

}  // unnamed namespace

namespace legged_locomotion_mpc 
{

  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Pinocchio6DofEndEffectorKinematicsCppAd::Pinocchio6DofEndEffectorKinematicsCppAd(const PinocchioInterface& pinocchioInterface,
    const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    std::vector<std::string> endEffectorIds, size_t stateDim,
    size_t inputDim, const std::string& modelName,
    const std::string& modelFolder, bool recompileLibraries,
    bool verbose)

    :Pinocchio6DofEndEffectorKinematicsCppAd(pinocchioInterface,
      mapping, std::move(endEffectorIds), stateDim, inputDim,
      &defaultUpdatePinocchioInterface, modelName, modelFolder,
      recompileLibraries, verbose) 
  {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Pinocchio6DofEndEffectorKinematicsCppAd::Pinocchio6DofEndEffectorKinematicsCppAd(
    const PinocchioInterface& pinocchioInterface,
    const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    std::vector<std::string> endEffectorIds, size_t stateDim,
    size_t inputDim, update_pinocchio_interface_callback updateCallback,
    const std::string& modelName, const std::string& modelFolder,
    bool recompileLibraries, bool verbose)
    :endEffectorIds_(std::move(endEffectorIds)) 
  {
    for (const auto& bodyName : endEffectorIds_) {
      endEffectorFrameIds_.push_back(pinocchioInterface.getModel().getFrameId(bodyName));
    }

    // initialize CppAD interface
    auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();

    // set pinocchioInterface to mapping
    std::unique_ptr<PinocchioStateInputMapping<ad_scalar_t>> mappingPtr(mapping.clone());
    mappingPtr->setPinocchioInterface(pinocchioInterfaceCppAd);

    // position function
    auto positionFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
      updateCallback(x, pinocchioInterfaceCppAd);
      y = getPositionCppAd(pinocchioInterfaceCppAd, *mappingPtr, x);
    };
    positionCppAdInterfacePtr_.reset(new CppAdInterface(positionFunc, stateDim, modelName + "_position", modelFolder));

    // orientation function
    auto orientationFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
      updateCallback(x, pinocchioInterfaceCppAd);
      y = getOrientationCppAd(pinocchioInterfaceCppAd, *mappingPtr, x);
    };
    orientationCppAdInterfacePtr_.reset(new CppAdInterface(orientationFunc, stateDim, modelName + "_orientation", modelFolder));

    // linear velocity function
    auto linearVelocityFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
      const ad_vector_t state = x.head(stateDim);
      const ad_vector_t input = x.tail(inputDim);
      updateCallback(state, pinocchioInterfaceCppAd);
      y = getLinearVelocityCppAd(pinocchioInterfaceCppAd, *mappingPtr, state, input);
    };
    linearVelocityCppAdInterfacePtr_.reset(new CppAdInterface(linearVelocityFunc, stateDim + inputDim, modelName + "_linear_velocity", modelFolder));

    // angular velocity function
    auto angularVelocityFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) {
      const ad_vector_t state = x.head(stateDim);
      const ad_vector_t input = x.tail(inputDim);
      updateCallback(state, pinocchioInterfaceCppAd);
      y = getAngularVelocityCppAd(pinocchioInterfaceCppAd, *mappingPtr, state, input);
    };
    angularVelocityCppAdInterfacePtr_.reset(new CppAdInterface(angularVelocityFunc, stateDim + inputDim, modelName + "_angular_velocity", modelFolder));

    if (recompileLibraries) {
      positionCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
      orientationCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
      linearVelocityCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
      angularVelocityCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    } else {
      positionCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
      orientationCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
      linearVelocityCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
      angularVelocityCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Pinocchio6DofEndEffectorKinematicsCppAd::Pinocchio6DofEndEffectorKinematicsCppAd(const Pinocchio6DofEndEffectorKinematicsCppAd& rhs)
    :positionCppAdInterfacePtr_(new CppAdInterface(*rhs.positionCppAdInterfacePtr_)),
    linearVelocityCppAdInterfacePtr_(new CppAdInterface(*rhs.linearVelocityCppAdInterfacePtr_)),
    angularVelocityCppAdInterfacePtr_(new CppAdInterface(*rhs.angularVelocityCppAdInterfacePtr_)),
    orientationCppAdInterfacePtr_(new CppAdInterface(*rhs.orientationCppAdInterfacePtr_)),
    endEffectorIds_(rhs.endEffectorIds_),
    endEffectorFrameIds_(rhs.endEffectorFrameIds_) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Pinocchio6DofEndEffectorKinematicsCppAd* Pinocchio6DofEndEffectorKinematicsCppAd::clone() const 
  {
    return new Pinocchio6DofEndEffectorKinematicsCppAd(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const std::vector<std::string>& Pinocchio6DofEndEffectorKinematicsCppAd::getIds() const 
  {
    return endEffectorIds_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ad_vector_t Pinocchio6DofEndEffectorKinematicsCppAd::getPositionCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    const ad_vector_t& state) 
  {
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    ad_vector_t positions(3 * endEffectorFrameIds_.size());
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) {
      const size_t frameId = endEffectorFrameIds_[i];
      positions.segment<3>(3 * i) = data.oMf[frameId].translation();
    }
    return positions;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto Pinocchio6DofEndEffectorKinematicsCppAd::getPosition(const vector_t& state) const -> std::vector<vector3_t> 
  {
    const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);

    std::vector<vector3_t> positions;
    for (int i = 0; i < endEffectorIds_.size(); i++) {
      positions.emplace_back(positionValues.segment<3>(3 * i));
    }
    return positions;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> Pinocchio6DofEndEffectorKinematicsCppAd::getPositionLinearApproximation(
      const vector_t& state) const 
  {
    const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);
    const matrix_t positionJacobian = positionCppAdInterfacePtr_->getJacobian(state);

    std::vector<VectorFunctionLinearApproximation> positions;
    for (int i = 0; i < endEffectorIds_.size(); i++) {
      VectorFunctionLinearApproximation pos;
      pos.f = positionValues.segment<3>(3 * i);
      pos.dfdx = positionJacobian.block(3 * i, 0, 3, state.rows());
      positions.emplace_back(std::move(pos));
    }
    return positions;
  }


  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ad_vector_t Pinocchio6DofEndEffectorKinematicsCppAd::getOrientationCppAd(PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    const ad_vector_t& state) 
  {
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    ad_vector_t orientations(3 * endEffectorFrameIds_.size());
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
    {
      const size_t frameId = endEffectorFrameIds_[i];
      orientations.segment<3>(3 * i) = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(data.oMf[frameId].rotation());
    }
    return orientations;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto Pinocchio6DofEndEffectorKinematicsCppAd::getOrientation(const vector_t& state) const -> std::vector<vector3_t> 
  {
    const vector_t orientationValues = orientationCppAdInterfacePtr_->getFunctionValue(state);
    std::vector<vector3_t> orientations;
    for (int i = 0; i < endEffectorIds_.size(); i++) 
    {
      orientations.emplace_back(orientationValues.segment<3>(3 * i));
    }
    return orientations;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> Pinocchio6DofEndEffectorKinematicsCppAd::getOrientationLinearApproximation(
    const vector_t& state) const 
  {
    const vector_t orientationValues = orientationCppAdInterfacePtr_->getFunctionValue(state);
    const matrix_t orientationJacobian = orientationCppAdInterfacePtr_->getJacobian(state);

    std::vector<VectorFunctionLinearApproximation> orientations;
    for (int i = 0; i < endEffectorIds_.size(); i++) 
    {
      VectorFunctionLinearApproximation rot;
      rot.f = orientationValues.segment<3>(3 * i);
      rot.dfdx = orientationJacobian.block(3 * i, 0, 3, state.rows());
      orientations.emplace_back(std::move(rot));
    }
    return orientations;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ad_vector_t Pinocchio6DofEndEffectorKinematicsCppAd::getLinearVelocityCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    const ad_vector_t& state, const ad_vector_t& input) 
  {
    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);
    const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

    pinocchio::forwardKinematics(model, data, q, v);

    ad_vector_t velocities(3 * endEffectorFrameIds_.size());
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
    {
      const size_t frameId = endEffectorFrameIds_[i];
      velocities.segment<3>(3 * i) = pinocchio::getFrameVelocity(model, data, frameId, rf).linear();
    }
    return velocities;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto Pinocchio6DofEndEffectorKinematicsCppAd::getLinearVelocity(
    const vector_t& state, const vector_t& input) const -> std::vector<vector3_t> 
  {
    vector_t stateInput(state.rows() + input.rows());
    stateInput << state, input;
    const vector_t velocityValues = linearVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);

    std::vector<vector3_t> velocities;
    for (int i = 0; i < endEffectorIds_.size(); i++) 
    {
      velocities.emplace_back(velocityValues.segment<3>(3 * i));
    }
    return velocities;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> Pinocchio6DofEndEffectorKinematicsCppAd::getLinearVelocityLinearApproximation(
      const vector_t& state, const vector_t& input) const 
  {
    vector_t stateInput(state.rows() + input.rows());
    stateInput << state, input;
    const vector_t velocityValues = linearVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);
    const matrix_t velocityJacobian = linearVelocityCppAdInterfacePtr_->getJacobian(stateInput);

    std::vector<VectorFunctionLinearApproximation> velocities;
    for (int i = 0; i < endEffectorIds_.size(); i++) 
    {
      VectorFunctionLinearApproximation vel;
      vel.f = velocityValues.segment<3>(3 * i);
      vel.dfdx = velocityJacobian.block(3 * i, 0, 3, state.rows());
      vel.dfdu = velocityJacobian.block(3 * i, state.rows(), 3, input.rows());
      velocities.emplace_back(std::move(vel));
    }
    return velocities;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ad_vector_t Pinocchio6DofEndEffectorKinematicsCppAd::getAngularVelocityCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const PinocchioStateInputMapping<ad_scalar_t>& mapping,
    const ad_vector_t& state, const ad_vector_t& input) 
  {
    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD;
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);
    const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

    pinocchio::forwardKinematics(model, data, q, v);

    ad_vector_t velocities(3 * endEffectorFrameIds_.size());
    for (int i = 0; i < endEffectorFrameIds_.size(); i++) 
    {
      const size_t frameId = endEffectorFrameIds_[i];
      velocities.segment<3>(3 * i) = pinocchio::getFrameVelocity(model, data, frameId, rf).angular();
    }
    return velocities;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto Pinocchio6DofEndEffectorKinematicsCppAd::getAngularVelocity(
    const vector_t& state, const vector_t& input) const -> std::vector<vector3_t> 
  {
    vector_t stateInput(state.rows() + input.rows());
    stateInput << state, input;
    const vector_t velocityValues = angularVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);
    
    std::vector<vector3_t> velocities;
    for (int i = 0; i < endEffectorIds_.size(); i++) 
    {
      velocities.emplace_back(velocityValues.segment<3>(3 * i));
    }
    return velocities;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> Pinocchio6DofEndEffectorKinematicsCppAd::getAngularVelocityLinearApproximation(
    const vector_t& state, const vector_t& input) const 
  {
    vector_t stateInput(state.rows() + input.rows());
    stateInput << state, input;
    const vector_t velocityValues = angularVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);
    const matrix_t velocityJacobian = angularVelocityCppAdInterfacePtr_->getJacobian(stateInput);
    
    std::vector<VectorFunctionLinearApproximation> velocities;
    for (int i = 0; i < endEffectorIds_.size(); i++) 
    {
      VectorFunctionLinearApproximation vel;
      vel.f = velocityValues.segment<3>(3 * i);
      vel.dfdx = velocityJacobian.block(3 * i, 0, 3, state.rows());
      vel.dfdu = velocityJacobian.block(3 * i, state.rows(), 3, input.rows());
      velocities.emplace_back(std::move(vel));
    }
    return velocities;
  }

}  // namespace legged_locomotion_mpc