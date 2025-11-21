#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <floating_base_model/QuaterionEulerTransforms.hpp>

namespace legged_locomotion_mpc 
{

  using namespace ocs2;
  using namespace floating_base_model;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioForwardEndEffectorKinematicsCppAd::PinocchioForwardEndEffectorKinematicsCppAd(
    const PinocchioInterface& pinocchioInterface,
    const FloatingBaseModelInfo info, const std::string& modelName,
    const std::string& modelFolder, bool recompileLibraries, bool verbose): 
      numEndEffectors_(info.numThreeDofContacts + info.numSixDofContacts), 
      info_(std::move(info))
  {
    // initialize CppAD interface
    auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  
    // mapping
    FloatingBaseModelPinocchioMappingCppAd mappingCppAd(info.toCppAd());
     mappingCppAd.setPinocchioInterface(pinocchioInterfaceCppAd);

    // position function
    auto positionFunc = [&, this](const ad_vector_t& x, ad_vector_t& y) 
    {
      y = getPositionCppAd(pinocchioInterfaceCppAd, mappingCppAd, x);
    };
    positionCppAdInterfacePtr_.reset(new CppAdInterface(positionFunc, info_.stateDim, modelName + "_position", modelFolder));

    // orientation function
    auto orientationFunc = [&](const ad_vector_t& x, ad_vector_t& y) 
    {
      y = getOrientationCppAd(pinocchioInterfaceCppAd, mappingCppAd, x);
    };
    orientationCppAdInterfacePtr_.reset(new CppAdInterface(orientationFunc, info_.stateDim, modelName + "_orientation", modelFolder));

    // linear velocity function
    auto linearVelocityFunc = [&](const ad_vector_t& x, ad_vector_t& y) 
    {
      const ad_vector_t state = x.head(info_.stateDim);
      const ad_vector_t input = x.tail(info_.inputDim);
      y = getLinearVelocityCppAd(pinocchioInterfaceCppAd, mappingCppAd, state, input);
    };
    linearVelocityCppAdInterfacePtr_.reset(new CppAdInterface(linearVelocityFunc, info_.stateDim + info_.inputDim, modelName + "_linear_velocity", modelFolder));

    // angular velocity function
    auto angularVelocityFunc = [&](const ad_vector_t& x, ad_vector_t& y) 
    {
      const ad_vector_t state = x.head(info_.stateDim);
      const ad_vector_t input = x.tail(info_.inputDim);
      y = getAngularVelocityCppAd(pinocchioInterfaceCppAd, mappingCppAd, state, input);
    };
    angularVelocityCppAdInterfacePtr_.reset(new CppAdInterface(angularVelocityFunc, info_.stateDim + info_.inputDim, modelName + "_angular_velocity", modelFolder));

    if (recompileLibraries) 
    {
      positionCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
      orientationCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
      linearVelocityCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
      angularVelocityCppAdInterfacePtr_->createModels(CppAdInterface::ApproximationOrder::First, verbose);
    } 
    else 
    {
      positionCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
      orientationCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
      linearVelocityCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
      angularVelocityCppAdInterfacePtr_->loadModelsIfAvailable(CppAdInterface::ApproximationOrder::First, verbose);
    }
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioForwardEndEffectorKinematicsCppAd::PinocchioForwardEndEffectorKinematicsCppAd(const PinocchioForwardEndEffectorKinematicsCppAd& rhs)
    :positionCppAdInterfacePtr_(new CppAdInterface(*rhs.positionCppAdInterfacePtr_)),
    orientationCppAdInterfacePtr_(new CppAdInterface(*rhs.orientationCppAdInterfacePtr_)),
    linearVelocityCppAdInterfacePtr_(new CppAdInterface(*rhs.linearVelocityCppAdInterfacePtr_)),
    angularVelocityCppAdInterfacePtr_(new CppAdInterface(*rhs.angularVelocityCppAdInterfacePtr_)),
    numEndEffectors_(rhs.numEndEffectors_),
    info_(rhs.info_) {}

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  PinocchioForwardEndEffectorKinematicsCppAd* PinocchioForwardEndEffectorKinematicsCppAd::clone() const 
  {
    return new PinocchioForwardEndEffectorKinematicsCppAd(*this);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const FloatingBaseModelInfo& PinocchioForwardEndEffectorKinematicsCppAd::getInfo() const
  {
    return info_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ad_vector_t PinocchioForwardEndEffectorKinematicsCppAd::getPositionCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const FloatingBaseModelPinocchioMappingCppAd& mapping,
    const ad_vector_t& state) 
  {
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    ad_vector_t positions(3 * numEndEffectors_);
    for (size_t i = 0; i < numEndEffectors_; i++) {
      const size_t frameId = info_.endEffectorFrameIndices[i];
      positions.segment<3>(3 * i) = data.oMf[frameId].translation();
    }
    return positions;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto PinocchioForwardEndEffectorKinematicsCppAd::getPosition(const vector_t& state) const -> std::vector<vector3_t> 
  {
    const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);

    std::vector<vector3_t> positions;
    for (size_t i = 0; i < numEndEffectors_; i++) {
      positions.emplace_back(positionValues.segment<3>(3 * i));
    }
    return positions;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> PinocchioForwardEndEffectorKinematicsCppAd::getPositionLinearApproximation(
      const vector_t& state) const 
  {
    const vector_t positionValues = positionCppAdInterfacePtr_->getFunctionValue(state);
    const matrix_t positionJacobian = positionCppAdInterfacePtr_->getJacobian(state);

    std::vector<VectorFunctionLinearApproximation> positions;
    for (size_t i = 0; i < numEndEffectors_; i++) {
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
  ad_vector_t PinocchioForwardEndEffectorKinematicsCppAd::getOrientationCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const FloatingBaseModelPinocchioMappingCppAd& mapping,
    const ad_vector_t& state) 
  {
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);

    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    ad_vector_t orientations(3 * info_.numSixDofContacts);
    for (size_t i = info_.numThreeDofContacts; i < numEndEffectors_; i++) 
    {
      const size_t frameId = info_.endEffectorFrameIndices[i];
      orientations.segment<3>(3 * i) = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(data.oMf[frameId].rotation());
    }
    return orientations;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto PinocchioForwardEndEffectorKinematicsCppAd::getOrientation(const vector_t& state) const -> std::vector<vector3_t> 
  {
    const vector_t orientationValues = orientationCppAdInterfacePtr_->getFunctionValue(state);
    std::vector<vector3_t> orientations;
    for (size_t i = 0; i < info_.numSixDofContacts; i++) 
    {
      orientations.emplace_back(orientationValues.segment<3>(3 * i));
    }
    return orientations;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> PinocchioForwardEndEffectorKinematicsCppAd::getOrientationLinearApproximation(
    const vector_t& state) const 
  {
    const vector_t orientationValues = orientationCppAdInterfacePtr_->getFunctionValue(state);
    const matrix_t orientationJacobian = orientationCppAdInterfacePtr_->getJacobian(state);

    std::vector<VectorFunctionLinearApproximation> orientations;
    for (size_t i = 0; i < info_.numSixDofContacts; i++) 
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
  ad_vector_t PinocchioForwardEndEffectorKinematicsCppAd::getLinearVelocityCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const FloatingBaseModelPinocchioMappingCppAd& mapping,
    const ad_vector_t& state, const ad_vector_t& input) 
  {
    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);
    const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

    pinocchio::forwardKinematics(model, data, q, v);

    ad_vector_t velocities(3 * numEndEffectors_);
    for (size_t i = 0; i < numEndEffectors_; i++) 
    {
      const size_t frameId = info_.endEffectorFrameIndices[i];
      velocities.segment<3>(3 * i) = pinocchio::getFrameVelocity(model, data, frameId, rf).linear();
    }
    return velocities;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto PinocchioForwardEndEffectorKinematicsCppAd::getLinearVelocity(
    const vector_t& state, const vector_t& input) const -> std::vector<vector3_t> 
  {
    vector_t stateInput(state.rows() + input.rows());
    stateInput << state, input;
    const vector_t velocityValues = linearVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);

    std::vector<vector3_t> velocities;
    for (size_t i = 0; i < numEndEffectors_; i++) 
    {
      velocities.emplace_back(velocityValues.segment<3>(3 * i));
    }
    return velocities;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> PinocchioForwardEndEffectorKinematicsCppAd::getLinearVelocityLinearApproximation(
      const vector_t& state, const vector_t& input) const 
  {
    vector_t stateInput(state.rows() + input.rows());
    stateInput << state, input;
    const vector_t velocityValues = linearVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);
    const matrix_t velocityJacobian = linearVelocityCppAdInterfacePtr_->getJacobian(stateInput);

    std::vector<VectorFunctionLinearApproximation> velocities;
    for (size_t i = 0; i < numEndEffectors_; i++) 
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
  ad_vector_t PinocchioForwardEndEffectorKinematicsCppAd::getAngularVelocityCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const FloatingBaseModelPinocchioMappingCppAd& mapping,
    const ad_vector_t& state, const ad_vector_t& input) 
  {
    const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::WORLD;
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();
    const ad_vector_t q = mapping.getPinocchioJointPosition(state);
    const ad_vector_t v = mapping.getPinocchioJointVelocity(state, input);

    pinocchio::forwardKinematics(model, data, q, v);

    ad_vector_t velocities(3 * info_.numSixDofContacts);
    for (size_t i = info_.numThreeDofContacts; i < numEndEffectors_; i++) 
    {
      const size_t frameId = info_.endEffectorFrameIndices[i];
      velocities.segment<3>(3 * i) = pinocchio::getFrameVelocity(model, data, frameId, rf).angular();
    }
    return velocities;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  auto PinocchioForwardEndEffectorKinematicsCppAd::getAngularVelocity(
    const vector_t& state, const vector_t& input) const -> std::vector<vector3_t> 
  {
    vector_t stateInput(state.rows() + input.rows());
    stateInput << state, input;
    const vector_t velocityValues = angularVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);
    
    std::vector<vector3_t> velocities;
    for (size_t i = 0; i < info_.numSixDofContacts; i++) 
    {
      velocities.emplace_back(velocityValues.segment<3>(3 * i));
    }
    return velocities;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::vector<VectorFunctionLinearApproximation> PinocchioForwardEndEffectorKinematicsCppAd::getAngularVelocityLinearApproximation(
    const vector_t& state, const vector_t& input) const 
  {
    vector_t stateInput(state.rows() + input.rows());
    stateInput << state, input;
    const vector_t velocityValues = angularVelocityCppAdInterfacePtr_->getFunctionValue(stateInput);
    const matrix_t velocityJacobian = angularVelocityCppAdInterfacePtr_->getJacobian(stateInput);
    
    std::vector<VectorFunctionLinearApproximation> velocities;
    for (size_t i = 0; i < info_.numSixDofContacts; i++) 
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