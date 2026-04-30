#include <gtest/gtest.h>

#include "../include/definitions.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;

const scalar_t tolerance = 1e-6;

const size_t NUM_TEST = 20;

TEST(PinocchioForwardEndEffectorKinematicsCppAdTest, getPosition)
{
  const std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  const std::vector<std::string> meldogFake3DofContactNames = {"RFF_link", "RRF_link"};
  const std::vector<std::string> meldogFake6DofContactNames = {"LFF_link", "LRF_link"};
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, 
    baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    meldogFake3DofContactNames, meldogFake6DofContactNames);

  FloatingBaseModelPinocchioMapping mapping(modelInfo);
  mapping.setPinocchioInterface(interface);

  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, "forwardKinematicsCppAD");

  const pinocchio::Model trueModel = interface.getModel();
  pinocchio::Data trueData = interface.getData();

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const auto q = mapping.getPinocchioJointPosition(state);
    pinocchio::framesForwardKinematics(trueModel, trueData, q);
    const auto positons = forwardKinematics.getPosition(state);

    for(size_t j = 0; j < modelInfo.endEffectorFrameIndices.size(); ++j)
    {
      const auto frameIndex = modelInfo.endEffectorFrameIndices[j];
      EXPECT_TRUE((trueData.oMf[frameIndex].translation() - positons[j]).norm() < tolerance);
    }
  }
}

TEST(PinocchioForwardEndEffectorKinematicsCppAdTest, getOrientation)
{
  const std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  const std::vector<std::string> meldogFake3DofContactNames = {"RFF_link", "RRF_link"};
  const std::vector<std::string> meldogFake6DofContactNames = {"LFF_link", "LRF_link"};
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, 
    baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    meldogFake3DofContactNames, meldogFake6DofContactNames);

  FloatingBaseModelPinocchioMapping mapping(modelInfo);
  mapping.setPinocchioInterface(interface);

  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, "forwardKinematicsCppAD");

  const pinocchio::Model trueModel = interface.getModel();
  pinocchio::Data trueData = interface.getData();

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const auto q = mapping.getPinocchioJointPosition(state);
    pinocchio::framesForwardKinematics(trueModel, trueData, q);
    const auto orientations = forwardKinematics.getOrientation(state);

    for(size_t j = modelInfo.numThreeDofContacts; 
      j < modelInfo.endEffectorFrameIndices.size(); ++j)
    {
      const auto rotationMatrix = getRotationMatrixFromZyxEulerAngles(orientations[j - modelInfo.numThreeDofContacts]);
      const auto frameIndex = modelInfo.endEffectorFrameIndices[j];
      EXPECT_TRUE((trueData.oMf[frameIndex].rotation() - rotationMatrix).norm() < tolerance);
    }
  }
}

TEST(PinocchioForwardEndEffectorKinematicsCppAdTest, getLinearVelocity)
{
  const std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  const std::vector<std::string> meldogFake3DofContactNames = {"RFF_link", "RRF_link"};
  const std::vector<std::string> meldogFake6DofContactNames = {"LFF_link", "LRF_link"};
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, 
    baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    meldogFake3DofContactNames, meldogFake6DofContactNames);

  FloatingBaseModelPinocchioMapping mapping(modelInfo);
  mapping.setPinocchioInterface(interface);

  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, "forwardKinematicsCppAD");

  const pinocchio::Model trueModel = interface.getModel();
  pinocchio::Data trueData = interface.getData();

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const vector_t input = vector_t::Random(modelInfo.inputDim);
    const auto q = mapping.getPinocchioJointPosition(state);
    const auto v = mapping.getPinocchioJointVelocity(state, input);
    pinocchio::forwardKinematics(trueModel, trueData, q, v);
    pinocchio::updateFramePlacements(trueModel, trueData);
    const auto velocities = forwardKinematics.getLinearVelocity(state, input);

    for(size_t j = 0; j < modelInfo.endEffectorFrameIndices.size(); ++j)
    {
      const auto frameIndex = modelInfo.endEffectorFrameIndices[j];
      const vector3_t trueVelocity = pinocchio::getFrameVelocity(trueModel, trueData, frameIndex, 
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
      EXPECT_TRUE((trueVelocity - velocities[j]).norm() < tolerance);
    }
  }
}

TEST(PinocchioForwardEndEffectorKinematicsCppAdTest, getAngularVelocity)
{
  const std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  const std::vector<std::string> meldogFake3DofContactNames = {"RFF_link", "RRF_link"};
  const std::vector<std::string> meldogFake6DofContactNames = {"LFF_link", "LRF_link"};
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, 
    baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, 
    meldogFake3DofContactNames, meldogFake6DofContactNames);

  FloatingBaseModelPinocchioMapping mapping(modelInfo);
  mapping.setPinocchioInterface(interface);

  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, "forwardKinematicsCppAD");

  const pinocchio::Model trueModel = interface.getModel();
  pinocchio::Data trueData = interface.getData();

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const vector_t input = vector_t::Random(modelInfo.inputDim);
    const auto q = mapping.getPinocchioJointPosition(state);
    const auto v = mapping.getPinocchioJointVelocity(state, input);
    pinocchio::forwardKinematics(trueModel, trueData, q, v);
    pinocchio::updateFramePlacements(trueModel, trueData);
    const auto velocities = forwardKinematics.getAngularVelocity(state, input);

    for(size_t j = modelInfo.numThreeDofContacts; 
      j < modelInfo.endEffectorFrameIndices.size(); ++j)
    {
      const auto frameIndex = modelInfo.endEffectorFrameIndices[j];
      const vector3_t trueVelocity = pinocchio::getFrameVelocity(trueModel, trueData, frameIndex, 
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED).angular();
      EXPECT_TRUE((trueVelocity - velocities[j - modelInfo.numThreeDofContacts]).norm() < tolerance);
    }
  }
}