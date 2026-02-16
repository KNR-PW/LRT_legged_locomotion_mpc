#include <gtest/gtest.h>

#include "../include/definitions.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/FactoryFunctions.hpp>
#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <legged_locomotion_mpc/collision/PinocchioForwardCollisionKinematicsCppAd.hpp>

using namespace ocs2;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::collision;

const scalar_t tolerance = 1e-6;

const size_t NUM_TEST = 20;

TEST(PinocchioForwardCollisionKinematicsCppAdTest, getCollisonNumber)
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

  CollisionSettings collisionSettings;
  collisionSettings.collisionLinkNames = meldogCollisions;
  collisionSettings.terrainCollisionLinkNames = {"LFLL_link", "RFLL_link"};
  collisionSettings.selfCollisionPairNames = {{"LFLL_link", "RFLL_link"}, {"LFLL_link", "LRLL_link"}, {"RRLL_link", "RRF_link"}};
  collisionSettings.maxExcesses = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.2);
  collisionSettings.relaxations = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.5);
  collisionSettings.shrinkRatio = 0.5;

  const std::string modelName = "collision_kinematics";

  PinocchioForwardCollisionKinematicsCppAd collisionKinematics(interface, modelInfo, 
    collisionSettings, modelName);

  const pinocchio::Model trueModel = interface.getModel();
  pinocchio::Data trueData = interface.getData();

  std::vector<size_t> collisionIndexes;

  for(const auto& collisionName: meldogCollisions)
  {
    collisionIndexes.push_back(trueModel.getFrameId(collisionName));
  }
  EXPECT_TRUE(collisionKinematics.getCollisionNumber() == collisionIndexes.size());
}

TEST(PinocchioForwardCollisionKinematicsCppAdTest, getPosition)
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

  CollisionSettings collisionSettings;
  collisionSettings.collisionLinkNames = meldogCollisions;
  collisionSettings.terrainCollisionLinkNames = {"LFLL_link", "RFLL_link"};
  collisionSettings.selfCollisionPairNames = {{"LFLL_link", "RFLL_link"}, {"LFLL_link", "LRLL_link"}, {"RRLL_link", "RRF_link"}};
  collisionSettings.maxExcesses = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.2);
  collisionSettings.relaxations = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.5);
  collisionSettings.shrinkRatio = 0.5;

  const std::string modelName = "collision_kinematics";

  PinocchioForwardCollisionKinematicsCppAd collisionKinematics(interface, modelInfo, 
    collisionSettings, modelName);

  const pinocchio::Model trueModel = interface.getModel();
  pinocchio::Data trueData = interface.getData();

  std::vector<size_t> collisionIndexes;

  for(const auto& collisionName: meldogCollisions)
  {
    collisionIndexes.push_back(trueModel.getFrameId(collisionName));
  }

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const auto q = mapping.getPinocchioJointPosition(state);
    pinocchio::framesForwardKinematics(trueModel, trueData, q);
    const auto positons = collisionKinematics.getPosition(state);

    for(size_t j = 0; j < collisionIndexes.size(); ++j)
    {
      const auto frameIndex = collisionIndexes[j];
      EXPECT_TRUE((trueData.oMf[frameIndex].translation() - positons[j]).norm() < tolerance);
    }
  }
}

TEST(PinocchioForwardCollisionKinematicsCppAdTest, getOrientation)
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

  CollisionSettings collisionSettings;
  collisionSettings.collisionLinkNames = meldogCollisions;
  collisionSettings.terrainCollisionLinkNames = {"LFLL_link", "RFLL_link"};
  collisionSettings.selfCollisionPairNames = {{"LFLL_link", "RFLL_link"}, {"LFLL_link", "LRLL_link"}, {"RRLL_link", "RRF_link"}};
  collisionSettings.maxExcesses = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.2);
  collisionSettings.relaxations = std::vector<scalar_t>(
    meldog3DofContactNames.size() + collisionSettings.collisionLinkNames.size(), 0.5);
  collisionSettings.shrinkRatio = 0.5;

  const std::string modelName = "collision_kinematics";

  PinocchioForwardCollisionKinematicsCppAd collisionKinematics(interface, modelInfo, 
    collisionSettings, modelName);

  const pinocchio::Model trueModel = interface.getModel();
  pinocchio::Data trueData = interface.getData();

  std::vector<size_t> collisionIndexes;

  for(const auto& collisionName: meldogCollisions)
  {
    collisionIndexes.push_back(trueModel.getFrameId(collisionName));
  }

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector_t state = vector_t::Random(modelInfo.stateDim);
    const auto q = mapping.getPinocchioJointPosition(state);
    pinocchio::framesForwardKinematics(trueModel, trueData, q);
    const auto orientations = collisionKinematics.getOrientation(state);

    for(size_t j = 0; j < collisionIndexes.size(); ++j)
    {
      const auto rotationMatrix = getRotationMatrixFromZyxEulerAngles(orientations[j]);
      const auto frameIndex = collisionIndexes[j];
      EXPECT_TRUE((trueData.oMf[frameIndex].rotation() - rotationMatrix).norm() < tolerance);
    }
  }
}