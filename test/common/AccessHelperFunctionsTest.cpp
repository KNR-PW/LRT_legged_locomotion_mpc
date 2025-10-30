#include <gtest/gtest.h>

#include <legged_locomotion_mpc/common/AccessHelperFunctions.hpp>
#include "../include/definitions.hpp"

#include <floating_base_model/FactoryFunctions.hpp>


using namespace floating_base_model;

using namespace legged_locomotion_mpc;
using namespace access_helper_functions;

TEST(AccessHelperFunctions, RobotStateConst)
{
  ocs2::PinocchioInterface interface = floating_base_model::createPinocchioInterfaceFromUrdfFile(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto state = getAccessTestRobotState();

  // READ ACCESS

  const vector6_t baseVelocity = getBaseVelocity(state, info);
  const Eigen::Vector<double, 3> baseLinearVelocity = getBaseLinearVelocity(state, info);
  const Eigen::Vector<double, 3> baseAngularVelocity = getBaseAngularVelocity(state, info);

  ASSERT_TRUE(baseLinearVelocity== Eigen::Vector3d(0.0, 1.0, 2.0));
  ASSERT_TRUE(baseAngularVelocity == Eigen::Vector3d(3.0, 4.0, 5.0));
  const vector6_t baseVelocityTrue = vector6_t(0.0, 1.0, 2.0, 3.0, 4.0, 5.0);
  ASSERT_TRUE(baseVelocity == baseVelocityTrue);
  
  const vector6_t basePose = getBasePose(state, info);
  const Eigen::Vector<double, 3> basePosition = getBasePosition(state, info);
  const Eigen::Vector<double, 3> baseOrientation = getBaseOrientationZyx(state, info);

  ASSERT_TRUE(basePosition == Eigen::Vector3d(6.0, 7.0, 8.0));
  ASSERT_TRUE(baseOrientation == Eigen::Vector3d(9.0, 10.0, 11.0));
  const vector6_t basePoseTrue = vector6_t(6.0, 7.0, 8.0, 9.0, 10.0, 11.0);
  ASSERT_TRUE(basePose == basePoseTrue);

  const Eigen::Vector<double, 12> jointPositions = getJointPositions(state, info);
  const Eigen::Vector<double, 12> jointPositionsTrue = state.block<12, 1>(12, 0);
  ASSERT_TRUE(jointPositions == jointPositionsTrue);

  const Eigen::Vector<double, 12> jointVelocities = getJointVelocities(state, info);
  const Eigen::Vector<double, 12> jointVelocitiesTrue = state.block<12, 1>(24, 0);
  ASSERT_TRUE(jointVelocities == jointVelocitiesTrue);
}

TEST(AccessHelperFunctions, RobotStateNonConst)
{
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(meldogWithBaseLinkUrdfFile, baseLink);
  auto info = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);
  const auto state = getAccessTestRobotState();

  // READ/WRITE ACCESS

  vector6_t baseVelocity = getBaseVelocity(state, info);
  Eigen::Vector<double, 3> baseLinearVelocity = getBaseLinearVelocity(state, info);
  Eigen::Vector<double, 3> baseAngularVelocity = getBaseAngularVelocity(state, info);

  ASSERT_TRUE(baseLinearVelocity== Eigen::Vector3d(0.0, 1.0, 2.0));
  ASSERT_TRUE(baseAngularVelocity == Eigen::Vector3d(3.0, 4.0, 5.0));
  vector6_t baseVelocityTrue = vector6_t(0.0, 1.0, 2.0, 3.0, 4.0, 5.0);
  ASSERT_TRUE(baseVelocity == baseVelocityTrue);
  
  vector6_t basePose = getBasePose(state, info);
  Eigen::Vector<double, 3> basePosition = getBasePosition(state, info);
  Eigen::Vector<double, 3> baseOrientation = getBaseOrientationZyx(state, info);

  ASSERT_TRUE(basePosition == Eigen::Vector3d(6.0, 7.0, 8.0));
  ASSERT_TRUE(baseOrientation == Eigen::Vector3d(9.0, 10.0, 11.0));
  vector6_t basePoseTrue = vector6_t(6.0, 7.0, 8.0, 9.0, 10.0, 11.0);
  ASSERT_TRUE(basePose == basePoseTrue);

  Eigen::Vector<double, 12> jointPositions = getJointPositions(state, info);
  Eigen::Vector<double, 12> jointPositionsTrue = state.block<12, 1>(12, 0);
  ASSERT_TRUE(jointPositions == jointPositionsTrue);

  Eigen::Vector<double, 12> jointVelocities = getJointVelocities(state, info);
  Eigen::Vector<double, 12> jointVelocitiesTrue = state.block<12, 1>(24, 0);
  ASSERT_TRUE(jointVelocities == jointVelocitiesTrue);
}