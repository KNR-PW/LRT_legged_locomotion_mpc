#include <gtest/gtest.h>

#include <legged_locomotion_mpc/locomotion/SwingTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/common/AccessHelperFunctions.hpp>
#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>

#include <floating_base_model/FactoryFunctions.hpp>

#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

#include "../test/include/definitions.hpp"

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::locomotion;
using namespace legged_locomotion_mpc::planners;
using namespace floating_base_model;
using namespace ocs2;
using namespace terrain_model;

const double eps = 1e-9;
const size_t TEST_NUM = 100;

// Not round because std::lower_bound is sensitive for time points of mode change
const size_t ITERATIONS = 50;

TEST(LeggedReferenceManagerTest, standingInPlace)
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.05;

  /** STANDING IN PLACE */
  scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.plannerFrequency = 2.0;
  staticParams.timeHorizion = 0.7;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 0.0;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {0, 0, 0};

  auto modeSequenceTemplate = getDynamicModeSequenceTemplate(currentPhase,
    finalTime, staticParams, dynamicParams);

  const vector3_t terrainEulerZyx{0.0, 0.0, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Zero(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = deltaTime;
  staticSettings.initialBaseHeight = 0.25 * std::sqrt(2.0);
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 0.5;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidtLateral = 0.2;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = 0.0;
  command.baseLateralVelocity = 0.0;
  command.baseVerticalVelocity = 0.0;
  command.yawRate = 0.0;
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  SwingTrajectoryPlanner::StaticSettings swingStaticSettings;
  SwingTrajectoryPlanner::DynamicSettings swingDynamicSettings;
  swingDynamicSettings.invertedPendulumHeight = staticSettings.initialBaseHeight;

  GaitPlanner gaitPlanner(staticParams, dynamicParams, modeSequenceTemplate, 
    currentPhase, defTime);

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics);

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);

  std::vector<scalar_t> goodTimings = {};
  std::vector<size_t> goodSequence = {15};

  EXPECT_TRUE(modeSchedule.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTimings.size(); ++i)
  {
    EXPECT_NEAR(modeSchedule.eventTimes[i], goodTimings[i], 1e-6);
  }

  const vector3_t initPosition = vector3_t{0.0, 0.0, 0.0};

  const vector3_t planeLocalEulerZyx = vector3_t{0.0, 0.0, 0.0};

  matrix3_t initRotationOnPlane = getRotationMatrixFromZyxEulerAngles(planeLocalEulerZyx);

  const vector3_t headingVector = initRotationOnPlane.col(0);
  const vector3_t xAxis = slopyTerrain.projectVectorInWorldOntoPlaneAlongGravity(headingVector).normalized();
  const vector3_t zAxis = slopyTerrain.getSurfaceNormalInWorld();
  const vector3_t yAxis = zAxis.cross(xAxis);

  matrix3_t initRotationMatrix;
  initRotationMatrix.col(0) = xAxis;
  initRotationMatrix.col(1) = yAxis;
  initRotationMatrix.col(2) = zAxis;

  const vector3_t initEulerZyx = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
    initRotationMatrix);

  const vector2_t initPositionXY{initPosition.x(), initPosition.y()};

  vector3_t initialBasePosition(initPosition.x(), initPosition.y(),
    terrainModel.getSmoothedPositon(initPositionXY).z());
  
  initialBasePosition.z() += staticSettings.initialBaseHeight / slopyTerrain.getSurfaceNormalInWorld().z();
  
  state_vector_t initialState = state_vector_t::Zero(12 + modelInfo.actuatedDofNum * 2);
  initialState.block<3,1>(6, 0) = initialBasePosition;
  initialState.block<3,1>(9, 0) = initEulerZyx;

  legged_locomotion_mpc::access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326;
  contact_flags_t contactFlag = modeSchedule.modeAtTime(initTime);
  
  basePlanner.updateTerrain(terrainModel);
  swingPlanner.updateTerrain(terrainModel);

  TargetTrajectories trajectory;

  basePlanner.updateTargetTrajectory(initTime, finalTime, command, initialState, 
    trajectory);
  swingPlanner.updateSwingMotions(initTime, finalTime, initialState, trajectory, 
    modeSchedule);
  
  scalar_t testTime = initTime;
  const auto startPositions = swingPlanner.getEndEffectorPositions(testTime);
  while(testTime < finalTime)
  { 
    const auto positions = swingPlanner.getEndEffectorPositions(testTime);
    for(size_t i = 0; i < 4; ++i)
    {
      EXPECT_NEAR(positions[i].x(), startPositions[i].x(), 1e-6);
      EXPECT_NEAR(positions[i].y(), startPositions[i].y(), 1e-6);
      EXPECT_NEAR(positions[i].z(), startPositions[i].z(), 1e-6);
    }
    testTime += deltaTime;
  }
}

TEST(LeggedReferenceManagerTest, TrotInPlace)
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.05;

  /* STANDING TROT */
  scalar_t currentPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  staticParams.plannerFrequency = 2.0;
  staticParams.timeHorizion = 0.7;

  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;
  
  dynamicParams.phaseOffsets = {-currentPhase , -currentPhase , 0};

  auto modeSequenceTemplate = getDynamicModeSequenceTemplate(currentPhase,
    finalTime, staticParams, dynamicParams);

  const vector3_t terrainEulerZyx{0.0, 0.0, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Zero(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = deltaTime;
  staticSettings.initialBaseHeight = 0.25 * std::sqrt(2.0);
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 0.5;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidtLateral = 0.2;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = 0.0;
  command.baseLateralVelocity = 0.0;
  command.baseVerticalVelocity = 0.0;
  command.yawRate = 0.0;
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  SwingTrajectoryPlanner::StaticSettings swingStaticSettings;
  SwingTrajectoryPlanner::DynamicSettings swingDynamicSettings;
  swingDynamicSettings.invertedPendulumHeight = staticSettings.initialBaseHeight;

  GaitPlanner gaitPlanner(staticParams, dynamicParams, modeSequenceTemplate, 
    currentPhase, defTime);

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics);

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);

  std::vector<scalar_t> goodTimings = {0.0, 0.3, 0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05};
  std::vector<size_t> goodSequence = {15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15};

  EXPECT_TRUE(modeSchedule.modeSequence == goodSequence);

  for(size_t i = 0; i < goodTimings.size(); ++i)
  {
    EXPECT_NEAR(modeSchedule.eventTimes[i], goodTimings[i], 1e-6);
  }

  const vector3_t initPosition = vector3_t{0.0, 0.0, 0.0};

  const vector3_t planeLocalEulerZyx = vector3_t{0.0, 0.0, 0.0};

  matrix3_t initRotationOnPlane = getRotationMatrixFromZyxEulerAngles(planeLocalEulerZyx);

  const vector3_t headingVector = initRotationOnPlane.col(0);
  const vector3_t xAxis = slopyTerrain.projectVectorInWorldOntoPlaneAlongGravity(headingVector).normalized();
  const vector3_t zAxis = slopyTerrain.getSurfaceNormalInWorld();
  const vector3_t yAxis = zAxis.cross(xAxis);

  matrix3_t initRotationMatrix;
  initRotationMatrix.col(0) = xAxis;
  initRotationMatrix.col(1) = yAxis;
  initRotationMatrix.col(2) = zAxis;

  const vector3_t initEulerZyx = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
    initRotationMatrix);

  const vector2_t initPositionXY{initPosition.x(), initPosition.y()};

  vector3_t initialBasePosition(initPosition.x(), initPosition.y(),
    terrainModel.getSmoothedPositon(initPositionXY).z());
  
  initialBasePosition.z() += staticSettings.initialBaseHeight / slopyTerrain.getSurfaceNormalInWorld().z();
  
  state_vector_t initialState = state_vector_t::Zero(12 + modelInfo.actuatedDofNum * 2);
  initialState.block<3,1>(6, 0) = initialBasePosition;
  initialState.block<3,1>(9, 0) = initEulerZyx;

  legged_locomotion_mpc::access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326;
  contact_flags_t contactFlag = modeSchedule.modeAtTime(initTime);
  
  basePlanner.updateTerrain(terrainModel);
  swingPlanner.updateTerrain(terrainModel);

  TargetTrajectories trajectory;

  basePlanner.updateTargetTrajectory(initTime, finalTime, command, initialState, 
    trajectory);
  swingPlanner.updateSwingMotions(initTime, finalTime, initialState, trajectory, 
    modeSchedule);
  
  scalar_t testTime = initTime;
  const auto startPositions = swingPlanner.getEndEffectorPositions(testTime);
  while(testTime < finalTime)
  { 
    const auto positions = swingPlanner.getEndEffectorPositions(testTime);
    for(size_t i = 0; i < 4; ++i)
    {
      EXPECT_NEAR(positions[i].x(), startPositions[i].x(), 1e-6);
      EXPECT_NEAR(positions[i].y(), startPositions[i].y(), 1e-6);
    }
    testTime += deltaTime;
  }
}