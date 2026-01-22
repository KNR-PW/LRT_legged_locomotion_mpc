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

TEST(SwingTrajectoryPlannerTest, standingInPlace)
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
  swingDynamicSettings.phases.resize(4);
  swingDynamicSettings.swingHeights.resize(4);
  swingDynamicSettings.tangentialProgresses.resize(4);
  swingDynamicSettings.tangentialVelocityFactors.resize(4);
  for(size_t i = 0; i < 4; ++i)
  {
    swingDynamicSettings.phases[i] = 0.5;
    swingDynamicSettings.swingHeights[i] = 0.1;
    swingDynamicSettings.tangentialProgresses[i] = 0.5;
    swingDynamicSettings.tangentialVelocityFactors[i] = 2.0;
  }


  GaitPlanner gaitPlanner(staticParams, dynamicParams, currentPhase, defTime);

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics);

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);

  std::vector<scalar_t> goodTimings = {2.1};
  std::vector<size_t> goodSequence = {15, 15};

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

TEST(SwingTrajectoryPlannerTest, TrotInPlace)
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
  swingDynamicSettings.phases.resize(4);
  swingDynamicSettings.swingHeights.resize(4);
  swingDynamicSettings.tangentialProgresses.resize(4);
  swingDynamicSettings.tangentialVelocityFactors.resize(4);
  for(size_t i = 0; i < 4; ++i)
  {
    swingDynamicSettings.phases[i] = 0.5;
    swingDynamicSettings.swingHeights[i] = 0.1;
    swingDynamicSettings.tangentialProgresses[i] = 0.5;
    swingDynamicSettings.tangentialVelocityFactors[i] = 2.0;
  }

  GaitPlanner gaitPlanner(staticParams, dynamicParams, currentPhase, defTime);

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics);

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);

  std::vector<scalar_t> goodTimings = {0.0, 0.3, 0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1};
  std::vector<size_t> goodSequence = {15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 15};

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

TEST(SwingTrajectoryPlannerTest, Troting)
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
  command.baseHeadingVelocity = 0.5;
  command.baseLateralVelocity = 0.5;
  command.baseVerticalVelocity = 0.0;
  command.yawRate = 0.0;
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  SwingTrajectoryPlanner::StaticSettings swingStaticSettings;
  SwingTrajectoryPlanner::DynamicSettings swingDynamicSettings;
  swingDynamicSettings.invertedPendulumHeight = staticSettings.initialBaseHeight;
  swingDynamicSettings.phases.resize(4);
  swingDynamicSettings.swingHeights.resize(4);
  swingDynamicSettings.tangentialProgresses.resize(4);
  swingDynamicSettings.tangentialVelocityFactors.resize(4);
  for(size_t i = 0; i < 4; ++i)
  {
    swingDynamicSettings.phases[i] = 0.5;
    swingDynamicSettings.swingHeights[i] = 0.1;
    swingDynamicSettings.tangentialProgresses[i] = 0.5;
    swingDynamicSettings.tangentialVelocityFactors[i] = 2.0;
  }

  GaitPlanner gaitPlanner(staticParams, dynamicParams, currentPhase, defTime);

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics);

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);

  std::vector<scalar_t> goodTimings = {0.0, 0.3, 0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1};
  std::vector<size_t> goodSequence = {15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 15};

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
  std::vector<scalar_t> times;
  while(testTime < finalTime)
  { 
    times.push_back(testTime);
    const auto trajectoryPoint = swingPlanner.getEndEffectorTrajectoryPoint(testTime);
    const auto nextTrajectoryPoint = swingPlanner.getEndEffectorTrajectoryPoint(testTime + deltaTime);
    const auto flags = gaitPlanner.getContactFlagsAtTime(testTime);
    const auto nextFlags = gaitPlanner.getContactFlagsAtTime(testTime + deltaTime);

    const auto& positions = trajectoryPoint.positions;
    const auto& nextPositions = nextTrajectoryPoint.positions;
    const auto& velocities = trajectoryPoint.velocities;
    const auto& clearances = trajectoryPoint.clearances;
    const auto& surfaceNormals = trajectoryPoint.surfaceNormals;

    for(size_t i = 0; i < 4; ++i)
    {
      if(flags[i])
      {
        EXPECT_NEAR(velocities[i].x(), 0.0, 1e-6);
        EXPECT_NEAR(velocities[i].y(), 0.0, 1e-6);
        EXPECT_NEAR(clearances[i], 0.0, 1e-6);
        EXPECT_NEAR((surfaceNormals[i] - vector3_t{0.0, 0.0, 1.0}).norm(), 0.0, 1e-6);
      }

      if(!flags[i])
      {
        EXPECT_TRUE(velocities[i].x() > 0.0);
        EXPECT_TRUE(velocities[i].y() > 0.0);
        EXPECT_NEAR((surfaceNormals[i] - vector3_t{0.0, 0.0, 1.0}).norm(), 0.0, 1e-6);
      }

      if(flags[i] && nextFlags[i])
      {
        EXPECT_NEAR(nextPositions[i].x(), positions[i].x(), 1e-6);
        EXPECT_NEAR(nextPositions[i].y(), positions[i].y(), 1e-6);
      }

      if(!flags[i] && !nextFlags[i]) 
      {
        EXPECT_TRUE(nextPositions[i].x() > positions[i].x());
        EXPECT_TRUE(nextPositions[i].y() > positions[i].y());
      }
    }
    testTime += deltaTime;
  }

  const auto footTangentialConstraints = swingPlanner.getFootTangentialConstraintTrajectories(times);
  const auto& constraintTimes = footTangentialConstraints.times;
  const auto& constraintsTrajectory = footTangentialConstraints.constraints;

  const auto contactFlags = gaitPlanner.getContactFlagsAtTime(initTime);
  std::vector<bool> isInFirstContact(4);
  for(size_t i = 0; i < 4; ++i)
  {
    isInFirstContact[i] = contactFlag[i];
  }

  for(size_t index = 0; index < constraintTimes.size(); ++index)
  {
    const auto contactFlags = gaitPlanner.getContactFlagsAtTime(constraintTimes[index]);
    const auto& constraints = constraintsTrajectory[index];
    for(size_t i = 0; i < 4; ++i)
    {
      // When robot leg is already in contact at the start, constraint does not work
      if(contactFlags[i] && isInFirstContact[i])  EXPECT_TRUE(!constraints[i].isActive() && isInFirstContact[i]);
      
      // When robot legs in in motion after first contact, constraint does not work
      if(!contactFlags[i] && isInFirstContact[i])  
      {
        isInFirstContact[i] = !isInFirstContact[i];
        EXPECT_TRUE(!constraints[i].isActive());
      }

      // Later contacts (after first one, if leg started from contact)
      if(contactFlags[i] && !isInFirstContact[i])
      {
        EXPECT_TRUE(constraints[i].isActive() && constraints[i].A.rows() == 3);
      }

      // Later motions (after first one, if leg started from contact)
      if(!contactFlags[i] && !isInFirstContact[i])
      {
        EXPECT_TRUE(!constraints[i].isActive() && !isInFirstContact[i]);
      }
    }
  }
}