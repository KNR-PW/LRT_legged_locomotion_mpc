
#include <gtest/gtest.h>
#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>

#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>

#include <pinocchio/spatial/explog.hpp>

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::planners;
using namespace ocs2;
using namespace terrain_model;
using namespace floating_base_model;

const scalar_t tol = 1e-9;

// From ocs2_switched_interface/core/Rotations.h by rgrandia on 22.10.19.
void rotateInPlaceZ(vector3_t& v, scalar_t angle) 
{
  const scalar_t c = cos(angle);
  const scalar_t s = sin(angle);
  const scalar_t vx = v.x();
  const scalar_t vy = v.y();
  v.x() = c * vx - s * vy;
  v.y() = s * vx + c * vy;
}

TEST(BaseTrajectoryPlannerTest, translationOnflatTerrain) 
{
  TerrainPlane flatTerrain(vector3_t::Random(), matrix3_t::Identity());

  PlanarTerrainModel terrainModel(flatTerrain);

  FloatingBaseModelInfo modelInfo;
  modelInfo.actuatedDofNum = 12;
  modelInfo.generalizedCoordinatesNum = 12 + 6;
  modelInfo.inputDim = 12 + 12;
  modelInfo.stateDim = 12 + 12;
  modelInfo.numSixDofContacts = 0;
  modelInfo.numThreeDofContacts = 4;

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = 0.1;
  staticSettings.initialBaseHeight = 2.5;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidtLateral = 0.2;

  BaseTrajectoryPlanner planner(modelInfo, staticSettings);
  planner.updateTerrain(terrainModel);

  const scalar_t initTime = 0.5;
  const scalar_t finalTime = 1.5;

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = std::rand() / scalar_t(RAND_MAX);

  state_vector_t initialState = state_vector_t::Zero(12 + modelInfo.actuatedDofNum * 2);
  const vector3_t initPosition = vector3_t::Random();

  const vector3_t initEulerZyx = vector3_t{std::rand() / scalar_t(RAND_MAX), 0.0, 0.0};

  const vector2_t initPositionXY{initPosition.x(), initPosition.y()};
  initialState.block<3,1>(6, 0) = initPosition;
  initialState.block<3,1>(9, 0) = initEulerZyx;

  TargetTrajectories trajectories;

  planner.updateTargetTrajectory(initTime, finalTime, command, 
    initialState, trajectories);


  const vector3_t initialBasePosition(initPosition.x(), initPosition.y(),
    staticSettings.initialBaseHeight + terrainModel.getSmoothedPositon(initPositionXY).z());

  vector3_t velocityCommand(command.baseHeadingVelocity, command.baseLateralVelocity, 
    command.baseVerticalVelocity);
  rotateInPlaceZ(velocityCommand, initEulerZyx.x());

  const size_t size = (finalTime - initTime) / staticSettings.deltaTime + 1;
  
  // Linear and angular velocities are in BASE frame!
  const vector3_t velocityCommandInBase(command.baseHeadingVelocity, command.baseLateralVelocity, 
    command.baseVerticalVelocity);

  for (size_t i = 0; i < size; ++i) 
  {
    const scalar_t currentTime = trajectories.timeTrajectory[i];
    const vector_t& currentState = trajectories.stateTrajectory[i];

    using namespace floating_base_model::access_helper_functions;
    const vector3_t& basePosition = getBasePosition(currentState, modelInfo);
    const vector3_t& baseOrientationZyx = getBaseOrientationZyx(currentState, modelInfo);
    const vector3_t& baseLinearVelocity = getBaseLinearVelocity(currentState, modelInfo);
    const vector3_t& baseAngularVelocity = getBaseAngularVelocity(currentState, modelInfo);

    EXPECT_TRUE(std::abs(currentTime - (initTime + i * staticSettings.deltaTime)) < tol);
    EXPECT_TRUE(basePosition.isApprox(initialBasePosition + i * 
      staticSettings.deltaTime * velocityCommand, tol));
    EXPECT_TRUE(baseOrientationZyx.isApprox(initEulerZyx, tol));

    // Linear and angular velocities are in BASE frame!
    EXPECT_TRUE(baseLinearVelocity.isApprox(velocityCommandInBase, tol));
    EXPECT_TRUE(baseAngularVelocity.norm() < tol);
  }
}

TEST(BaseTrajectoryPlannerTest, rotationOnflatTerrain) 
{

  const scalar_t slope = 0.0;
  const vector3_t terrainEulerZyx{0.0, slope, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Random(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  FloatingBaseModelInfo modelInfo;
  modelInfo.actuatedDofNum = 12;
  modelInfo.generalizedCoordinatesNum = 12 + 6;
  modelInfo.inputDim = 12 + 12;
  modelInfo.stateDim = 12 + 12;
  modelInfo.numSixDofContacts = 0;
  modelInfo.numThreeDofContacts = 4;

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = 0.1;
  staticSettings.initialBaseHeight = 2.5;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidtLateral = 0.2;

  BaseTrajectoryPlanner planner(modelInfo, staticSettings);
  planner.updateTerrain(terrainModel);

  const scalar_t initTime = 0.5;
  const scalar_t finalTime = 1.5;

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = 0.0;
  command.baseLateralVelocity = 0.0;
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX);

  const vector3_t initPosition = vector3_t::Random();

  const vector3_t planeLocalEulerZyx = vector3_t{0.0, 0.0, 
    0.0};
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
  
  initialBasePosition.z() += slopyTerrain.getSurfaceNormalInWorld().z() * staticSettings.initialBaseHeight;
  
  state_vector_t initialState = state_vector_t::Zero(12 + modelInfo.actuatedDofNum * 2);
  initialState.block<3,1>(6, 0) = initialBasePosition;
  initialState.block<3,1>(9, 0) = initEulerZyx;

  std::cerr << "Init position: " << initialBasePosition.transpose() << std::endl;
  std::cerr << "Init euler: " << initEulerZyx.transpose() << std::endl;

  TargetTrajectories trajectories;

  planner.updateTargetTrajectory(initTime, finalTime, command, 
    initialState, trajectories);

  const size_t size = (finalTime - initTime) / staticSettings.deltaTime + 1;

  const auto eulerZyxRate = vector3_t(command.yawRate, 0.0, 0.0);

  const vector3_t velocityCommandInBase(command.baseHeadingVelocity, command.baseLateralVelocity, 
    command.baseVerticalVelocity);

  vector3_t basePositionTrue = initialBasePosition;
  vector3_t baseOrientationZyxTrue = initEulerZyx;
  vector3_t baseLinearVelocityTrue;
  vector3_t baseAngularVelocityTrue = vector3_t(0.0, 0.0, command.yawRate);

  const vector3_t constHalfEulerDelta = 0.5 * staticSettings.deltaTime * baseAngularVelocityTrue;
  const vector3_t constEulerDelta = staticSettings.deltaTime * baseAngularVelocityTrue;
  for (size_t i = 0; i < size; ++i) 
  {
    const scalar_t currentTime = trajectories.timeTrajectory[i];
    const vector_t& currentState = trajectories.stateTrajectory[i];

    using namespace floating_base_model::access_helper_functions;
    const vector3_t& basePosition = getBasePosition(currentState, modelInfo);
    const vector3_t& baseOrientationZyx = getBaseOrientationZyx(currentState, modelInfo);
    const vector3_t& baseLinearVelocity = getBaseLinearVelocity(currentState, modelInfo);
    const vector3_t& baseAngularVelocity = getBaseAngularVelocity(currentState, modelInfo);

    const matrix3_t currentRotation = getRotationMatrixFromZyxEulerAngles(
      baseOrientationZyxTrue);
    
    const matrix3_t velocityRotation = currentRotation * pinocchio::exp3(constHalfEulerDelta); 

    const vector3_t velocityHeadingVector = velocityRotation.col(0);

    const vector3_t xVelocityAxis = slopyTerrain.projectVectorInWorldOntoPlaneAlongGravity(
      velocityHeadingVector).normalized();
    const vector3_t zVelocityAxis = slopyTerrain.getSurfaceNormalInWorld();
    const vector3_t yVelocityAxis = zVelocityAxis.cross(xVelocityAxis);

    matrix3_t velocityRotationOnPlane;
    velocityRotationOnPlane.col(0) = xVelocityAxis;
    velocityRotationOnPlane.col(1) = yVelocityAxis;
    velocityRotationOnPlane.col(2) = zVelocityAxis;

    EXPECT_TRUE(std::abs(currentTime - (initTime + i * staticSettings.deltaTime)) < tol);
    EXPECT_TRUE(basePosition.isApprox(basePositionTrue, tol));

    EXPECT_TRUE((baseOrientationZyx - baseOrientationZyxTrue).norm() < tol);

    // Linear and angular velocities are in BASE frame!
    EXPECT_TRUE((baseLinearVelocity - currentRotation.transpose() * velocityRotationOnPlane * velocityCommandInBase).norm() < tol);
    EXPECT_TRUE((baseAngularVelocity - baseAngularVelocityTrue).norm() < tol);

    std::cerr << "Iteration: " << i << std::endl;
    std::cerr << "Position: " << basePosition.transpose() << std::endl;
    std::cerr << "Real position: " << basePositionTrue.transpose() << std::endl;
    std::cerr << "Orientation: " << baseOrientationZyx.transpose() << std::endl;
    std::cerr << "Real orientation: " << baseOrientationZyxTrue.transpose() << std::endl;
    std::cerr << "Linear velocity: " << baseLinearVelocity.transpose() << std::endl;
    std::cerr << "Real linear velocity: " << (currentRotation.transpose() * velocityRotationOnPlane * velocityCommandInBase).transpose() << std::endl;
    std::cerr << "Angular velocity: " << baseAngularVelocity.transpose() << std::endl;
    std::cerr << "Real angular velocity: " << baseAngularVelocityTrue.transpose() << std::endl;

    basePositionTrue += velocityRotationOnPlane * velocityCommandInBase * staticSettings.deltaTime;
    
    const matrix3_t newRotation = currentRotation * pinocchio::exp3(constEulerDelta); 
    baseOrientationZyxTrue = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
      newRotation);
  }
}

TEST(BaseTrajectoryPlannerTest, translationAndrotationOnSlopyTerrain) 
{
  const scalar_t slope = -M_PI / 4;
  const vector3_t terrainEulerZyx{0.0, slope, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Random(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  FloatingBaseModelInfo modelInfo;
  modelInfo.actuatedDofNum = 12;
  modelInfo.generalizedCoordinatesNum = 12 + 6;
  modelInfo.inputDim = 12 + 12;
  modelInfo.stateDim = 12 + 12;
  modelInfo.numSixDofContacts = 0;
  modelInfo.numThreeDofContacts = 4;

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = 0.1;
  staticSettings.initialBaseHeight = 2.5;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidtLateral = 0.2;

  BaseTrajectoryPlanner planner(modelInfo, staticSettings);
  planner.updateTerrain(terrainModel);

  const scalar_t initTime = 0.5;
  const scalar_t finalTime = 1.5;

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  std::cerr << "Command velocity: " << command.baseHeadingVelocity << std::endl;
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = std::rand() / scalar_t(RAND_MAX);
  command.yawRate = std::rand() / scalar_t(RAND_MAX);

  const vector3_t initPosition = vector3_t::Random();

  const vector3_t planeLocalEulerZyx = vector3_t{0.0, 0.0, 
    0.0};
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

  std::cerr << "Init position: " << initialBasePosition.transpose() << std::endl;
  std::cerr << "Init euler: " << initEulerZyx.transpose() << std::endl;

  TargetTrajectories trajectories;

  planner.updateTargetTrajectory(initTime, finalTime, command, 
    initialState, trajectories);

  const size_t size = (finalTime - initTime) / staticSettings.deltaTime + 1;

  const auto eulerZyxRate = vector3_t(command.yawRate, 0.0, 0.0);

  const vector3_t velocityCommandInBase(command.baseHeadingVelocity, command.baseLateralVelocity, 
    command.baseVerticalVelocity);

  vector3_t basePositionTrue = initialBasePosition;
  vector3_t baseOrientationZyxTrue = initEulerZyx;
  vector3_t baseLinearVelocityTrue;
  vector3_t baseAngularVelocityTrue = vector3_t(0.0, 0.0, command.yawRate);

  const vector3_t constHalfEulerDelta = 0.5 * staticSettings.deltaTime * baseAngularVelocityTrue;
  const vector3_t constEulerDelta = staticSettings.deltaTime * baseAngularVelocityTrue;
  for (size_t i = 0; i < size; ++i) 
  {
    const scalar_t currentTime = trajectories.timeTrajectory[i];
    const vector_t& currentState = trajectories.stateTrajectory[i];

    using namespace floating_base_model::access_helper_functions;
    const vector3_t& basePosition = getBasePosition(currentState, modelInfo);
    const vector3_t& baseOrientationZyx = getBaseOrientationZyx(currentState, modelInfo);
    const vector3_t& baseLinearVelocity = getBaseLinearVelocity(currentState, modelInfo);
    const vector3_t& baseAngularVelocity = getBaseAngularVelocity(currentState, modelInfo);

    const matrix3_t currentRotation = getRotationMatrixFromZyxEulerAngles(
      baseOrientationZyxTrue);
    
    const matrix3_t velocityRotation = currentRotation * pinocchio::exp3(constHalfEulerDelta); 

    const vector3_t velocityHeadingVector = velocityRotation.col(0);

    const vector3_t xVelocityAxis = slopyTerrain.projectVectorInWorldOntoPlaneAlongGravity(
      velocityHeadingVector).normalized();
    const vector3_t zVelocityAxis = slopyTerrain.getSurfaceNormalInWorld();
    const vector3_t yVelocityAxis = zVelocityAxis.cross(xVelocityAxis);

    matrix3_t velocityRotationOnPlane;
    velocityRotationOnPlane.col(0) = xVelocityAxis;
    velocityRotationOnPlane.col(1) = yVelocityAxis;
    velocityRotationOnPlane.col(2) = zVelocityAxis;

    pinocchio::SE3 currentTransform(currentRotation, basePositionTrue);

    const pinocchio::Motion twistDelta(velocityCommandInBase * staticSettings.deltaTime, constEulerDelta);

    pinocchio::SE3 delta = pinocchio::exp6(
        twistDelta);

    const pinocchio::SE3 newTransform = currentTransform * delta;

    EXPECT_TRUE(std::abs(currentTime - (initTime + i * staticSettings.deltaTime)) < tol);
    EXPECT_TRUE(basePosition.isApprox(basePositionTrue, tol));

    EXPECT_TRUE(baseOrientationZyx.isApprox(baseOrientationZyxTrue, tol));

    // Linear and angular velocities are in BASE frame!
    EXPECT_TRUE((baseLinearVelocity - velocityCommandInBase).norm() < tol);
    EXPECT_TRUE((baseAngularVelocity - baseAngularVelocityTrue).norm() < tol);

    std::cerr << "Iteration: " << i << std::endl;
    std::cerr << "Position: " << basePosition.transpose() << std::endl;
    std::cerr << "Real position: " << basePositionTrue.transpose() << std::endl;
    std::cerr << "Orientation: " << baseOrientationZyx.transpose() << std::endl;
    std::cerr << "Real orientation: " << baseOrientationZyxTrue.transpose() << std::endl;
    std::cerr << "Linear velocity: " << baseLinearVelocity.transpose() << std::endl;
    std::cerr << "Real linear velocity: " << velocityCommandInBase.transpose() << std::endl;
    std::cerr << "Angular velocity: " << baseAngularVelocity.transpose() << std::endl;
    std::cerr << "Real angular velocity: " << baseAngularVelocityTrue.transpose() << std::endl;


    basePositionTrue = newTransform.translation();
    
    const matrix3_t newRotation = newTransform.rotation(); 
    baseOrientationZyxTrue = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
      newRotation);
  }
}