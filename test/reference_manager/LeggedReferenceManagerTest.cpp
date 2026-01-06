#include <gtest/gtest.h>

#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>
#include <legged_locomotion_mpc/common/AccessHelperFunctions.hpp>

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
using namespace multi_end_effector_kinematics;

const double eps = 1e-3;

TEST(LeggedReferenceManagerTest, getContactFlags) 
{
  const scalar_t defTime = 0.0;
  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 2.1;
  const scalar_t deltaTime = 0.2;

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

  const scalar_t slope = 0.0;
  const vector3_t terrainEulerZyx{0.0, slope, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Random(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = deltaTime;
  staticSettings.initialBaseHeight = 0.4;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidtLateral = 0.2;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  // solverSettings.stepCoefficient = 0.1;
  solverSettings.tolerance = 1e-3;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX) / 100.0;
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX) / 100.0;
  command.baseVerticalVelocity = 0.0;
  command.yawRate = std::rand() / scalar_t(RAND_MAX) / 100.0;
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  SwingTrajectoryPlanner::StaticSettings swingStaticSettings;
  SwingTrajectoryPlanner::DynamicSettings swingDynamicSettings;

  GaitPlanner gaitPlanner(staticParams, dynamicParams, modeSequenceTemplate, 
    currentPhase, defTime);

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics);

  JointTrajectoryPlanner jointPlanner(modelInfo, std::move(kinematicsSolver));

  ContactForceWrenchTrajectoryPlanner forcePlanner(modelInfo);
  
  LeggedReferenceManager::Settings managerSettings;

  LeggedReferenceManager manager(modelInfo, managerSettings, gaitPlanner, swingPlanner, basePlanner, 
    jointPlanner, forcePlanner);

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);

  const vector3_t initPosition = vector3_t::Random();

  const vector3_t planeLocalEulerZyx = vector3_t{std::rand() / scalar_t(RAND_MAX), 0.0, 
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

  legged_locomotion_mpc::access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.52359877559, 1.0471975512, 0, -0.52359877559, 1.0471975512, 0, -0.52359877559, 1.0471975512, 0, -0.52359877559, 1.0471975512;
  
  contact_flags_t contactFlag = modeSchedule.modeAtTime(initTime);

  std::unique_ptr<TerrainModel> terrainModelPtr = 
    std::make_unique<PlanarTerrainModel>(slopyTerrain);
  
  manager.initialize(initTime, finalTime, initialState, contactFlag, 
    std::move(dynamicParams), std::move(terrainModelPtr));

  std::vector<scalar_t> goodTimings = {0, 0.3, 0.35, 0.65, 0.7, 1, 1.05, 1.35, 1.4, 1.7, 1.75, 2.05, 2.1};
  std::vector<size_t> goodSequence = {15, 9, 15, 6, 15, 9, 15, 6, 15, 9, 15, 6, 15, 15};
  const auto mySequence = manager.getModeSchedule().modeSequence;

  EXPECT_TRUE(mySequence == goodSequence);
  for(size_t i = 0; i < goodSequence.size(); ++i)
  {
    const auto mode = contactFlags2ModeNumber(manager.getContactFlags(goodTimings[i] - eps));
    const auto trueMode = goodSequence[i];
    EXPECT_TRUE(mode == trueMode);
  }
}