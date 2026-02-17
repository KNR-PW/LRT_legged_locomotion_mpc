#include <benchmark/benchmark.h>

#include <legged_locomotion_mpc/locomotion/SwingTrajectoryPlanner.hpp>

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>

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

static void SwingTrajectoryPlanner_UPDATE_AND_GENERATE(benchmark::State & state)
{

  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 1.0;

  /* STANDING TROT */
  scalar_t startingPhase = 3.5 / 7.0;

  GaitStaticParameters staticParams;
  staticParams.endEffectorNumber = 4;
  
  staticParams.timeHorizion = 0.7;

  /* FLYING TROT */
  GaitDynamicParameters dynamicParams;
  dynamicParams.swingRatio =  0.33 / 0.6;
  dynamicParams.steppingFrequency = 1.0 / 0.6;
  scalar_t currentPhase = dynamicParams.swingRatio;

  dynamicParams.phaseOffsets = {-currentPhase + 0.03 / 0.6, -currentPhase + 0.03 / 0.6, 0};


  auto modeSequenceTemplate = getDynamicModeSequenceTemplate(startingPhase,
    finalTime, staticParams, dynamicParams);

  GaitPlanner gaitPlanner(staticParams, dynamicParams, modeSequenceTemplate, startingPhase, initTime);

  const auto modeSchedule = gaitPlanner.getModeSchedule(initTime, finalTime);

  const scalar_t slope = -M_PI / 4;
  const vector3_t terrainEulerZyx{0.0, slope, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane slopyTerrain(vector3_t::Random(), terrainRotation.transpose());

  PlanarTerrainModel terrainModel(slopyTerrain);

  BaseTrajectoryPlanner::StaticSettings staticSettings;
  staticSettings.deltaTime = 0.1;
  staticSettings.initialBaseHeight = 2.5;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidtLateral = 0.2;

  std::string urdfPathName = meldogWithBaseLinkUrdfFile;

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  BaseTrajectoryPlanner basePlanner(modelInfo, staticSettings);
  basePlanner.updateTerrain(terrainModel);

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = std::rand() / scalar_t(RAND_MAX);
  command.yawRate = std::rand() / scalar_t(RAND_MAX);

  state_vector_t initialState = state_vector_t::Zero(12 + modelInfo.actuatedDofNum * 2);
  const vector3_t initPosition = vector3_t::Random();
  
  TargetTrajectories trajectories;
  basePlanner.updateTargetTrajectory(initTime, finalTime, command, initialState, 
    trajectories);
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  SwingTrajectoryPlanner::StaticSettings swingStaticSettings;
  SwingTrajectoryPlanner::DynamicSettings swingDynamicSettings;

  SwingTrajectoryPlanner swingPlanner(modelInfo, swingStaticSettings, 
    swingDynamicSettings, forwardKinematics); 

  swingPlanner.updateTerrain(terrainModel);

  for (auto _ : state) {
    swingPlanner.updateSwingMotions(initTime, finalTime, 
      initialState, trajectories, modeSchedule);
    benchmark::DoNotOptimize(swingPlanner.getEndEffectorPositionTrajectories(
      trajectories.timeTrajectory));
    benchmark::DoNotOptimize(swingPlanner.getEndEffectorVelocityTrajectories(
      trajectories.timeTrajectory));
  }
}

BENCHMARK(SwingTrajectoryPlanner_UPDATE_AND_GENERATE);