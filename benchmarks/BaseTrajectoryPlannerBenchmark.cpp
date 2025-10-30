#include <benchmark/benchmark.h>


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

static void BaseTrajectoryPlanner_UPDATE(benchmark::State & state)
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
  staticSettings.deltaTime = 0.05;
  staticSettings.initialBaseHeight = 2.5;
  staticSettings.minimumBaseHeight = 0.1;
  staticSettings.maximumBaseHeight = 5.0;
  staticSettings.nominalBaseWidthHeading = 0.2;
  staticSettings.nominalBaseWidtLateral = 0.2;

  BaseTrajectoryPlanner planner(modelInfo, staticSettings);
  planner.updateTerrain(terrainModel);

  const scalar_t initTime = 0.0;
  const scalar_t finalTime = 1.0;

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseLateralVelocity = std::rand() / scalar_t(RAND_MAX);
  command.baseVerticalVelocity = std::rand() / scalar_t(RAND_MAX);
  command.yawRate = std::rand() / scalar_t(RAND_MAX);

  state_vector_t initialState = state_vector_t::Zero(12 + modelInfo.actuatedDofNum * 2);
  const vector3_t initPosition = vector3_t::Random();

  TargetTrajectories trajectories;
  for (auto _ : state) {
    planner.updateTargetTrajectory(initTime, finalTime, command, 
    initialState, trajectories);
  }
}

BENCHMARK(BaseTrajectoryPlanner_UPDATE);

BENCHMARK_MAIN();