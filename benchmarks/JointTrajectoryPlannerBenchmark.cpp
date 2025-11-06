#include <benchmark/benchmark.h>

#include <legged_locomotion_mpc/trajectory_planners/JointTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/common/AccessHelperFunctions.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/FactoryFunctions.hpp>

#include "../test/include/definitions.hpp"

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::planners;
using namespace ocs2;
using namespace terrain_model;
using namespace floating_base_model;
using namespace multi_end_effector_kinematics;


static void JointTrajectoryPlanner_UPDATE(benchmark::State & state)
{
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

  JointTrajectoryPlanner planner(modelInfo, std::move(kinematicsSolver));
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  const scalar_t initTime = 0.0;
  const scalar_t endTime = 1.0;
  const scalar_t deltaTime = 0.1;
  const size_t referenceSize = (endTime - initTime) / deltaTime + 1;

  TargetTrajectories trajectory;
  TargetTrajectories trajectoryTrue;

  trajectory.timeTrajectory.resize(referenceSize);
  trajectoryTrue.timeTrajectory.resize(referenceSize);

  vector_t systemState  = vector_t::Zero(modelInfo.stateDim);
  vector_t input = vector_t::Zero(modelInfo.inputDim);

  trajectory.stateTrajectory.resize(referenceSize, systemState );
  trajectoryTrue.stateTrajectory.resize(referenceSize, systemState );

  trajectory.inputTrajectory.resize(referenceSize, input);
  trajectoryTrue.inputTrajectory.resize(referenceSize, input);


  using position_trajectories = std::vector<std::vector<vector3_t>>;
  using velocity_trajectories = std::vector<std::vector<vector3_t>>;

  position_trajectories endEffectorPositionsTrajectories;
  position_trajectories endEffectorVelocitiesTrajectories;

  std::vector<vector_t> jointPositionsTrue;
  std::vector<vector_t> jointVelocitiesTrue;

  vector6_t currentBasePose = vector6_t::Random();
  vector6_t currentBaseVelocity = vector6_t::Random();

  vector_t currentJointPositions = vector_t::Random(ACTUATED_DOF_NUM);
  vector_t currentJointVelocities = vector_t::Random(ACTUATED_DOF_NUM);

  systemState  = vector_t::Zero(modelInfo.stateDim);
  input = vector_t::Zero(modelInfo.inputDim);

  floating_base_model::access_helper_functions::
    getBasePose(systemState , modelInfo) = currentBasePose;
  floating_base_model::access_helper_functions::
    getBaseVelocity(systemState , modelInfo) = currentBaseVelocity;
  floating_base_model::access_helper_functions::
    getJointPositions(systemState , modelInfo) = currentJointPositions;
    
  floating_base_model::access_helper_functions::
    getJointVelocities(input, modelInfo) = currentJointVelocities;
  
  trajectory.timeTrajectory[0] = initTime;
  trajectoryTrue.timeTrajectory[0] = initTime;

  trajectoryTrue.stateTrajectory[0] = systemState ;
  trajectoryTrue.inputTrajectory[0] = input;

  // Give only data of base trajectory
  trajectory.stateTrajectory[0] = systemState ;

  const std::vector<vector3_t> startEndEffectorPositions = 
    forwardKinematics.getPosition(systemState );
  endEffectorPositionsTrajectories.push_back(startEndEffectorPositions);

  const std::vector<vector3_t> startEndEffectorVelocities = 
    forwardKinematics.getLinearVelocity(systemState , input);
  endEffectorVelocitiesTrajectories.push_back(startEndEffectorVelocities);

  for(size_t i = 1; i < referenceSize; ++i)
  {
    currentBasePose += currentBaseVelocity * deltaTime;
    currentJointPositions += currentJointVelocities * deltaTime;

    currentBaseVelocity = vector6_t::Random();
    currentJointVelocities = vector_t::Random(ACTUATED_DOF_NUM);

    systemState = vector_t::Zero(modelInfo.stateDim);
    input = vector_t::Zero(modelInfo.inputDim);

    floating_base_model::access_helper_functions::
      getBasePose(systemState , modelInfo) = currentBasePose;
    floating_base_model::access_helper_functions::
      getBaseVelocity(systemState , modelInfo) = currentBaseVelocity;

    // Give only data of base trajectory
    trajectory.stateTrajectory[i] = systemState ;
    trajectory.inputTrajectory[i] = input;
    
    floating_base_model::access_helper_functions::
      getJointPositions(systemState , modelInfo) = currentJointPositions;
    
    floating_base_model::access_helper_functions::
      getJointVelocities(input, modelInfo) = currentJointVelocities;

    trajectory.timeTrajectory[i] = initTime + i * deltaTime;
    trajectoryTrue.timeTrajectory[i] = initTime + i * deltaTime;

    trajectoryTrue.stateTrajectory[i] = systemState ;
    trajectoryTrue.inputTrajectory[i] = input;

    const std::vector<vector3_t> endEffectorPositions = forwardKinematics.getPosition(systemState );
    endEffectorPositionsTrajectories.push_back(endEffectorPositions);

    const std::vector<vector3_t> endEffectorVelocities = 
      forwardKinematics.getLinearVelocity(systemState , input);
    endEffectorVelocitiesTrajectories.push_back(endEffectorVelocities);

  }

  state_vector_t currentState = state_vector_t::Zero(12 + 2 * ACTUATED_DOF_NUM);

  legged_locomotion_mpc::access_helper_functions::
    getBasePose(currentState, modelInfo) = 
    floating_base_model::access_helper_functions::
    getBasePose(trajectoryTrue.stateTrajectory[0], modelInfo);

  legged_locomotion_mpc::access_helper_functions::
    getBaseVelocity(currentState, modelInfo) = 
    floating_base_model::access_helper_functions::
    getBaseVelocity(trajectoryTrue.stateTrajectory[0], modelInfo);

  legged_locomotion_mpc::access_helper_functions::
    getJointPositions(currentState, modelInfo) = 
    floating_base_model::access_helper_functions::
    getJointPositions(trajectoryTrue.stateTrajectory[0], modelInfo);

  legged_locomotion_mpc::access_helper_functions::
    getJointVelocities(currentState, modelInfo) = 
    floating_base_model::access_helper_functions::
    getJointVelocities(trajectoryTrue.inputTrajectory[0], modelInfo);

  for (auto _ : state) {
    planner.updateTrajectory(currentState, trajectory, 
      endEffectorPositionsTrajectories, 
      endEffectorVelocitiesTrajectories);
  }
}

BENCHMARK(JointTrajectoryPlanner_UPDATE);