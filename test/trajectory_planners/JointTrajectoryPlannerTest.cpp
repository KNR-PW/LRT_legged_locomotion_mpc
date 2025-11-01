#include <gtest/gtest.h>

#include <legged_locomotion_mpc/trajectory_planners/JointTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/common/AccessHelperFunctions.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/FactoryFunctions.hpp>

#include "../include/definitions.hpp"

using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::planners;
using namespace ocs2;
using namespace terrain_model;
using namespace floating_base_model;
using namespace multi_end_effector_kinematics;

const scalar_t positionTolerance = 1e-1;
const scalar_t velocityTolerance = 1e-3;

const scalar_t stateTolerance = 1e-5;
const scalar_t inputTolerance = 1e-5;

const size_t NUM_TEST = 100;
const size_t POSITION_CHANGE_CONST = 100;

TEST(JointTrajectoryPlannerTest, computeJointPositions) 
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  solverSettings.tolerance = 1e-8;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);
    
  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  JointTrajectoryPlanner planner(modelInfo, std::move(kinematicsSolver));
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector6_t currentBasePose = vector6_t::Random();
    const vector_t currentJointPositions = vector_t::Random(ACTUATED_DOF_NUM) * M_PI_2;
    const vector_t jointPositionsTrue = currentJointPositions + 
      vector_t::Random(ACTUATED_DOF_NUM) / POSITION_CHANGE_CONST;

    vector_t state = vector_t::Zero(modelInfo.stateDim);

    floating_base_model::access_helper_functions::
      getBasePose(state, modelInfo) = currentBasePose;
    floating_base_model::access_helper_functions::
      getJointPositions(state, modelInfo) = jointPositionsTrue;

    const std::vector<vector3_t> endEffectorPositions = 
      forwardKinematics.getPosition(state);

    const vector_t calculatedJointPositions = planner.computeJointPositions(
      currentJointPositions, currentBasePose, endEffectorPositions);
    
    std::cerr << (calculatedJointPositions - jointPositionsTrue).norm() << std::endl;
    EXPECT_TRUE((calculatedJointPositions - jointPositionsTrue).norm() < positionTolerance);
  }
}

TEST(JointTrajectoryPlannerTest, computeJointVelocities) 
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  solverSettings.tolerance = 1e-8;
  MultiEndEffectorKinematics kinematicsSolver(urdfPathName, modelSettings, 
    solverSettings, solverName);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfPathName, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  JointTrajectoryPlanner planner(modelInfo, std::move(kinematicsSolver));
  
  std::string modelName = "meldogForwardKinematics";
  PinocchioForwardEndEffectorKinematicsCppAd forwardKinematics(interface, 
    modelInfo, modelName);

  for(size_t i = 0; i < NUM_TEST; ++i)
  {
    const vector6_t currentBasePose = vector6_t::Random();
    const vector6_t currentBaseVelocity = vector6_t::Random();
    const vector_t currentJointPositions = vector_t::Random(ACTUATED_DOF_NUM) * M_PI_2;
    const vector_t jointVelocitiesTrue = vector_t::Random(ACTUATED_DOF_NUM);

    vector_t state = vector_t::Zero(modelInfo.stateDim);
    vector_t input = vector_t::Zero(modelInfo.inputDim);

    floating_base_model::access_helper_functions::
      getBasePose(state, modelInfo) = currentBasePose;
    floating_base_model::access_helper_functions::
      getBaseVelocity(state, modelInfo) = currentBaseVelocity;
    floating_base_model::access_helper_functions::
      getJointPositions(state, modelInfo) = currentJointPositions;
    

    floating_base_model::access_helper_functions::
      getJointVelocities(input, modelInfo) = jointVelocitiesTrue;

    const std::vector<vector3_t> endEffectorVelocities = 
      forwardKinematics.getLinearVelocity(state, input);

    const vector_t calculatedJointVelocities = planner.computeJointVelocities(
      currentJointPositions, currentBasePose, currentBaseVelocity, endEffectorVelocities);
    
    EXPECT_TRUE((calculatedJointVelocities - jointVelocitiesTrue).norm() < velocityTolerance);
  }
}

TEST(JointTrajectoryPlannerTest, updateTrajectory) 
{
  std::string urdfPathName = meldogWithBaseLinkUrdfFile;
  std::string solverName = "NewtonRaphson";

  KinematicsModelSettings modelSettings;
  modelSettings.baseLinkName = baseLink;
  modelSettings.threeDofEndEffectorNames = meldog3DofContactNames;

  InverseSolverSettings solverSettings;
  solverSettings.tolerance = 1e-8;
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
  const size_t referenceSize = (endTime - initTime) / 0.1 + 1;

  TargetTrajectories trajectory;
  TargetTrajectories trajectoryTrue;

  trajectory.timeTrajectory.resize(referenceSize);
  trajectoryTrue.timeTrajectory.resize(referenceSize);

  vector_t state = vector_t::Zero(modelInfo.stateDim);
  vector_t input = vector_t::Zero(modelInfo.inputDim);

  trajectory.stateTrajectory.resize(referenceSize, state);
  trajectoryTrue.stateTrajectory.resize(referenceSize, state);

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

  state = vector_t::Zero(modelInfo.stateDim);
  input = vector_t::Zero(modelInfo.inputDim);

  floating_base_model::access_helper_functions::
    getBasePose(state, modelInfo) = currentBasePose;
  floating_base_model::access_helper_functions::
    getBaseVelocity(state, modelInfo) = currentBaseVelocity;
  floating_base_model::access_helper_functions::
    getJointPositions(state, modelInfo) = currentJointPositions;
    
  floating_base_model::access_helper_functions::
    getJointVelocities(input, modelInfo) = currentJointVelocities;
  
  trajectory.timeTrajectory[0] = initTime;
  trajectoryTrue.timeTrajectory[0] = initTime;

  trajectoryTrue.stateTrajectory[0] = state;
  trajectoryTrue.inputTrajectory[0] = input;

  // Give only data of base trajectory
  trajectory.stateTrajectory[0] = state;

  const std::vector<vector3_t> startEndEffectorPositions = 
    forwardKinematics.getPosition(state);
  endEffectorPositionsTrajectories.push_back(startEndEffectorPositions);

  const std::vector<vector3_t> startEndEffectorVelocities = 
    forwardKinematics.getLinearVelocity(state, input);
  endEffectorVelocitiesTrajectories.push_back(startEndEffectorVelocities);

  for(size_t i = 1; i < referenceSize; ++i)
  {
    currentBasePose += currentBaseVelocity * deltaTime;
    currentJointPositions += currentJointVelocities * deltaTime;

    currentBaseVelocity = vector6_t::Random();
    currentJointVelocities = vector_t::Random(ACTUATED_DOF_NUM);

    state = vector_t::Zero(modelInfo.stateDim);
    input = vector_t::Zero(modelInfo.inputDim);

    floating_base_model::access_helper_functions::
      getBasePose(state, modelInfo) = currentBasePose;
    floating_base_model::access_helper_functions::
      getBaseVelocity(state, modelInfo) = currentBaseVelocity;

    // Give only data of base trajectory
    trajectory.stateTrajectory[i] = state;
    trajectory.inputTrajectory[i] = input;
    
    floating_base_model::access_helper_functions::
      getJointPositions(state, modelInfo) = currentJointPositions;
    
    floating_base_model::access_helper_functions::
      getJointVelocities(input, modelInfo) = currentJointVelocities;

    trajectory.timeTrajectory[i] = initTime + i * deltaTime;
    trajectoryTrue.timeTrajectory[i] = initTime + i * deltaTime;

    trajectoryTrue.stateTrajectory[i] = state;
    trajectoryTrue.inputTrajectory[i] = input;

    const std::vector<vector3_t> endEffectorPositions = forwardKinematics.getPosition(state);
    endEffectorPositionsTrajectories.push_back(endEffectorPositions);

    const std::vector<vector3_t> endEffectorVelocities = 
      forwardKinematics.getLinearVelocity(state, input);
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
  
  planner.updateTrajectory(currentState, trajectory, 
    endEffectorPositionsTrajectories, 
    endEffectorVelocitiesTrajectories);

  for(size_t i = 0; i < referenceSize; ++i)
  {
    EXPECT_TRUE(std::abs(trajectory.timeTrajectory[i] - 
      trajectoryTrue.timeTrajectory[i]) < 1e-9);
    
    std::cerr << (trajectory.stateTrajectory[i] - 
      trajectoryTrue.stateTrajectory[i]).norm() << std::endl;
    EXPECT_TRUE((trajectory.stateTrajectory[i] - 
      trajectoryTrue.stateTrajectory[i]).norm() < stateTolerance);
    std::cerr << (trajectory.inputTrajectory[i] - 
      trajectoryTrue.inputTrajectory[i]).norm() << std::endl;
    EXPECT_TRUE((trajectory.inputTrajectory[i] - 
      trajectoryTrue.inputTrajectory[i]).norm() < stateTolerance);
  }
  
}
