// #include "rclcpp/rclcpp.hpp"

#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>

#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/oc_data/PerformanceIndex.h>
#include <ocs2_oc/oc_data/LoopshapingPrimalSolution.h>

#include <terrain_model/core/TerrainPlane.hpp>
#include <terrain_model/planar_model/PlanarTerrainModel.hpp>

#include <floating_base_model/FactoryFunctions.hpp>

#include <legged_locomotion_mpc/robot_interface/LeggedInterface.hpp>

#include <legged_locomotion_mpc/path_management/package_path.h>

#include <legged_locomotion_mpc/visualization/LeggedVisializer.hpp>

#include "../../test/include/definitions.hpp"

using namespace ocs2;
using namespace terrain_model;
using namespace floating_base_model;
using namespace legged_locomotion_mpc;
using namespace legged_locomotion_mpc::utils;
using namespace legged_locomotion_mpc::locomotion;
using namespace legged_locomotion_mpc::planners;

int main(int argc, char* argv[]) 
{
  const std::string configFilePath = package_path::getPath() + "/demo/melman_demo/config/";
  const std::string taskFilePath = configFilePath + "task.info";
  const std::string modelFilePath = configFilePath + "legged_model.info";
  const std::string urdfFilePath = legged_locomotion_mpc::package_path::getPath() + "/test/test_models/melman.urdf";

  const scalar_t initTime = 0.0;

  const vector3_t terrainEulerZyx{0.0, 0.0, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane terrainPlane(vector3_t::Zero(), terrainRotation.transpose());

  std::unique_ptr<TerrainModel> terrainModel = 
    std::make_unique<PlanarTerrainModel>(terrainPlane);

  const std::string melmanBaseLink = "Trunk";
  const std::vector<std::string> melmanEndEffectors6Dof{"Right_Foot", "Left_Foot"};
  std::vector<std::string> melmanEndEffectors3Dof;

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfFilePath, melmanBaseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, melmanEndEffectors3Dof, melmanEndEffectors6Dof);

  std::cerr << modelInfo << std::endl;

  const auto& model = interface.getModel();

  for(size_t i = 0; i < model.names.size(); ++i)
  {
    std::cerr << i << ": " << model.names[i] << std::endl;
  }

  const vector2_t initPositionXY{0.0, 0.0};

  vector3_t initialBasePosition(initPositionXY.x(), initPositionXY.y(),
    terrainModel->getSmoothedPositon(initPositionXY).z());

  BaseTrajectoryPlanner::StaticSettings baseSettings = 
    loadBasePlannerStaticSettings(modelFilePath);
  
  initialBasePosition.z() += baseSettings.initialBaseHeight / terrainPlane.getSurfaceNormalInWorld().z();

  vector_t initialState = vector_t::Zero(30);
  initialState.block<3,1>(6, 0) = initialBasePosition;
  initialState.block<3,1>(9, 0) = terrainEulerZyx;
  access_helper_functions::getJointPositions(initialState, modelInfo) << 0.78539816339, 0, 2 * 0.78539816339, 0, 0, 0.4, -0.8, 0.4, 0, 0.78539816339, 0, 2 * 0.78539816339, 0, 0, 0.4, -0.8, 0.4, 0;
  // access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326;

  LeggedInterface leggedInterface(initTime, initialState, 
    std::move(terrainModel), taskFilePath, modelFilePath, urdfFilePath);

  auto& referenceManager = leggedInterface.getLeggedReferenceManager();
  
  // /* DYNAMIC WALK */

  // GaitDynamicParameters dynamicParams;
  // dynamicParams.swingRatio =  0.3;
  // dynamicParams.steppingFrequency = 1.0;

  // dynamicParams.phaseOffsets = {0.5, 0.2, 0.7};

  /* STANDING TROT */
  
  GaitDynamicParameters firstDynamicParams;
  firstDynamicParams.steppingFrequency = 1.0 / 0.7;
  firstDynamicParams.swingRatio = 3.0 / 7.0;

  const scalar_t firstOffset = 3.5 / 7.0;
  
  firstDynamicParams.phaseOffsets = {-firstOffset};

  /* FLYING TROT */
  GaitDynamicParameters secondDynamicParams;
  secondDynamicParams.swingRatio =  0.33 / 0.6;
  secondDynamicParams.steppingFrequency = 1.0 / 0.6;
  scalar_t secondOffset = secondDynamicParams.swingRatio;

  secondDynamicParams.phaseOffsets = {-secondOffset + 0.03 / 0.6};

  BaseTrajectoryPlanner::BaseReferenceCommand firstCommand;
  firstCommand.baseHeadingVelocity = 0.05;
  firstCommand.baseLateralVelocity = 0.0;
  firstCommand.baseVerticalVelocity = 0.0;
  firstCommand.yawRate = 1 * 0.0314;

  BaseTrajectoryPlanner::BaseReferenceCommand secondCommand;
  secondCommand.baseHeadingVelocity = 0.15;
  secondCommand.baseLateralVelocity = -0.0;
  secondCommand.baseVerticalVelocity = 0.0;
  secondCommand.yawRate = -1 * 0.0314;

  const scalar_t firstGaitTime = 1.0;
  bool firstChange = true;
  const scalar_t secondGaitTime = 8.0;
  bool secondChange = true;

  const scalar_t firstMoveTime = 2.0;
  const scalar_t secondMoveTime = 15.0;
  const scalar_t endTime = 20.0;

  // referenceManager.updateCommand(command);
  // referenceManager.updateGaitParemeters(dynamicParams);
  // referenceManager.preSolverRun(moveTime, endTime, initialState);

  // DDP

  const auto mpcSettings = leggedInterface.mpcSettings();
  const auto ddpSettings = leggedInterface.ddpSettings();

  const auto& optimalProblem = leggedInterface.getOptimalControlProblem();
  const auto& initializer = leggedInterface.getInitializer();

  auto& rollout = leggedInterface.getRollout();

  std::unique_ptr<MPC_BASE> mpcPtr = std::make_unique<GaussNewtonDDP_MPC>(mpcSettings, 
    ddpSettings, rollout, optimalProblem, initializer);

  mpcPtr->getSolverPtr()->setReferenceManager(leggedInterface.getReferenceManagerPtr());

  std::unique_ptr<ocs2::RolloutBase> rolloutPtr(rollout.clone());

  MPC_MRT_Interface mpcInterface(*mpcPtr);

  const size_t standingMode = ((0x01 << (4)) - 1);
  const contact_flags_t standingFlags(standingMode);

  // ====== Execute the scenario ========
  ocs2::SystemObservation observation;
  observation.time = initTime;
  observation.state = initialState;
  observation.input = weightCompensatingInput(modelInfo, standingFlags);

  std::cerr << observation.input.transpose() << std::endl;

  // Wait for the first policy
  mpcInterface.setCurrentObservation(observation);

  // for(size_t i = 0; i < referenceTrajectory.timeTrajectory.size(); ++i)
  // {
  //   std::cerr << "Time: " << referenceTrajectory.timeTrajectory[i] << std::endl;
  //   std::cerr << "Base position: " << referenceTrajectory.stateTrajectory[i].block(6, 0, 3, 1).transpose() << std::endl;
  //   std::cerr << "Joint positions: " << access_helper_functions::getJointPositions(referenceTrajectory.stateTrajectory[i], modelInfo).transpose() << std::endl;
  //   std::cerr << "Joint velocities: " << access_helper_functions::getJointVelocities(referenceTrajectory.inputTrajectory[i], modelInfo).transpose() << std::endl;
  //   std::cerr << "Force: " << access_helper_functions::getContactForces(referenceTrajectory.inputTrajectory[i], 0 , modelInfo).transpose() << std::endl;
  // }
  // return 0;

  std::cerr << "Initialization!" << std::endl;
  while (!mpcInterface.initialPolicyReceived()) 
  {
    mpcInterface.advanceMpc();
  }
  mpcInterface.initRollout(&leggedInterface.getRollout());

  std::cerr << "Main loop!" << std::endl;

  // run MPC till final time
  TargetTrajectories optimalTrajectory;
  ModeSchedule modeSchedule;
  modeSchedule.clear();
  std::vector<PerformanceIndex> performances;

  std::vector<ocs2::SystemObservation> observations;
  std::vector<contact_flags_t> contactFlags;

  std::vector<ocs2::scalar_array_t> optimizedTimeTrajectories;
  std::vector<ocs2::vector_array_t> optimizedStateTrajectories;

  while(observation.time < endTime) 
  {
    std::cout << "Time: " << observation.time << "\n";
    observations.push_back(observation);
    try 
    {
      // run MPC at current observation
      mpcInterface.setCurrentObservation(observation);
      referenceManager.updateObservation(observation);
      

      if(observation.time > firstGaitTime && firstChange)
      {
        firstChange = false;
        referenceManager.updateGaitParemeters(firstDynamicParams);
        // referenceManager.updateCommand(firstCommand);
        // referenceManager.preSolverRun(gaitTime, endTime, observation.state);

        // const auto targetTrajectory = referenceManager.getTargetTrajectories();

        // for(size_t i = 0; i < targetTrajectory.size(); ++i)
        // {
        //   SystemObservation currentObservation;
        //   currentObservation.time = targetTrajectory.timeTrajectory[i];
        //   currentObservation.state = targetTrajectory.stateTrajectory[i];
        //   currentObservation.input = targetTrajectory.inputTrajectory[i];
        //   observations.push_back(currentObservation);

        //   const auto referenceEndEffectorTrajectoryPoint = 
        //     referenceManager.getEndEffectorTrajectoryPoint(currentObservation.time);

        //   referenceEndEffectorTrajectories.push_back(std::move(
        //     referenceEndEffectorTrajectoryPoint));
        // }
        // break;
      }

      if(observation.time > secondGaitTime && secondChange)
      {
        secondChange = false;
        referenceManager.updateGaitParemeters(secondDynamicParams);
      }

      if(observation.time > firstMoveTime)
      {
        referenceManager.updateCommand(firstCommand);
      }

      if(observation.time > secondMoveTime)
      {
        referenceManager.updateCommand(secondCommand);
      }

      const auto& lastTrajectoryState = referenceManager.getTargetTrajectories().stateTrajectory.back();

      std::cerr << "Ostatni euler angle z targetu: ";
      std::cerr << lastTrajectoryState.block(9, 0, 3, 1).transpose() << std::endl;

      std::cerr << "Rzeczywisty euler angle: ";
      std::cerr << observation.state.block(9, 0, 3, 1).transpose() << std::endl;

      if(std::abs(observation.time - 11.25) < 1e-6)
      {
        std::cerr << "Zaczyna sie!" << std::endl;
      }

      mpcInterface.advanceMpc();
      mpcInterface.updatePolicy();

      performances.push_back(mpcInterface.getPerformanceIndices());

      // Evaluate the optimized solution - change to optimal controller
      vector_t tmp;
      mpcInterface.evaluatePolicy(observation.time, observation.state, tmp,
        observation.input, observation.mode);

      
      observation.input = LinearInterpolation::interpolate(
        observation.time, mpcInterface.getPolicy().timeTrajectory_,
        mpcInterface.getPolicy().inputTrajectory_);

      optimizedTimeTrajectories.push_back(mpcInterface.getPolicy().timeTrajectory_);
      optimizedStateTrajectories.push_back(mpcInterface.getPolicy().stateTrajectory_);

      optimalTrajectory.timeTrajectory.push_back(observation.time);
      optimalTrajectory.stateTrajectory.push_back(observation.state);
      optimalTrajectory.inputTrajectory.push_back(observation.input);


      if(modeSchedule.modeSequence.empty()) 
      {
        modeSchedule.modeSequence.push_back(observation.mode);
      } 
      else if(modeSchedule.modeSequence.back() != observation.mode) 
      {
        modeSchedule.modeSequence.push_back(observation.mode);
        modeSchedule.eventTimes.push_back(observation.time);
      }

      // perform a rollout
      scalar_array_t timeTrajectory;
      size_array_t postEventIndicesStock;
      vector_array_t stateTrajectory, inputTrajectory;
      const scalar_t finalTime = observation.time +  1.0 / mpcSettings.mpcDesiredFrequency_;
      auto modeschedule = mpcInterface.getPolicy().modeSchedule_;
      rolloutPtr->run(observation.time, observation.state, finalTime,
                   mpcInterface.getPolicy().controllerPtr_.get(), modeschedule,
                   timeTrajectory, postEventIndicesStock, stateTrajectory,
                   inputTrajectory);

      observation.time = finalTime;
      observation.state = stateTrajectory.back();
      observation.input = inputTrajectory.back();

      const auto targetState = referenceManager.getTargetTrajectories().getDesiredState(finalTime);
      const auto targetInput = referenceManager.getTargetTrajectories().getDesiredInput(finalTime);

      // std::cerr << "Time: " << observation.time << std::endl;
      // std::cerr << "Real base position: " << observation.state.block(6, 0, 3, 1).transpose() << std::endl;
      // std::cerr << "Reference base position: " << targetState.block(6, 0, 3, 1).transpose() << std::endl;
      // std::cerr << "Real base linear velocity: " << observation.state.block(0, 0, 3, 1).transpose() << std::endl;
      // std::cerr << "Reference base linear velocity: " << targetState.block(0, 0, 3, 1).transpose() << std::endl;
      // std::cerr << "Real joint positions: " << access_helper_functions::getJointPositions(observation.state, modelInfo).transpose() << std::endl;
      // std::cerr << "Reference joint positions: " << access_helper_functions::getJointPositions(targetState, modelInfo).transpose() << std::endl;
      // std::cerr << "Real joint velocities: " << access_helper_functions::getJointVelocities(observation.input, modelInfo).transpose() << std::endl;
      // std::cerr << "Reference joint velocities: " << access_helper_functions::getJointVelocities(targetInput, modelInfo).transpose() << std::endl;
      // std::cerr << "Real force: " << access_helper_functions::getContactForces(observation.input, 0 , modelInfo).transpose() << std::endl;
      // std::cerr << "Refrence force: " << access_helper_functions::getContactForces(targetInput, 0 , modelInfo).transpose() << std::endl;
    } 
    catch (std::exception& e) 
    {
      std::cout << "MPC failed\n";
      std::cout << e.what() << "\n";
      break;
    }
  }

  // for(size_t i = 0; i < optimalTrajectory.timeTrajectory.size(); ++i)
  // {
  //   std::cerr << "Time: " << optimalTrajectory.timeTrajectory[i] << std::endl;
  //   std::cerr << "Base position: " << optimalTrajectory.stateTrajectory[i].block(6, 0, 3, 1).transpose() << std::endl;
  //   std::cerr << "Base linear velocity: " << optimalTrajectory.stateTrajectory[i].block(0, 0, 3, 1).transpose() << std::endl;
  //   std::cerr << "Joint positions: " << access_helper_functions::getJointPositions(optimalTrajectory.stateTrajectory[i], modelInfo).transpose() << std::endl;
  //   std::cerr << "Joint velocities: " << access_helper_functions::getJointVelocities(optimalTrajectory.inputTrajectory[i], modelInfo).transpose() << std::endl;
  //   std::cerr << "Force: " << access_helper_functions::getContactForces(optimalTrajectory.inputTrajectory[i], 0 , modelInfo).transpose() << std::endl;
  // }
  // std::cerr << modeSchedule << std::endl;


  rclcpp::init(argc, argv);

  const auto visualizerSettings = ros::LeggedVisualizer::Settings();
  const auto modelSettings = leggedInterface.modelSettings();
  const auto& forwardKinematics = leggedInterface.forwardKinematics();
  const auto& torqueApproximator = leggedInterface.torqueApproximator();
  const auto& robotModel = leggedInterface.pinocchioInterface().getModel();

  const auto leggedVisualizer = std::make_shared<ros::LeggedVisualizer>(visualizerSettings, 
    modelSettings, modelInfo, robotModel, forwardKinematics, torqueApproximator);

  while(true)
  {
    scalar_t visualizationTime = initTime;
    size_t visualizationIndex = 0;
    while(visualizationTime < observations.back().time)
    {
      const auto currentTimeStamp = leggedVisualizer->now();
      leggedVisualizer->publishObservation(currentTimeStamp, observations[visualizationIndex]);
      if(visualizationIndex < optimizedTimeTrajectories.size())
      {
        leggedVisualizer->publishOptimizedTrajectory(currentTimeStamp, 
          optimizedTimeTrajectories[visualizationIndex], 
          optimizedStateTrajectories[visualizationIndex]);
      }
      visualizationIndex++;
      if(visualizationIndex < observations.size())
      {
        const auto duration = observations[visualizationIndex].time - visualizationTime;
        const auto durationSeconds = std::chrono::duration<scalar_t>(duration);
        const auto durationNanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
          durationSeconds);
        rclcpp::sleep_for(durationNanoseconds);
        visualizationTime = observations[visualizationIndex].time;
      }


    }
  }
  
  rclcpp::shutdown();
  return 0;
}