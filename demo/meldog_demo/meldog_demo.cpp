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
  const std::string configFilePath = package_path::getPath() + "/demo/meldog_demo/config/";
  const std::string taskFilePath = configFilePath + "task.info";
  const std::string modelFilePath = configFilePath + "legged_model.info";
  const std::string urdfFilePath = meldogWithBaseLinkUrdfFile;

  const scalar_t initTime = 0.0;

  const vector3_t terrainEulerZyx{0.0, 0.0, 0.0};
  const matrix3_t terrainRotation = getRotationMatrixFromZyxEulerAngles(terrainEulerZyx); 

  TerrainPlane terrainPlane(vector3_t::Zero(), terrainRotation.transpose());

  std::unique_ptr<TerrainModel> terrainModel = 
    std::make_unique<PlanarTerrainModel>(terrainPlane);

  ocs2::PinocchioInterface interface = createPinocchioInterfaceFromUrdfFile(urdfFilePath, baseLink);
  const FloatingBaseModelInfo modelInfo = createFloatingBaseModelInfo(interface, meldog3DofContactNames, meldog6DofContactNames);

  const vector2_t initPositionXY{0.0, 0.0};

  vector3_t initialBasePosition(initPositionXY.x(), initPositionXY.y(),
    terrainModel->getSmoothedPositon(initPositionXY).z());

  BaseTrajectoryPlanner::StaticSettings baseSettings = 
    loadBasePlannerStaticSettings(modelFilePath);
  
  initialBasePosition.z() += baseSettings.initialBaseHeight / terrainPlane.getSurfaceNormalInWorld().z();

  vector_t initialState = vector_t::Zero(Meldog::STATE_DIM);
  initialState.block<3,1>(6, 0) = initialBasePosition;
  initialState.block<3,1>(9, 0) = terrainEulerZyx;
  access_helper_functions::getJointPositions(initialState, modelInfo) << 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326, 0, -0.785398163, 1.570796326;

  LeggedInterface leggedInterface(initTime, initialState, 
    std::move(terrainModel), taskFilePath, modelFilePath, urdfFilePath);

  auto& referenceManager = leggedInterface.getLeggedReferenceManager();
  
   /* STANDING TROT */
  GaitDynamicParameters dynamicParams;
  dynamicParams.steppingFrequency = 1.0 / 0.7;
  dynamicParams.swingRatio = 3.0 / 7.0;

  const scalar_t offset = 3.5 / 7.0;
  
  dynamicParams.phaseOffsets = {-offset , -offset , 0};

  BaseTrajectoryPlanner::BaseReferenceCommand command;
  command.baseHeadingVelocity = 0.25;
  command.baseLateralVelocity = 0.0;
  command.baseVerticalVelocity = 0.0;
  command.yawRate = 0.0;

  const scalar_t moveTime = 0.5;
  const scalar_t endTime = 5.0;

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

  // Wait for the first policy
  mpcInterface.setCurrentObservation(observation);

  const auto& referenceTrajectory = referenceManager.getTargetTrajectories();

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

  std::cerr << "Main loop!" << std::endl;

  // run MPC till final time
  TargetTrajectories optimalTrajectory;
  ModeSchedule modeSchedule;
  modeSchedule.clear();
  std::vector<PerformanceIndex> performances;
  bool firstChange = true;

  while(observation.time < endTime) 
  {
    std::cout << "Time: " << observation.time << "\n";
    try 
    {
      // run MPC at current observation
      mpcInterface.setCurrentObservation(observation);
      

      if(observation.time > moveTime)
      {
        // referenceManager.updateCommand(command);
        if(firstChange)
        {
          firstChange = false;
          referenceManager.updateGaitParemeters(dynamicParams);
        }
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
    } 
    catch (std::exception& e) 
    {
      std::cout << "MPC failed\n";
      std::cout << e.what() << "\n";
      break;
    }

    // for(size_t i = 0; i < referenceTrajectory.timeTrajectory.size(); ++i)
    // {
    //   std::cerr << "Time: " << referenceTrajectory.timeTrajectory[i] << std::endl;
    //   std::cerr << "Base position: " << referenceTrajectory.stateTrajectory[i].block(6, 0, 3, 1).transpose() << std::endl;
    //   std::cerr << "Joint positions: " << access_helper_functions::getJointPositions(referenceTrajectory.stateTrajectory[i], modelInfo).transpose() << std::endl;
    //   std::cerr << "Joint velocities: " << access_helper_functions::getJointVelocities(referenceTrajectory.inputTrajectory[i], modelInfo).transpose() << std::endl;
    //   std::cerr << "Force: " << access_helper_functions::getContactForces(referenceTrajectory.inputTrajectory[i], 0 , modelInfo).transpose() << std::endl;
    // }
  }

  for(size_t i = 0; i < optimalTrajectory.timeTrajectory.size(); ++i)
  {
    std::cerr << "Time: " << optimalTrajectory.timeTrajectory[i] << std::endl;
    std::cerr << "Base position: " << optimalTrajectory.stateTrajectory[i].block(6, 0, 3, 1).transpose() << std::endl;
    std::cerr << "Joint positions: " << access_helper_functions::getJointPositions(optimalTrajectory.stateTrajectory[i], modelInfo).transpose() << std::endl;
    std::cerr << "Joint velocities: " << access_helper_functions::getJointVelocities(optimalTrajectory.inputTrajectory[i], modelInfo).transpose() << std::endl;
    std::cerr << "Force: " << access_helper_functions::getContactForces(optimalTrajectory.inputTrajectory[i], 0 , modelInfo).transpose() << std::endl;
  }
  std::cerr << modeSchedule << std::endl;

  return 0;
}