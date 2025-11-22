#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

#include <ocs2_core/misc/Lookup.h>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace locomotion;
  using namespace planners;
  using namespace terrain_model;
  using namespace floating_base_model;
  using EndEffectorTrajectories = legged_locomotion_mpc::locomotion::SwingTrajectoryPlanner::EndEffectorTrajectories;
  using EndEffectorTrajectoriesPoint = legged_locomotion_mpc::locomotion::SwingTrajectoryPlanner::EndEffectorTrajectoriesPoint;

  LeggedReferenceManager::LeggedReferenceManager(
    FloatingBaseModelInfo modelInfo,
    LeggedReferenceManager::Settings settings,
    std::shared_ptr<locomotion::GaitPlanner> gaitPlannerPtr,
    std::shared_ptr<locomotion::SwingTrajectoryPlanner> swingTrajectoryPtr,
    std::shared_ptr<planners::BaseTrajectoryPlanner> baseTrajectoryPtr,
    std::shared_ptr<planners::JointTrajectoryPlanner> jointTrajectoryPtr,
    std::shared_ptr<planners::ContactForceWrenchTrajectoryPlanner> forceTrajectoryPtr):
      ReferenceManager(TargetTrajectories(), ModeSchedule()),
      modelInfo_(std::move(modelInfo)),
      settings_(std::move(settings)),
      gaitPlannerPtr_(std::move(gaitPlannerPtr)), 
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      baseTrajectoryPtr_(std::move(baseTrajectoryPtr)), 
      jointTrajectoryPtr_(std::move(jointTrajectoryPtr)),
      forceTrajectoryPtr_(std::move(forceTrajectoryPtr)),
      currentState_(state_vector_t()),
      currentContactFlags_(contact_flags_t()),
      currentGaitParameters_(GaitDynamicParameters()),
      currentCommand_(BaseTrajectoryPlanner::BaseReferenceCommand()),
      bufferedTerrainModel_(nullptr),
      referenceTrajectories_(EndEffectorTrajectories()) {}

  void LeggedReferenceManager::initalize(scalar_t initTime, scalar_t finalTime, 
    const state_vector_t& currenState, const contact_flags_t& currentContactFlags,
    locomotion::GaitDynamicParameters&& currentGaitParameters,
    std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel)
  {
    updateState(currenState);
    updateContactFlags(currentContactFlags);
    updateGaitParemeters(std::move(currentGaitParameters));

    // Get copy of current terrain for getter, buffered value might be changed in parallel task
    currentTerrainModel_ = std::unique_ptr<TerrainModel>(currentTerrainModel->clone());

    updateTerrainModel(std::move(currentTerrainModel));
    
    newTrajectories_ = std::async(std::launch::async, [this, initTime, finalTime]()
      {return generateNewTargetTrajectories(initTime, finalTime);});
    newTrajectories_.wait();

    referenceTrajectories_.updateFromBuffer();
  }

  const contact_flags_t& LeggedReferenceManager::getContactFlags(scalar_t time) const
  {
    return contact_flags_t(this->getModeSchedule().modeAtTime(time));
  }

  const TerrainModel& LeggedReferenceManager::getTerrainModel() const
  {
    return *currentTerrainModel_.get();
  }

  EndEffectorTrajectoriesPoint LeggedReferenceManager::getEndEffectorTrajectoryPoint(
    scalar_t time) const
  {
    const auto timeIndex = lookup::findIndexInTimeArray(
      getTargetTrajectories().timeTrajectory, time);

    EndEffectorTrajectoriesPoint point;

    const EndEffectorTrajectories& referenceTrajectories = referenceTrajectories_.get();
    
    point.positions = referenceTrajectories.positions[timeIndex];
    point.velocities = referenceTrajectories.velocities[timeIndex];
    point.clearances = referenceTrajectories.clearances[timeIndex];
  }

  void LeggedReferenceManager::preSolverRun(scalar_t initTime, scalar_t finalTime, 
    const vector_t& initState)
  {
    if(newTrajectories_.wait_for(std::chrono::seconds(0)) == std::future_status::timeout)
    {
      return;
    }

    ReferenceManager::preSolverRun(initTime, finalTime, initState);

    referenceTrajectories_.updateFromBuffer();

    // Get copy of current terrain for getter, active value might be changed in parallel task
    currentTerrainModel_ = std::unique_ptr<TerrainModel>(bufferedTerrainModel_.get().clone());

    newTrajectories_ = std::async(std::launch::async, [this, initTime, finalTime]()
      {return generateNewTargetTrajectories(initTime, finalTime);});
  }

  void LeggedReferenceManager::generateNewTargetTrajectories(
    scalar_t initTime, scalar_t finalTime)
  {
    currentState_.updateFromBuffer();
    const state_vector_t& currentState = currentState_.get();

    // no new gait parameters -> no update
    if(currentGaitParameters_.updateFromBuffer())
    {
      const GaitDynamicParameters& currentGaitParameters = currentGaitParameters_.get();
      gaitPlannerPtr_->updateDynamicParameters(initTime, currentGaitParameters);
    }

    // No new contact flag -> no update
    if(currentContactFlags_.updateFromBuffer());
    {
      const contact_flags_t& currentContactFlags = currentContactFlags_.get();
      gaitPlannerPtr_->updateCurrentContacts(initTime, currentContactFlags);
    }

    const ModeSchedule newModeSchedule = gaitPlannerPtr_->getModeSchedule(initTime, 
      finalTime);

    // No new terrain model -> no update
    if(bufferedTerrainModel_.updateFromBuffer());
    {
      const TerrainModel& currentTerrainModel = bufferedTerrainModel_.get();

      baseTrajectoryPtr_->updateTerrain(currentTerrainModel);
    
      swingTrajectoryPtr_->updateTerrain(currentTerrainModel);
    }
    
    // No new command -> stay in place
    BaseTrajectoryPlanner::BaseReferenceCommand currentCommand;
    if(currentCommand_.updateFromBuffer())
    {
      currentCommand = currentCommand_.get();
    }
    else
    {
      currentCommand.baseHeadingVelocity = 0.0;
      currentCommand.baseLateralVelocity = 0.0;
      currentCommand.baseVerticalVelocity = 0.0;
      currentCommand.yawRate = 0.0;
    }

    TargetTrajectories newTrajectory;

    baseTrajectoryPtr_->updateTargetTrajectory(initTime, finalTime, currentCommand, 
      currentState, newTrajectory);
    
    swingTrajectoryPtr_->updateSwingMotions(initTime, finalTime, currentState, 
      newTrajectory, newModeSchedule);

    const EndEffectorTrajectories endEffectorTrajectories = 
      swingTrajectoryPtr_->getEndEffectorTrajectories(newTrajectory.timeTrajectory);
    
    jointTrajectoryPtr_->updateTrajectory(currentState, newTrajectory, 
      endEffectorTrajectories.positions, endEffectorTrajectories.velocities);
    
    TargetTrajectories subsampledTrajectory = utils::subsampleReferenceTrajectory(
      newTrajectory, initTime, finalTime, settings_.maximumReferenceSampleInterval);

    const std::vector<contact_flags_t> contactTrajectory = 
      gaitPlannerPtr_->getContactFlagsAtTimes(subsampledTrajectory.timeTrajectory);

    forceTrajectoryPtr_->updateTargetTrajectory(contactTrajectory, subsampledTrajectory);

    setTargetTrajectories(std::move(subsampledTrajectory));
    setModeSchedule(std::move(newModeSchedule));
    referenceTrajectories_.setBuffer(std::move(endEffectorTrajectories));
  }

  void LeggedReferenceManager::updateState(const state_vector_t& currenState)
  {
    currentState_.setBuffer(currenState);
  }

  void LeggedReferenceManager::updateContactFlags(
    const contact_flags_t& currentContactFlags)
  {
    currentContactFlags_.setBuffer(currentContactFlags);
  }

  void LeggedReferenceManager::updateGaitParemeters(
    GaitDynamicParameters&& currentGaitParameters)
  {
    currentGaitParameters_.setBuffer(std::move(currentGaitParameters));
  }
  
  void LeggedReferenceManager::updateTerrainModel(
    std::unique_ptr<TerrainModel> currentTerrainModel)
  {
    bufferedTerrainModel_.setBuffer(std::move(currentTerrainModel));
  }

  LeggedReferenceManager::~LeggedReferenceManager()
  {
    newTrajectories_.wait();
  }
} // namespace legged_locomotion_mpc
