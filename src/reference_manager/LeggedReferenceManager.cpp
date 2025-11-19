#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace locomotion;
  using namespace planners;
  using namespace terrain_model;

  LeggedReferenceManager::LeggedReferenceManager(LeggedReferenceManager::Settings settings,
    std::shared_ptr<locomotion::GaitPlanner> gaitPlannerPtr,
    std::shared_ptr<locomotion::SwingTrajectoryPlanner> swingTrajectoryPtr,
    std::shared_ptr<planners::BaseTrajectoryPlanner> baseTrajectoryPtr,
    std::shared_ptr<planners::JointTrajectoryPlanner> jointTrajectoryPtr,
    std::shared_ptr<planners::ContactForceWrenchTrajectoryPlanner> forceTrajectoryPtr):
      ReferenceManager(TargetTrajectories(), ModeSchedule()),
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
      bufferedTerrainModel_(nullptr) {}

  void LeggedReferenceManager::initalize(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, 
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
  }

  const contact_flags_t& LeggedReferenceManager::getContactFlags(ocs2::scalar_t time) const
  {
    return contact_flags_t(this->getModeSchedule().modeAtTime(time));
  }

  const TerrainModel& LeggedReferenceManager::getTerrainModel() const
  {
    return *currentTerrainModel_.get();
  }

  void LeggedReferenceManager::preSolverRun(scalar_t initTime, scalar_t finalTime, 
    const vector_t& initState)
  {
    newTrajectories_.wait();

    ReferenceManager::preSolverRun(initTime, finalTime, initState);

    // Get copy of current terrain for getter, buffered value might be changed in parallel task
    currentTerrainModel_ = std::unique_ptr<TerrainModel>(bufferedTerrainModel_.get().clone());

    newTrajectories_ = std::async(std::launch::async, [this, initTime, finalTime]()
      {return generateNewTargetTrajectories(initTime, finalTime);});
  }

  void LeggedReferenceManager::generateNewTargetTrajectories(
    ocs2::scalar_t initTime, ocs2::scalar_t finalTime)
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

    using EndEffectorTrajectories = std::vector<std::vector<vector3_t>>;

    const EndEffectorTrajectories newEndEffectorPositions = 
      swingTrajectoryPtr_->getEndEffectorPositionTrajectories(newTrajectory.timeTrajectory);
    
    const EndEffectorTrajectories newEndEffectorVelocities = 
      swingTrajectoryPtr_->getEndEffectorVelocityTrajectories(newTrajectory.timeTrajectory);

    jointTrajectoryPtr_->updateTrajectory(currentState, newTrajectory, 
      newEndEffectorPositions, newEndEffectorVelocities);
    
    TargetTrajectories subsampledTrajectory = utils::subsampleReferenceTrajectory(
      newTrajectory, initTime, finalTime, settings_.maximumReferenceSampleInterval);

    const std::vector<contact_flags_t> contactTrajectory = 
      gaitPlannerPtr_->getContactFlagsAtTimes(subsampledTrajectory.timeTrajectory);

    forceTrajectoryPtr_->updateTargetTrajectory(contactTrajectory, subsampledTrajectory);

    setTargetTrajectories(std::move(subsampledTrajectory));
    setModeSchedule(std::move(newModeSchedule));
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
