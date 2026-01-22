#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>

#include <limits>

#include <ocs2_core/misc/LinearInterpolation.h>

#include <legged_locomotion_mpc/common/Utils.hpp>

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
    locomotion::GaitPlanner& gaitPlanner,
    locomotion::SwingTrajectoryPlanner& swingTrajectory,
    planners::BaseTrajectoryPlanner& baseTrajectory,
    planners::JointTrajectoryPlanner& jointTrajectory,
    planners::ContactForceWrenchTrajectoryPlanner& forceTrajectory):
      ReferenceManager(TargetTrajectories(), ModeSchedule()),
      modelInfo_(std::move(modelInfo)),
      settings_(std::move(settings)),
      gaitPlanner_(gaitPlanner), 
      swingTrajectory_(swingTrajectory),
      baseTrajectory_(baseTrajectory), 
      jointTrajectory_(jointTrajectory),
      forceTrajectory_(forceTrajectory),
      currentState_(state_vector_t()),
      currentContactFlags_(contact_flags_t()),
      currentGaitParameters_(GaitDynamicParameters()),
      currentCommand_(BaseTrajectoryPlanner::BaseReferenceCommand()),
      bufferedTerrainModel_(nullptr),
      referenceTrajectories_(EndEffectorTrajectories()),
      footConstraintTrajectories_(FootTangentialConstraintTrajectories()) {}

  void LeggedReferenceManager::initialize(scalar_t initTime, scalar_t finalTime, 
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

    // Get reference trajectories and constraints
    referenceTrajectories_.updateFromBuffer();
    footConstraintTrajectories_.updateFromBuffer();

    // Get initialized target trajectory and mode schedule 
    ReferenceManager::preSolverRun(initTime, finalTime, 
      vector_t::Zero(modelInfo_.stateDim));
  }

  const contact_flags_t LeggedReferenceManager::getContactFlags(scalar_t time) const
  {
    const size_t index = utils::findIndexInTimeArray(getModeSchedule().eventTimes, time);
    return modeNumber2ContactFlags(getModeSchedule().modeSequence[index]);
  }

  const TerrainModel& LeggedReferenceManager::getTerrainModel() const
  {
    return *currentTerrainModel_.get();
  }

  EndEffectorTrajectoriesPoint LeggedReferenceManager::getEndEffectorTrajectoryPoint(
    scalar_t time) const
  {
    const auto& times = getTargetTrajectories().timeTrajectory;
    const auto indexAlpha = LinearInterpolation::timeSegment(time, times);
    size_t index = indexAlpha.first;
    const scalar_t alpha = indexAlpha.second;

    // Get previous time index if value is between (times[index - 1], times[index])
    if(index != 0 && (times[index] - time) > std::numeric_limits<scalar_t>::min())
    {
      index -= 1;
    }

    const scalar_t one_minus_alpha = 1.0 - alpha;

    const size_t numEndEffectors = modelInfo_.numThreeDofContacts + 
      modelInfo_.numSixDofContacts;

    EndEffectorTrajectoriesPoint point;

    point.positions.reserve(numEndEffectors);
    point.velocities.reserve(numEndEffectors);
    point.clearances.reserve(numEndEffectors);

    const EndEffectorTrajectories& referenceTrajectories = referenceTrajectories_.get();
    
    const auto& lhsPositions = referenceTrajectories.positions[index];
    const auto& rhsPositions = referenceTrajectories.positions[index + 1];

    const auto& lhsVelocities = referenceTrajectories.velocities[index];
    const auto& rhsVelocities = referenceTrajectories.velocities[index + 1];

    const auto& lhsClearances = referenceTrajectories.clearances[index];
    const auto& rhsClearances = referenceTrajectories.clearances[index + 1];

    const auto& lhsNormals = referenceTrajectories.surfaceNormals[index];
    const auto& rhsNormals = referenceTrajectories.surfaceNormals[index + 1];
    
    for(size_t i = 0; i < numEndEffectors; ++i)
    {
      const vector3_t& lhsPosition = lhsPositions[i];
      const vector3_t& rhsPosition = rhsPositions[i];

      const vector3_t& lhsVelocity = lhsVelocities[i];
      const vector3_t& rhsVelocity = rhsVelocities[i];

      const scalar_t lhsClearance = lhsClearances[i];
      const scalar_t rhsClearance = rhsClearances[i];

      const vector3_t& lhsNormal = lhsNormals[i];
      const vector3_t& rhsNormal = rhsNormals[i];

      point.positions.emplace_back(alpha * lhsPosition + one_minus_alpha * rhsPosition);
      point.velocities.emplace_back(alpha * lhsVelocity + one_minus_alpha * rhsVelocity);
      point.clearances.emplace_back(alpha * lhsClearance + one_minus_alpha * rhsClearance);
      point.surfaceNormals.emplace_back(alpha * lhsNormal + one_minus_alpha * rhsNormal);
    }
    return point;
  }

  const std::vector<FootTangentialConstraintMatrix>& LeggedReferenceManager::getEndEffectorConstraintMatrixes(
    ocs2::scalar_t time) const
  {
    const auto& footTangentialConstraintMatrixes = footConstraintTrajectories_.get();
    const auto& times = footTangentialConstraintMatrixes.times;
    const size_t index = utils::findIndexInTimeArray(times, time);
    if(index > times.size()) return footTangentialConstraintMatrixes.constraints.back();
    return footTangentialConstraintMatrixes.constraints[index];
  }

  void LeggedReferenceManager::preSolverRun(scalar_t initTime, scalar_t finalTime, 
    const vector_t& initState)
  {
    if(newTrajectories_.wait_for(std::chrono::seconds(0)) == std::future_status::timeout)
    {
      return;
    }

    ReferenceManager::preSolverRun(initTime, finalTime, initState);

    // Get reference trajectories and constraints
    referenceTrajectories_.updateFromBuffer();
    footConstraintTrajectories_.updateFromBuffer();

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
      gaitPlanner_.updateDynamicParameters(initTime, currentGaitParameters);
    }

    // No new contact flag -> no update
    if(currentContactFlags_.updateFromBuffer())
    {
      const contact_flags_t& currentContactFlags = currentContactFlags_.get();
      gaitPlanner_.updateCurrentContacts(initTime, currentContactFlags);
    }

    const ModeSchedule newModeSchedule = gaitPlanner_.getModeSchedule(initTime, 
      finalTime);

    // No new terrain model -> no update
    if(bufferedTerrainModel_.updateFromBuffer())
    {
      const TerrainModel& currentTerrainModel = bufferedTerrainModel_.get();

      baseTrajectory_.updateTerrain(currentTerrainModel);
    
      swingTrajectory_.updateTerrain(currentTerrainModel);
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

    baseTrajectory_.updateTargetTrajectory(initTime, finalTime, currentCommand, 
      currentState, newTrajectory);

    swingTrajectory_.updateSwingMotions(initTime, finalTime, currentState, 
      newTrajectory, newModeSchedule);

    EndEffectorTrajectories endEffectorTrajectories = 
      swingTrajectory_.getEndEffectorTrajectories(newTrajectory.timeTrajectory);
    
    jointTrajectory_.updateTrajectory(currentState, newTrajectory, 
      endEffectorTrajectories.positions, endEffectorTrajectories.velocities);

    // If trajectory was subsampled, get new reference for end effectors
    if(settings_.maximumReferenceSampleInterval < baseTrajectory_.getStaticSettings().deltaTime)
    {
      TargetTrajectories subsampledTrajectory = utils::subsampleReferenceTrajectory(
        newTrajectory, initTime, finalTime, settings_.maximumReferenceSampleInterval);

      const std::vector<contact_flags_t> contactTrajectory = 
        gaitPlanner_.getContactFlagsAtTimes(subsampledTrajectory.timeTrajectory);

      forceTrajectory_.updateTargetTrajectory(contactTrajectory, subsampledTrajectory);

      endEffectorTrajectories = swingTrajectory_.getEndEffectorTrajectories(
        subsampledTrajectory.timeTrajectory);

      const FootTangentialConstraintTrajectories footConstraintTrajectories = 
        swingTrajectory_.getFootTangentialConstraintTrajectories(newModeSchedule, 
        subsampledTrajectory.timeTrajectory);

      setTargetTrajectories(std::move(subsampledTrajectory));
      footConstraintTrajectories_.setBuffer(std::move(footConstraintTrajectories));
    }
    else
    {
      const std::vector<contact_flags_t> contactTrajectory = 
        gaitPlanner_.getContactFlagsAtTimes(newTrajectory.timeTrajectory);

      forceTrajectory_.updateTargetTrajectory(contactTrajectory, newTrajectory);

      const FootTangentialConstraintTrajectories footConstraintTrajectories = 
        swingTrajectory_.getFootTangentialConstraintTrajectories(newModeSchedule, 
        newTrajectory.timeTrajectory);

      setTargetTrajectories(std::move(newTrajectory));
      footConstraintTrajectories_.setBuffer(std::move(footConstraintTrajectories));
    }
    referenceTrajectories_.setBuffer(std::move(endEffectorTrajectories));
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
