#include <legged_locomotion_mpc/trajectory_planners/ContactForceWrenchTrajectoryPlanner.hpp>

#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {
    using namespace ocs2;
    using namespace floating_base_model;
    using namespace floating_base_model::access_helper_functions;

    ContactForceWrenchTrajectoryPlanner::ContactForceWrenchTrajectoryPlanner(
      FloatingBaseModelInfo modelInfo): modelInfo_(std::move(modelInfo)) {}
      
    void ContactForceWrenchTrajectoryPlanner::updateTargetTrajectory(
      const ModeSchedule& modeSchedule, TargetTrajectories& targetTrajectories)
    {
      const size_t referenceSize = targetTrajectories.timeTrajectory.size();
      const auto& timeTrajectory = targetTrajectories.timeTrajectory;
      auto& inputTrajectory = targetTrajectories.inputTrajectory;

      const auto& eventTimes = modeSchedule.eventTimes;
      const auto& modeSequence = modeSchedule.modeSequence;

      for(size_t i = 0; i < referenceSize; ++i)
      {
        vector_t& currentInput = inputTrajectory[i];

        const size_t currentIndex = utils::findIndexInTimeArray(eventTimes, 
          timeTrajectory[i]);

        const contact_flags_t currentFlags = locomotion::modeNumber2ContactFlags(
          modeSequence[currentIndex]);

        utils::weightCompensatingAppendInput(currentInput, modelInfo_, currentFlags);
      } 
    }
  } // namespace planners
} // namespace legged_locomotion_mp