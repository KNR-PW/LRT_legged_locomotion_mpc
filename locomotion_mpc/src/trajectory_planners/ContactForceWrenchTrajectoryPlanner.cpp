#include <legged_locomotion_mpc/trajectory_planners/ContactForceWrenchTrajectoryPlanner.hpp>

#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {
    using namespace ocs2;
    using namespace floating_base_model;
    using namespace access_helper_functions;

    ContactForceWrenchTrajectoryPlanner::ContactForceWrenchTrajectoryPlanner(
      const PinocchioWeightCompensator& weightCompensator): 
      weightCompensator_(weightCompensator) {}
      
    void ContactForceWrenchTrajectoryPlanner::updateTargetTrajectory(
      const ModeSchedule& modeSchedule, TargetTrajectories& targetTrajectories)
    {
      const auto& info = weightCompensator_.getInfo();

      const size_t forceWrenchSize = 3 * info.numThreeDofContacts + 6 * info.numSixDofContacts;

      const size_t referenceSize = targetTrajectories.timeTrajectory.size();
      const auto& timeTrajectory = targetTrajectories.timeTrajectory;
      const auto& stateTrajectory = targetTrajectories.stateTrajectory;
      auto& inputTrajectory = targetTrajectories.inputTrajectory;

      const auto& eventTimes = modeSchedule.eventTimes;
      const auto& modeSequence = modeSchedule.modeSequence;

      const vector_t& startState = stateTrajectory[0];
      vector_t& startInput = inputTrajectory[0];

      const size_t startIndex = utils::findIndexInTimeArray(eventTimes, 
        timeTrajectory[0]);

      const contact_flags_t startFlags = locomotion::modeNumber2ContactFlags(
        modeSequence[startIndex]);

      weightCompensator_.appendInput(startState, startInput, startFlags);

      size_t previousIndex = startIndex;

      for(size_t i = 1; i < referenceSize; ++i)
      {
        const vector_t& currentState = stateTrajectory[i];
        vector_t& currentInput = inputTrajectory[i];

        const size_t currentIndex = utils::findIndexInTimeArray(eventTimes, 
          timeTrajectory[i]);

        if(currentIndex == previousIndex && info.numSixDofContacts == 0)
        {
          // Copy previous compensation forces (only if there are no 6 DoF end effectors)
          const vector_t& previousInput = inputTrajectory[i - 1];
          currentInput.block(0, 0, forceWrenchSize, 1) = previousInput.block(0, 0, 
            forceWrenchSize, 1);
        }
        else
        {
          // Get new weight compensation forces and wrenches
          const contact_flags_t currentFlags = locomotion::modeNumber2ContactFlags(
          modeSequence[currentIndex]);

          weightCompensator_.appendInput(currentState, currentInput, currentFlags);
        }

        previousIndex = currentIndex;
      } 
    }
  } // namespace planners
} // namespace legged_locomotion_mp