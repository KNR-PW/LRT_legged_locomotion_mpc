#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>

#include <algorithm>
#include <limits>

#include <ocs2_core/misc/Lookup.h>


namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;
    using namespace ocs2::legged_robot;

    GaitPlanner::GaitPlanner(const GaitStaticParameters& staticParams,
      const GaitDynamicParameters initDynamicParams,
      const ModeSequenceTemplate& initModeSequenceTemplate,
      scalar_t initPhase):
        publicStaticParams_(staticParams),
        publicDynamicParams_(initDynamicParams),
        modeSequenceTemplate_(initModeSequenceTemplate)
    {
      currentPhase_ = initPhase;

      modeSchedule_.clear();

      // First mode is STAND
      size_t standMode = (0x01 << (staticParams.endEffectorNumber)) - 1;
      modeSchedule_.modeSequence.push_back(standMode);
      
      insertModeSequenceTemplate(0.0, modeSequenceTemplate_.switchingTimes.back(),
        modeSequenceTemplate_);
    }

    void GaitPlanner::setModeSchedule(const ModeSchedule &modeSchedule)
    { 
      modeSchedule_ = modeSchedule; 
    }

    ModeSchedule GaitPlanner::getModeSchedule(scalar_t lowerBoundTime,
      scalar_t upperBoundTime) 
    {
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(),
       lowerBoundTime) - eventTimes.begin();

      if(index > 0) 
      {
        // Update current phase value
        currentPhase_ += (eventTimes[index - 1] - eventTimes[0]) / publicDynamicParams_.steppingFrequency;
        // delete the old logic from index and set the default start phase to stance
        eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index - 1);
        // keep the one before the last to make it stance
        modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);
      }

      // Start tiling at time
      const auto tilingStartTime = eventTimes.empty() ? lowerBoundTime : eventTimes.back();

      // delete the last default stance phase
      eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
      modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

      // tile the template logic
      tileModeSequenceTemplate(tilingStartTime, upperBoundTime);
      return modeSchedule_;
    }

    void GaitPlanner::updateCurrentContacts(scalar_t time, contact_flags_t currentContacts)
    {
      const auto& eventTimes = modeSchedule_.eventTimes;
      auto& modeSequence = modeSchedule_.modeSequence;
      const size_t realMode = contactFlags2ModeNumber(currentContacts);
      const size_t plannedModeIndex = lookup::findIndexInTimeArray(eventTimes, time);
      const scalar_t plannedContactTime = eventTimes[plannedModeIndex];
      const size_t plannedMode = modeSequence[plannedModeIndex];

      if(realMode == plannedMode) return; // As planned, early return

      /* Not as planned, generating new state */
      scalar_t currentContactState = currentPhase_ + 
        (time - eventTimes.front()) / publicDynamicParams_.steppingFrequency;
      
      std::vector<size_t> wrongContactIndexes;
      contact_flags_t plannedFlags = modeNumber2ContactFlags(plannedMode);

      for(size_t i = 0; i < MAX_LEG_NUMBER; ++i)
      {
        if(plannedFlags[i] != currentContacts[i])
        {
          // Change end effector state to STANCE in next mode
          modeSequence[plannedModeIndex + 1] = setContactFlag(plannedMode, i, 1);
        }
      }
      // Update mode to real one
      modeSequence[plannedModeIndex] = realMode;
    }

    void GaitPlanner::updateWalkingGait(scalar_t startTime,
      scalar_t finalTime, 
      const GaitDynamicParameters& dynamicParams)
    {
      const auto &eventTimes = modeSchedule_.eventTimes;

      // find the index on which the new gait should be added
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(),
       startTime) - eventTimes.begin();

      scalar_t startingPhase = currentPhase_;

      // Get starting phase for new template
      if (index < eventTimes.size()) 
      {
        startingPhase += (startTime - eventTimes.front()) / publicDynamicParams_.steppingFrequency;
      }
      else
      {
        startingPhase += (eventTimes.back() - eventTimes.front()) / publicDynamicParams_.steppingFrequency;
      }

      // Update dynamic parameters
      publicDynamicParams_ = dynamicParams;

      ModeSequenceTemplate newModeSequenceTemplate = getDynamicModeSequenceTemplate(startingPhase,
        finalTime - startTime, publicStaticParams_, publicDynamicParams_);

      insertModeSequenceTemplate(startTime, finalTime, newModeSequenceTemplate);
    }

    const GaitStaticParameters& GaitPlanner::getStaticParameters()
    {
      return publicStaticParams_;
    }

    const GaitDynamicParameters& GaitPlanner::getDynamicParameters()
    {
      return publicDynamicParams_;
    }

    void GaitPlanner::insertModeSequenceTemplate(ocs2::scalar_t startTime,
      ocs2::scalar_t finalTime,
      const ocs2::legged_robot::ModeSequenceTemplate& modeSequenceTemplate)
    {
      modeSequenceTemplate_ = modeSequenceTemplate;
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;

      // find the index on which the new gait should be added
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(),
       startTime) - eventTimes.begin();

      // delete the old logic from the index
      if (index < eventTimes.size()) 
      {
        eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
        modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
      }

      // tile the mode sequence template from startTime to finalTime.
      tileModeSequenceTemplate(startTime, finalTime);
    }

    void GaitPlanner::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime)
    {
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      const auto &templateTimes = modeSequenceTemplate_.switchingTimes;
      const auto &templateModeSequence = modeSequenceTemplate_.modeSequence;
      const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();

      // If no template subsystem is defined, the last subsystem should continue for ever
      if (numTemplateSubsystems == 0) 
      {
        return;
      }

      if (!eventTimes.empty() && startTime <= eventTimes.back()) 
      {
        throw std::runtime_error(
          "The initial time for template-tiling is not greater than the last event time.");
      }

      // add a initial time
      eventTimes.push_back(startTime);

      // concatenate from index
      while (eventTimes.back() < finalTime) 
      {
        for (size_t i = 0; i < templateModeSequence.size(); i++) 
        {
          modeSequence.push_back(templateModeSequence[i]);
          scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
          eventTimes.push_back(eventTimes.back() + deltaTime);
        } // end of i loop
      } // end of while loop

       // default final phase
      modeSequence.push_back(ModeNumber::STANCE);
    }
  }
}