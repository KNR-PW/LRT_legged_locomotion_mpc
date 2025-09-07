#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>

#include <algorithm>
#include <limits>

#include <ocs2_core/misc/Lookup.h>


namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;

    GaitPlanner::GaitPlanner(const GaitStaticParameters& staticParams,
      const GaitDynamicParameters initDynamicParams,
      const ModeSequenceTemplate& initModeSequenceTemplate,
      scalar_t initPhase,
      scalar_t initTime):
        staticParams_(staticParams),
        modeSequenceTemplate_(initModeSequenceTemplate),
        gaitPhaseController_(initPhase, initTime, staticParams, initDynamicParams)
    {
      modeSchedule_.clear();

      // First mode is STAND
      size_t standMode = (0x01 << (staticParams.endEffectorNumber)) - 1;
      modeSchedule_.modeSequence.push_back(standMode);
      
      insertModeSequenceTemplate(initTime, modeSequenceTemplate_.switchingTimes.back(),
        modeSequenceTemplate_);
    }

    void GaitPlanner::setModeSchedule(const ModeSchedule &modeSchedule)
    { 
      modeSchedule_ = modeSchedule; 
    }

    ModeSchedule GaitPlanner::getModeSchedule(scalar_t startTime,
      scalar_t finalTime) 
    {
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(),
       startTime) - eventTimes.begin();

      if(index > 0) 
      {
        // Update gait phase controller
        gaitPhaseController_.remove(startTime);
        // delete the old logic from index and set the default start phase to stance
        eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index - 1);
        // keep the one before the last to make it stance
        modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);
      }

      // Start tiling at time
      const auto tilingStartTime = eventTimes.empty() ? startTime : eventTimes.back();

      // delete the last default stance phase
      eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
      modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

      // tile the template logic
      tileModeSequenceTemplate(tilingStartTime, finalTime);

      return modeSchedule_;
    }

    GaitFlags GaitPlanner::updateCurrentContacts(scalar_t time, contact_flags_t currentContacts)
    {
      const auto& eventTimes = modeSchedule_.eventTimes;
      auto& modeSequence = modeSchedule_.modeSequence;

      const contact_flags_t plannedFlags = gaitPhaseController_.getContactFlagsAtTime(time);

      if(plannedFlags == currentContacts)
      {
        return GaitFlags::OK; // As planned, early return
      } 

      if(contactFlags2ModeNumber(currentContacts) == 0)
      {
        return GaitFlags::IN_THE_AIR; // Cricital error, robot is in the air!
      }

      return GaitFlags::ERROR;
    }

    void GaitPlanner::updateDynamicParameters(scalar_t time,
      const GaitDynamicParameters& dynamicParams)
    {
      const auto &eventTimes = modeSchedule_.eventTimes;

      // Update gait phase controller
      gaitPhaseController_.update(time, dynamicParams);

      // Get current phase
      scalar_t startingPhase = gaitPhaseController_.getPhasesAtTime(time)[0];

      const scalar_t finalTime = eventTimes.back();

      ModeSequenceTemplate newModeSequenceTemplate = getDynamicModeSequenceTemplate(startingPhase,
        finalTime - time, staticParams_, dynamicParams);

      insertModeSequenceTemplate(time, finalTime, newModeSequenceTemplate);
    }

    const GaitStaticParameters& GaitPlanner::getStaticParameters()
    {
      return staticParams_;
    }

    std::vector<scalar_t> GaitPlanner::getPhasesAtTime(scalar_t time)
    {
      return gaitPhaseController_.getPhasesAtTime(time);
    }

    contact_flags_t GaitPlanner::getContactFlagsAtTime(scalar_t time)
    {
      return gaitPhaseController_.getContactFlagsAtTime(time);
    }

    void GaitPlanner::insertModeSequenceTemplate(scalar_t startTime,
      scalar_t finalTime,
      const ModeSequenceTemplate& modeSequenceTemplate)
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

      // tile the mode sequence template from last eventTime to finalTime.
      // scalar_t tilingTime = eventTimes.empty() ? startTime : eventTimes.back();
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
      modeSequence.push_back(((0x01 << (staticParams_.endEffectorNumber)) - 1));
    }
  }
}