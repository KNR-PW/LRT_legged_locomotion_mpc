#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>

#include <algorithm>
#include <limits>
#include <iostream>

#include <ocs2_core/misc/Numerics.h>

#include <legged_locomotion_mpc/common/Utils.hpp>


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

      // Add first modes and time
      modeSchedule_.eventTimes.push_back(initTime);

      const contact_flags_t initFlags = gaitPhaseController_.getContactFlagsAtTime(initTime);
      modeSchedule_.modeSequence.push_back(contactFlags2ModeNumber(initFlags));
      modeSchedule_.modeSequence.push_back(contactFlags2ModeNumber(initFlags));
      
      // insertModeSequenceTemplate(initTime, modeSequenceTemplate_.switchingTimes.back(),
      //   modeSequenceTemplate_);
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
      const size_t index = utils::findIndexInTimeArray(eventTimes, startTime);

      if(index > 0 && eventTimes.size() > 0) 
      {
        // Update gait phase controller
        gaitPhaseController_.remove(startTime);
        // delete the old logic from index and set the default start phase to stance
        eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index);
        // keep the one before the last to make it stance
        modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index);
      }

      // Start tiling at time
      const auto tilingStartTime = eventTimes.empty() ? startTime : eventTimes.back();

      // delete the last default stance phase
      if(eventTimes.size() > 0)
      {
        eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
        modeSequence.erase(modeSequence.end() - 1, modeSequence.end());
      }

      // tile the template logic
      tileModeSequenceTemplate(tilingStartTime, finalTime);

      return modeSchedule_;
    }

    GaitFlags GaitPlanner::updateCurrentContacts(scalar_t time, 
      const contact_flags_t& currentContacts)
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
      if(dynamicParams == gaitPhaseController_.getDynamicParametersAtTime(time)) return;
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

    std::vector<scalar_t> GaitPlanner::getPhasesAtTime(scalar_t time) const
    {
      return gaitPhaseController_.getPhasesAtTime(time);
    }

    std::vector<std::vector<scalar_t>> GaitPlanner::getPhasesAtTimes(
      std::vector<scalar_t> times) const
    {
      const size_t referenceSize = times.size();
      std::vector<std::vector<scalar_t>> phasesTrajectory;
      phasesTrajectory.reserve(referenceSize);
      for(const auto time: times)
      {
        std::vector<scalar_t> currentPhases = getPhasesAtTime(time);
        phasesTrajectory.push_back(std::move(currentPhases));
      }
      return phasesTrajectory;
    }

    contact_flags_t GaitPlanner::getContactFlagsAtTime(scalar_t time) const
    {
      return gaitPhaseController_.getContactFlagsAtTime(time);
    }

    std::vector<contact_flags_t> GaitPlanner::getContactFlagsAtTimes(
      std::vector<scalar_t> times) const
    {
      const size_t referenceSize = times.size();
      std::vector<contact_flags_t> contactFlagsTrajectory;
      contactFlagsTrajectory.reserve(referenceSize);
      for(const auto time: times)
      {
        contact_flags_t currentFlags = getContactFlagsAtTime(time);
        contactFlagsTrajectory.push_back(std::move(currentFlags));
      }
      return contactFlagsTrajectory;
    }

    void GaitPlanner::insertModeSequenceTemplate(scalar_t startTime,
      scalar_t finalTime,
      const ModeSequenceTemplate& modeSequenceTemplate)
    {
      modeSequenceTemplate_ = modeSequenceTemplate;
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;

      // find the index on which the new gait should be added
      const size_t index = utils::findIndexInTimeArray(eventTimes, startTime);

      // delete the old logic from the index
      if (index < eventTimes.size()) 
      {
        eventTimes.erase(eventTimes.begin() + index, eventTimes.end());
        modeSequence.erase(modeSequence.begin() + index + 1, modeSequence.end());
      }

      tileModeSequenceTemplate(startTime, finalTime);
    }

    void GaitPlanner::tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime)
    {
      auto& eventTimes = modeSchedule_.eventTimes;
      auto& modeSequence = modeSchedule_.modeSequence;
      const auto& templateTimes = modeSequenceTemplate_.switchingTimes;
      const auto& templateModeSequence = modeSequenceTemplate_.modeSequence;
      const size_t numTemplateSubsystems = modeSequenceTemplate_.modeSequence.size();

      // Get current mode first
      const contact_flags_t currentFlags = gaitPhaseController_.getContactFlagsAtTime(startTime);
      const size_t currentMode = contactFlags2ModeNumber(currentFlags);
      modeSequence.back() = currentMode;

      // If no template subsystem is defined, the last subsystem should continue for ever
      if (numTemplateSubsystems == 0) 
      {
        return;
      }

      if(numerics::almost_eq(startTime, finalTime, SCALAR_EPSILON))
      {
        const size_t standingMode = ((0x01 << (staticParams_.endEffectorNumber)) - 1);
        if(modeSequence.back() == standingMode)
        {
          eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
        }
        else
        {
          modeSequence.push_back(standingMode);
        }
        return;
      }

      if (!eventTimes.empty() && startTime < eventTimes.back()) 
      {
        throw std::runtime_error(
          "The initial time for template-tiling is not greater than the last event time.");
      }

      // If template starts with same mode, extend this mode and make one iteration
      if(templateModeSequence.front() == modeSequence.back())
      {
        scalar_t deltaTime = templateTimes[1] - templateTimes[0];
        eventTimes.push_back(startTime + deltaTime);
        for (size_t i = 1; i < templateModeSequence.size(); i++) 
        {
          scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
          if(templateModeSequence[i] == modeSequence.back())
          {
            eventTimes.back() += deltaTime;
          }
          else
          {
            modeSequence.push_back(templateModeSequence[i]);
            eventTimes.push_back(eventTimes.back() + deltaTime);
          }
        } // end of i loop
      }
      else
      {
        // end last mode in start time on new one
        eventTimes.push_back(startTime);
      }

      // concatenate from index
      while (eventTimes.back() < finalTime) 
      {
        for (size_t i = 0; i < templateModeSequence.size(); i++) 
        {
          scalar_t deltaTime = templateTimes[i + 1] - templateTimes[i];
          if(templateModeSequence[i] == modeSequence.back())
          {
            eventTimes.back() += deltaTime;
          }
          else
          {
            modeSequence.push_back(templateModeSequence[i]);
            eventTimes.push_back(eventTimes.back() + deltaTime);
          }
        } // end of i loop
      } // end of while loop

      // default final phase (only if it is not the same one)
      const size_t standingMode = ((0x01 << (staticParams_.endEffectorNumber)) - 1);
      if(modeSequence.back() == standingMode)
      {
        eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
      }
      else
      {
        modeSequence.push_back(standingMode);
      }
    }
  }
}