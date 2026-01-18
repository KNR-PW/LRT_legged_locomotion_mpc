#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>

#include <algorithm>
#include <limits>
#include <iostream>

#include <ocs2_core/misc/Numerics.h>

#include <legged_locomotion_mpc/common/Utils.hpp>
#include <legged_locomotion_mpc/locomotion/ModeSequenceTemplate.hpp>
#include <legged_locomotion_mpc/locomotion/ModeDynamicSequenceTemplate.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;

    GaitPlanner::GaitPlanner(GaitStaticParameters staticParams,
      GaitDynamicParameters initDynamicParams, scalar_t initPhase, scalar_t initTime):
        currentPhase_(std::move(initPhase)),
        staticParams_(std::move(staticParams)),
        currentChangeTime_(initTime),
        dynamicParams_(std::move(initDynamicParams))
    {
      if(initTime < 0.0)
      {
        std::string message = "[GaitPlanner]: Time lower than 0!";
        throw std::invalid_argument(message);
      }

      if(currentPhase_ < 0.0)
      {
        std::string message = "[GaitPlanner]: Phase lower than 0!";
        throw std::invalid_argument(message);
      }
    }

    ModeSchedule GaitPlanner::getModeSchedule(scalar_t startTime,
      scalar_t finalTime) 
    {
      assert(startTime >= currentChangeTime_);
      assert(!numerics::almost_eq(startTime, finalTime, SCALAR_EPSILON));
    
      ModeSchedule modeSchedule;
      modeSchedule.clear();

      // Get current phase in startTime
      const auto frequency = dynamicParams_.steppingFrequency;
      const scalar_t returnPhase = currentPhase_ + (startTime - currentChangeTime_) * frequency;
      const scalar_t startPhase = normalizePhase(returnPhase);

      ModeSequenceTemplate modeSequenceTemplate = getDynamicModeSequenceTemplate(
        startPhase, finalTime - startTime, staticParams_, dynamicParams_);

      auto& eventTimes = modeSchedule.eventTimes;
      auto& modeSequence = modeSchedule.modeSequence;
      const auto& templateTimes = modeSequenceTemplate.switchingTimes;
      const auto& templateModeSequence = modeSequenceTemplate.modeSequence;
      const size_t numTemplateSubsystems = modeSequenceTemplate.modeSequence.size();

      assert(numTemplateSubsystems != 0);

      const contact_flags_t currentFlags = getContactFlagsAtTime(startTime);

      eventTimes.push_back(startTime);
      modeSequence.push_back(contactFlags2ModeNumber(currentFlags));

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

      const size_t standingMode = ((0x01 << (staticParams_.endEffectorNumber)) - 1);
      modeSequence.push_back(standingMode);

      return modeSchedule;
    }

    GaitFlags GaitPlanner::updateCurrentContacts(scalar_t time, 
      const contact_flags_t& currentContacts)
    {
      const contact_flags_t plannedFlags = getContactFlagsAtTime(time);

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
      assert(time >= currentChangeTime_);
      if(dynamicParams == dynamicParams_) return;

      // Update current phase
      const auto frequency = dynamicParams_.steppingFrequency;
      const scalar_t returnPhase = currentPhase_ + (time - currentChangeTime_) * frequency;
      currentPhase_ = normalizePhase(returnPhase);

      // Update change time
      currentChangeTime_ = time;
      
      // Update dynamic parameters
      dynamicParams_ = dynamicParams;
    }

    const GaitStaticParameters& GaitPlanner::getStaticParameters()
    {
      return staticParams_;
    }

    const GaitDynamicParameters& GaitPlanner::getDynamicParameters()
    {
      return dynamicParams_;
    }

    std::vector<scalar_t> GaitPlanner::getPhasesAtTime(scalar_t time) const
    {

      assert(time >= currentChangeTime_);

      std::vector<scalar_t> returnPhases(staticParams_.endEffectorNumber);

      // Add phase between index - 1 time and query time
      const auto frequency = dynamicParams_.steppingFrequency;
      scalar_t returnPhase = currentPhase_ + (time - currentChangeTime_) * frequency;

      returnPhases[0] = normalizePhase(returnPhase);
      const auto& offsets = dynamicParams_.phaseOffsets;
      for(int i = 1; i < staticParams_.endEffectorNumber; ++i)
      {
        returnPhases[i] = normalizePhase(returnPhase + offsets[i - 1]);
      }

      return returnPhases;
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
      // - SCALAR_EPSILON -> Compatibility with ModeSchedule
      assert(time >= currentChangeTime_);

      contact_flags_t returnFlags(staticParams_.endEffectorNumber);
      
      // Add phase between index - 1 time and query time
      const auto frequency = dynamicParams_.steppingFrequency;
      scalar_t returnPhase = currentPhase_ + (time - currentChangeTime_) * frequency;

      const auto& offsets = dynamicParams_.phaseOffsets;
      const scalar_t swingRatio = dynamicParams_.swingRatio;

      returnFlags[0] = normalizePhase(returnPhase - SCALAR_EPSILON) >= swingRatio;
      for(int i = 1; i < staticParams_.endEffectorNumber; ++i)
      {
        returnFlags[i] = normalizePhase(returnPhase + offsets[i - 1] - SCALAR_EPSILON) >= swingRatio;
      }
      return returnFlags;
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
  }
}