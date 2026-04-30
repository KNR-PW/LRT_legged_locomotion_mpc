//
// Created by rgrandia on 27.04.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 01.09.2025 
//

#include <legged_locomotion_mpc/locomotion/SingleLegLogic.hpp>

#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {

    using namespace ocs2;

    std::vector<std::vector<ContactTiming>> extractContactTimingsPerLeg(
      const ModeSchedule &modeSchedule, size_t endEffectorNum) 
    {
      std::vector<std::vector<ContactTiming>> contactTimingsPerLeg(endEffectorNum);

      // Convert mode sequence to a contact flag vector per leg
      const auto contactSequencePerLeg = extractContactFlags(modeSchedule.modeSequence, endEffectorNum);

      // Extract timings per leg
      for (size_t legIndex = 0; legIndex < endEffectorNum; ++legIndex) 
      {
        contactTimingsPerLeg[legIndex] = extractContactTimings(
        modeSchedule.eventTimes, contactSequencePerLeg[legIndex]);
      }

      return contactTimingsPerLeg;
    }

    scalar_t getTimeOfNextLiftOff(scalar_t currentTime, 
      const std::vector<ContactTiming> &contactTimings) 
    {
      for (const auto &contactPhase: contactTimings) 
      {
        if(hasEndTime(contactPhase) && contactPhase.end > currentTime) 
        {
          return contactPhase.end;
        }
      }

      return timingNaN();
    }

    scalar_t getTimeOfNextTouchDown(scalar_t currentTime,
      const std::vector<ContactTiming> &contactTimings) 
    {
      for (const auto &contactPhase: contactTimings) 
      {
        if(hasStartTime(contactPhase) && contactPhase.start > currentTime) 
        {
          return contactPhase.start;
        }
      }

      return timingNaN();
    }

    std::vector<ContactTiming> extractContactTimings(
      const std::vector<scalar_t>& eventTimes,
      const std::vector<bool>& singleEndEffectorContactFlags) 
    {
      assert(eventTimes.size() + 1 == singleEndEffectorContactFlags.size());
      const size_t numPhases = singleEndEffectorContactFlags.size();

      std::vector<ContactTiming> contactTimings;
      contactTimings.reserve(1 + eventTimes.size() / 2); // Approximate upper bound

      size_t currentPhase = 0;

      while (currentPhase < numPhases) 
      {
        // Search where contact phase starts
        while (currentPhase < numPhases && !singleEndEffectorContactFlags[currentPhase]) 
        {
          ++currentPhase;
        }
        if(currentPhase >= numPhases) 
        {
          break; // No more contact phases
        }

        // Register start of the contact phase
        const scalar_t startTime = (currentPhase == 0)
          ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase - 1];

        // Find when the contact phase ends
        while (currentPhase + 1 < numPhases && singleEndEffectorContactFlags[currentPhase + 1]) 
        {
          ++currentPhase;
        }

        // Register end of the contact phase
        const scalar_t endTime = (currentPhase + 1 >= numPhases)
          ? std::numeric_limits<scalar_t>::quiet_NaN() : eventTimes[currentPhase];

        // Add to phases
        contactTimings.push_back({startTime, endTime});
        ++currentPhase;
      }
      
      return contactTimings;
    }

    std::vector<std::vector<bool>> extractContactFlags(
      const std::vector<size_t>& modeSequence, size_t endEffectorNum) 
    {
      const size_t numPhases = modeSequence.size();

      std::vector<std::vector<bool>> contactFlagStock(endEffectorNum);
      std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

      for (size_t i = 0; i < numPhases; i++) 
      {
        const contact_flags_t contactFlag = modeNumber2ContactFlags(modeSequence[i]);
        for (size_t j = 0; j < endEffectorNum; j++) 
        {
          contactFlagStock[j][i] = contactFlag[j];
        }
      }

      return contactFlagStock;
    }
  } // namespace locomotion
} // namespace legged_locomotion_mpc