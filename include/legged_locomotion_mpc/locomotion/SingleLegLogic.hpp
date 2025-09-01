//
// Created by rgrandia on 27.04.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 01.09.2025 
//

#ifndef __SINGLE_LEG_LOGIC_LEGGED_LOCOMOTION_MPC__
#define __SINGLE_LEG_LOGIC_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>

#include <ocs2_core/reference/ModeSchedule.h>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    struct ContactTiming 
    {
      ocs2::scalar_t start;
      ocs2::scalar_t end;
    };

    inline ocs2::scalar_t timingNaN() 
    {
      return std::numeric_limits<ocs2::scalar_t>::quiet_NaN();
    }

    inline bool hasStartTime(const ContactTiming& timing) 
    {
      return !std::isnan(timing.start);
    }
    inline bool hasEndTime(const ContactTiming& timing) 
    {
      return !std::isnan(timing.end);
    }
    inline bool startsWithSwingPhase(const std::vector<ContactTiming>& timings) 
    {
      return timings.empty() || hasStartTime(timings.front());
    }
    inline bool startsWithStancePhase(const std::vector<ContactTiming>& timings) 
    {
      return !startsWithSwingPhase(timings);
    }
    inline bool endsWithSwingPhase(const std::vector<ContactTiming>& timings) 
    {
      return timings.empty() || hasEndTime(timings.back());
    }
    inline bool endsWithStancePhase(const std::vector<ContactTiming>& timings) 
    {
      return !endsWithSwingPhase(timings);
    }

    inline bool touchesDownAtLeastOnce(const std::vector<ContactTiming>& timings) 
    {
      return std::any_of(timings.begin(), timings.end(), [](const ContactTiming& timing)
        { return hasStartTime(timing); });
    }

    inline bool liftsOffAtLeastOnce(const std::vector<ContactTiming>& timings) 
    {
      return !timings.empty() && hasEndTime(timings.front());
    }

    /** Extracts the contact timings for all legs from a modeSchedule */
    std::vector<std::vector<ContactTiming>> extractContactTimingsPerLeg(
      const ocs2::ModeSchedule& modeSchedule, size_t endEffectorNum);

    /** Returns time of the next lift off. Returns nan if leg is not lifting off */
    ocs2::scalar_t getTimeOfNextLiftOff(
      ocs2::scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

    /** Returns time of the  touch down for all legs from a modeschedule. Returns nan if leg does not touch down */
    ocs2::scalar_t getTimeOfNextTouchDown(
      ocs2::scalar_t currentTime, const std::vector<ContactTiming>& contactTimings);

    /**
     * Get {startTime, endTime} for all contact phases. Swingphases are always implied in between: endTime[i] < startTime[i+1]
     * times are NaN if they cannot be identified at the boundaries
     * Vector is empty if there are no contact phases
     */
    std::vector<ContactTiming> extractContactTimings(
      const std::vector<ocs2::scalar_t>& eventTimes,
      const std::vector<bool>& singleEndEffectorContactFlags);

    /**
     * Extracts for each leg the contact sequence over the motion phase sequence.
     * @param modeSequence : Sequence of contact modes.
     * @return Sequence of contact flags per leg.
     */
    std::vector<std::vector<bool>> extractContactFlags(
      const std::vector<size_t>& modeSequence, size_t endEffectorNum);

  }; // locomotion
}; // namespace legged_locomotion_mpc

#endif
