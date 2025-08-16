
#include <legged_locomotion_mpc/locomotion/ModeDynamicSequenceTemplate.hpp>

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <set>

#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;
    using namespace ocs2::legged_robot;

    // std::unordered_map<scalar_t, std::set<size_t>> getDuplicateIndexes(
    //   const std::vector<scalar_t>& vector)
    // {
    //   std::unordered_map<scalar_t, std::set<size_t>> duplicates;
    //   for(int i = 0; i < vector.size(); ++i)
    //   {
    //     for(int j = i + 1; j < vector.size(); ++j)
    //     {
    //       if(vector[i] == vector[j])
    //       {
    //         auto duplicateIterator = duplicates.find(vector[i]);
    //         if(duplicateIterator == duplicates.end())
    //         {
    //           duplicates.emplace(vector[i], {i, j});
    //         }
    //         else
    //         {
    //           duplicateIterator->second.insert(j);
    //         }
    //       }
    //     }
    //   }
    //   return duplicates;
    // }

     ModeSequenceTemplate getDynamicModeSequenceTemplate(
      scalar_t currentPhase,
      scalar_t timeHorizon,
      const GaitStaticParameters& staticParams,
      const GaitDynamicParameters& dynamicParams)
    {
      assert(staticParams.endEffectorNumber == (dynamicParams.phaseOffsets.size() + 1));
      const auto& phaseOffsets = dynamicParams.phaseOffsets;
      const scalar_t swingRatio = dynamicParams.swingRatio;
      scalar_t frequency = dynamicParams.steppingFrequency;

      contact_flags_t currentMode;
      std::vector<scalar_t> currentContactStates(staticParams.endEffectorNumber);

      currentContactStates[0] = normalizePhase(currentPhase);
      currentMode[0] = currentContactStates[0] >= swingRatio;
      for(int i = 1; i < staticParams.endEffectorNumber; ++i)
      {
        currentContactStates[i] = normalizePhase(currentPhase + phaseOffsets[i - 1]);
        currentMode[i] = currentContactStates[i] >= swingRatio;
      }

      std::vector<scalar_t> switchingTimes;
      switchingTimes.push_back(0.0);

      std::vector<size_t> modeSequence;
      modeSequence.push_back(currentMode.to_ulong());

      /* If frequency is really low, just stand in place with current mode */
      if(frequency < Definitions::MIN_STEPPING_FREQUENCY)
      {
        switchingTimes.push_back(timeHorizon);
        return ModeSequenceTemplate(switchingTimes, modeSequence);
      }

      if(frequency > Definitions::MAX_STEPPING_FREQUENCY)
      {
        frequency = Definitions::MAX_STEPPING_FREQUENCY;
      }

      // /* Check if any offset phases are not the same */
      // const auto duplicates = getDuplicateIndexes(phaseOffsets);
      // bool areDuplicates = !duplicates.empty();

      // if(areDuplicates)
      // {
        
      // }

      /* Min queue that gets earliest change */
      using timeIndexQueue = std::priority_queue<std::pair<scalar_t, size_t>,
        std::vector<std::pair<scalar_t, size_t>>,
        std::greater<std::pair<scalar_t, size_t>>>;

      const scalar_t gaitPeriod = 1 / dynamicParams.steppingFrequency;
      const scalar_t timeStance = (1 - swingRatio) * gaitPeriod;
      const scalar_t timeSwing = gaitPeriod - timeStance;

      timeIndexQueue timeEndEffectorIndexQueue;

      for(size_t i = 0; i < staticParams.endEffectorNumber; ++i)
      {
        const scalar_t timeToNextMode = getTimeToNextMode(currentContactStates[i], swingRatio, gaitPeriod);
        timeEndEffectorIndexQueue.push({timeToNextMode, i});
      }


      scalar_t time = 0;

      while(time < timeHorizon)
      {
        const size_t currentModeNum = modeSequence.back();
        scalar_t nextTime;
        size_t nextIndex;
        std::tie(nextTime, nextIndex) = timeEndEffectorIndexQueue.top();
        timeEndEffectorIndexQueue.pop();
        currentMode[nextIndex] = !currentMode[nextIndex];

        if(currentMode[nextIndex] == 0) // swing time
        {
          timeEndEffectorIndexQueue.push({nextTime + timeSwing, nextIndex});
        }
        else // stance time
        {
          timeEndEffectorIndexQueue.push({nextTime + timeStance, nextIndex});
        }

        while(nextTime == timeEndEffectorIndexQueue.top().first)
        {
          /* Found end effectors that have same offsets */
          size_t anothertIndex = timeEndEffectorIndexQueue.top().second;
          timeEndEffectorIndexQueue.pop();
          currentMode[anothertIndex] = !currentMode[anothertIndex];

          if(currentMode[anothertIndex] == 0) // swing time
          {
            timeEndEffectorIndexQueue.push({nextTime + timeSwing, anothertIndex});
          }
          else // stance time
          {
            timeEndEffectorIndexQueue.push({nextTime + timeStance, anothertIndex});
          }
        }
        modeSequence.push_back(currentMode.to_ulong());
        switchingTimes.push_back(nextTime);
        time = nextTime;
      }
      // Remove mode that starts from timeHorizon -> ?
      modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

      return ModeSequenceTemplate(switchingTimes, modeSequence);
    }
  }
}