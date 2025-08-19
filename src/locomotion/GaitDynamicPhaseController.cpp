#include <legged_locomotion_mpc/locomotion/GaitDynamicPhaseController.hpp>

#include <legged_locomotion_mpc/locomotion/GaitCommon.hpp>

#include <algorithm>
#include <iostream>
namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;

    GaitDynamicPhaseController::GaitDynamicPhaseController(ocs2::scalar_t initPhase,
      ocs2::scalar_t initTime,
      const GaitStaticParameters& initStaticParams,
      const GaitDynamicParameters& initDynamicParams)
        : staticParams_(initStaticParams)
    {
      eventTimes_.push_back(initTime);
      dynamicParamsVec_.push_back(initDynamicParams);
      cachedPhase_.push_back(initPhase);
    }

    std::vector<ocs2::scalar_t> GaitDynamicPhaseController::getPhasesAtTime(scalar_t time)
    {
      assert(time > eventTimes_.front());

      std::vector<scalar_t> returnPhases(staticParams_.endEffectorNumber);

      // Find index of time larger that query time
      const size_t index = std::lower_bound(eventTimes_.begin(), eventTimes_.end(),
       time) - eventTimes_.begin();
      
      // Add phase between index - 1 time and query time
      const auto frequency = dynamicParamsVec_[index - 1].steppingFrequency;
      scalar_t returnPhase = cachedPhase_[index - 1] + (time - eventTimes_[index - 1]) * frequency;

      returnPhases[0] = normalizePhase(returnPhase);
      const auto& offsets = dynamicParamsVec_[index - 1].phaseOffsets;
      for(int i = 1; i < staticParams_.endEffectorNumber; ++i)
      {
        returnPhases[i] = normalizePhase(returnPhase + offsets[i - 1]);
      }

      return returnPhases;
    }

    contact_flags_t GaitDynamicPhaseController::getContactFlagsAtTime(scalar_t time)
    {
      assert(time > eventTimes_.front());

      contact_flags_t returnFlags(staticParams_.endEffectorNumber);

      // Find index of time larger that query time
      const size_t index = std::lower_bound(eventTimes_.begin(), eventTimes_.end(),
       time) - eventTimes_.begin();
      
      // Add phase between index - 1 time and query time
      const auto frequency = dynamicParamsVec_[index - 1].steppingFrequency;
      scalar_t returnPhase = cachedPhase_[index - 1] + (time - eventTimes_[index - 1]) * frequency;

      const auto& offsets = dynamicParamsVec_[index - 1].phaseOffsets;
      const scalar_t swingRatio = dynamicParamsVec_[index - 1].swingRatio;

      returnFlags[0] = normalizePhase(returnPhase) >= swingRatio? 1 : 0;
      for(int i = 1; i < staticParams_.endEffectorNumber; ++i)
      {
        returnFlags[i] = normalizePhase(returnPhase + offsets[i - 1]) >= swingRatio? 1 : 0;
      }

      return returnFlags;
    }

    void GaitDynamicPhaseController::remove(scalar_t time)
    {
      // Find index of time larger that query time
      const size_t index = std::lower_bound(eventTimes_.begin(), eventTimes_.end(),
       time) - eventTimes_.begin();

      if(index == 0) return;

      // delete the old time and frequency values to time smaller than query time
      eventTimes_.erase(eventTimes_.begin(), eventTimes_.begin() + index - 1);
      dynamicParamsVec_.erase(dynamicParamsVec_.begin(), dynamicParamsVec_.begin() + index - 1);
      cachedPhase_.erase(cachedPhase_.begin(), cachedPhase_.begin() + index - 1);

      // Change front time smaller than query time to query time and update front cached phase
      cachedPhase_.front() += (time - eventTimes_.front()) * dynamicParamsVec_.front().steppingFrequency;
      eventTimes_.front() = time;
    }

    void GaitDynamicPhaseController::update(scalar_t newTime, 
      const GaitDynamicParameters& newDynamicParams)
    {
      assert(newTime > eventTimes_.back());

      // Calculate new cached phase
      scalar_t newCachedPhase = cachedPhase_.back() + (newTime - eventTimes_.back()) * dynamicParamsVec_.back().steppingFrequency;
      eventTimes_.push_back(newTime);
      dynamicParamsVec_.push_back(newDynamicParams);
      cachedPhase_.push_back(newCachedPhase);
    }
  }
}