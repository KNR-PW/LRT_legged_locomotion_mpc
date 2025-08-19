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
        : currentPhase_(initPhase), staticParams_(initStaticParams)
    {
      eventTimes_.push_back(initTime);
      dynamicParamsVec_.push_back(initDynamicParams);
    }

    std::vector<ocs2::scalar_t> GaitDynamicPhaseController::getPhasesAtTime(scalar_t time)
    {
      assert(time > eventTimes_.front());

      std::vector<scalar_t> returnPhases(staticParams_.endEffectorNumber);
      scalar_t returnPhase = currentPhase_;

      // Find index of time larger that query time
      const size_t index = std::lower_bound(eventTimes_.begin(), eventTimes_.end(),
       time) - eventTimes_.begin();
      
      // Loop through fixed times
      for(int i = 1; i < index; ++i)
      {
        returnPhase += (eventTimes_[i] - eventTimes_[i - 1]) * dynamicParamsVec_[i - 1].steppingFrequency;
      }
      // Add last time
      returnPhase += (time - eventTimes_[index - 1]) * dynamicParamsVec_[index - 1].steppingFrequency;
      std::cerr << "CALCULATED: " << returnPhase << std::endl;
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
      scalar_t returnPhase = currentPhase_;

      // Find index of time larger that query time
      const size_t index = std::lower_bound(eventTimes_.begin(), eventTimes_.end(),
       time) - eventTimes_.begin();
      
      // Loop through fixed times
      for(int i = 1; i < index; ++i)
      {
        returnPhase += (eventTimes_[i] - eventTimes_[i - 1]) * dynamicParamsVec_[i - 1].steppingFrequency;
      }
      // Add last time
      returnPhase += (time - eventTimes_[index - 1]) * dynamicParamsVec_[index - 1].steppingFrequency;

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

      // Update current phase, so that it matches value in query time
      // Loop through fixed times
      for(int i = 1; i < index; ++i)
      {
        currentPhase_ += (eventTimes_[i] - eventTimes_[i - 1]) * dynamicParamsVec_[i - 1].steppingFrequency;
      }
      // Add last time
      currentPhase_ += (time - eventTimes_[index - 1]) * dynamicParamsVec_[index - 1].steppingFrequency;

      // delete the old time and frequency values to last time smaller than query time
      eventTimes_.erase(eventTimes_.begin(), eventTimes_.begin() + index - 1);
      dynamicParamsVec_.erase(dynamicParamsVec_.begin(), dynamicParamsVec_.begin() + index - 1);

      // Change last time smaller than query time to query time
      eventTimes_.front() = time;
    }

    void GaitDynamicPhaseController::update(scalar_t newTime, 
      const GaitDynamicParameters& newDynamicParams)
    {
      assert(newTime > eventTimes_.back());

      eventTimes_.push_back(newTime);
      dynamicParamsVec_.push_back(newDynamicParams);
    }
  }
}