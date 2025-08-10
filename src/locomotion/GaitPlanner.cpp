#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/locomotion/MotionPhaseDefinitions.hpp>

#include <algorithm>
#include <limits>

#include <ocs2_core/misc/Lookup.h>


namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;

    GaitPlanner::GaitPlanner(
      const GaitStaticInfo& staticInfo,
      const GaitDynamicInfo& initDynamicInfo):
        publicStaticInfo_(staticInfo), privateStaticInfo(getPrivateStaticInfo(staticInfo)),
        publicDynamicInfo_(initDynamicInfo)
    {
      updatePrivateDynamicInfo(initDynamicInfo);
      updateCachedPhaseVector();

      // Start with standing forever
      modeSchedule_.eventTimes.push_back(0.5);
      const contact_flags_t stanceFlags(privateStaticInfo_.numEndEffectors, true);
      const size_t stanceMode = modeNumber2ContactFlags(stanceFlags);
      modeSchedule_.modeSequence.push_back(stanceMode);
      modeSchedule_.modeSequence.push_back(stanceMode);

    }

    void GaitPlanner::setModeSchedule(const ModeSchedule &modeSchedule)
    { 
      modeSchedule_ = modeSchedule; 
    }

    ModeSchedule GaitPlanner::getModeSchedule(ocs2::scalar_t initTime, ocs2::scalar_t finalTime)
    {

    }

    void GaitPlanner::insertCurrentContacts(scalar_t time, contact_flags_t currentContacts)
    {
      const size_t realMode = contactFlags2ModeNumber(currentContacts);
      const size_t plannedModeIndex = lookup::findIndexInTimeArray(eventTimes, time);

      if(realMode == modeSequence[plannedModeIndex]) return; // As planned, early return

      /* Not as planned, checking closest mode, that satisfies real contacts */

      const auto& eventTimes = modeSchedule_.eventTimes;
      const auto modeSequence = modeSchedule_.modeSequence;
      const size_t closestRange = 2 * plannedModeIndex;
      size_t currentIndex = plannedModeIndex;
      for(size_t i = 1; closestRange; ++i)
      {
        /* Alternating iterateration, starting from planned mode index */
        currentIndex += (2 * (i % 2) - 1) * i; // TODO SPRAWDZ TO
        if(modeSequence[currentIndex] == realMode) break;
      }
      // TODO ZNALZAŁEM DOBRY INDEKS, ZUPDATUJ modeSchedule_

    }

    void GaitPlanner::setDynamicInfo(const GaitDynamicInfo& dynamicInfo)
    {
      publicDynamicInfo_ = dynamicInfo;
      updatePrivateDynamicInfo(dynamicInfo);
      updateCachedPhaseVector();
    }

    const GaitStaticInfo& GaitPlanner::getStaticInfo()
    {
      return publicStaticInfo_;
    }

    const GaitDynamicInfo& GaitPlanner::getDynamicInfo()
    {
      return publicDynamicInfo_;
    }

    void GaitPlanner::updateState();

    GaitStaticPrivateInfo GaitPlanner::getPrivateStaticInfo(const GaitStaticInfo& info)
    {
      GaitStaticPrivateInfo privateStaticInfo;
      privateStaticInfo.numEndEffectors = info.threeDofendEffectorNames.size() + info.sixDofendEffectorNames.size();
      privateStaticInfo.plannerDeltaTime = 1 / info.plannerFrequency;
      privateStaticInfo.timeHorizonLentgh = info.timeHorizion * info.plannerFrequency;
      return privateStaticInfo;
    }

    GaitPrivateDynamicInfo GaitPlanner::updatePrivateDynamicInfo(const GaitDynamicInfo& info)
    {
      assert(privateDynamicInfo_.phaseIndexOffsets.size() == info.phaseOffsets.size());

      privateDynamicInfo_.cacheLength = 1 / (info.steppingFrequency * privateStaticInfo_.plannerDeltaTime);
      privateDynamicInfo_.swingStartIndex = info.swingRatio * privateDynamicInfo_.cacheLength;

      for(int i = 0; i < privateStaticInfo_.numEndEffectors; ++i)
      {
        privateDynamicInfo_.phaseIndexOffsets[i] = static_cast<size_t>(info.phaseOffsets[i] / (privateDynamicInfo_.cacheLength * 2 * M_PI));
      }
    }

    void GaitPlanner::updateCachedPhaseVector()
    {
      cachedPhaseVector_.resize(privateDynamicInfo_.cacheLength);
      std::fill(cachedPhaseVector_.begin(), cachedPhaseVector_.begin() + privateDynamicInfo_.swingStartIndex, true);
      std::fill(cachedPhaseVector_.begin() + privateDynamicInfo_.swingStartIndex, cachedPhaseVector_.end(), false);
    }
  }
}