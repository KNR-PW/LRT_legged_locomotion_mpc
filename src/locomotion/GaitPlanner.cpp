#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/locomotion/MotionPhaseDefinitions.hpp>

#include <algorithm>


namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    namespace ocs2;

    GaitPlanner::GaitPlanner(
      const GaitStaticInfo& staticInfo,
      const GaitDynamicInfo& initDynamicInfo):
        publicStaticInfo_(staticInfo), privateStaticInfo(getPrivateStaticInfo(staticInfo)),
        publicDynamicInfo_(initDynamicInfo)
    {
      updatePrivateDynamicInfo(initDynamicInfo);
      updateCachedPhaseVector();

      // Stand for 10000 seconds lmao
      modeSchedule_.eventTimes.push_back(0.0);
      modeSchedule_.eventTimes.push_back(10000.0);
      modeSchedule_.modeSequence.puh_back(0);

    }

    void GaitPlanner::setModeSchedule(const ModeSchedule &modeSchedule)
    { 
      modeSchedule_ = modeSchedule; 
    }

    ModeSchedule GaitPlanner::getModeSchedule(ocs2::scalar_t initTime, ocs2::scalar_t finalTime)
    {

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
      privateStaticInfo.numEndEffectors = info.endEffectorNames.size();
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