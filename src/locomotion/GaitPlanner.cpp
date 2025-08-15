#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/locomotion/ModeCommon.hpp>

#include <algorithm>
#include <limits>

#include <ocs2_core/misc/Lookup.h>


namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;
    using namespace ocs2::legged_robot;

    GaitPlanner::GaitPlanner(const GaitStaticParameters& staticParams,
      const GaitDynamicParameters initDynamicParams):
        publicStaticParams_(staticParams),
        privateStaticParams_(getPrivateStaticParams(staticParams)),
        publicDynamicParams_(initDynamicParams)
    {
      updatePrivateDynamicParams(initDynamicParams);

      // Start with standing 
      currentPhase_ = initDynamicParams.swingRatio;
      scalar_t startTimeHorizon = 10.0;

      modeSequenceTemplate_ = getDynamicModeSequenceTemplate(currentPhase_, startTimeHorizon,
        publicStaticParams_, publicDynamicParams_);

      currentPhase_ += startTimeHorizon / publicDynamicParams_.steppingFrequency;
      
      insertModeSequenceTemplate(0.0, startTimeHorizon, modeSequenceTemplate_);

    }

    void GaitPlanner::setModeSchedule(const ModeSchedule &modeSchedule)
    { 
      modeSchedule_ = modeSchedule; 
    }

    ModeSchedule GaitPlanner::getModeSchedule(scalar_t lowerBoundTime,
      scalar_t upperBoundTime) 
    {
      auto &eventTimes = modeSchedule_.eventTimes;
      auto &modeSequence = modeSchedule_.modeSequence;
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(),
       lowerBoundTime) - eventTimes.begin();

      if (index > 0) 
      {
        // delete the old logic from index and set the default start phase to stance
        eventTimes.erase(eventTimes.begin(), eventTimes.begin() + index - 1);
        // keep the one before the last to make it stance
        modeSequence.erase(modeSequence.begin(), modeSequence.begin() + index - 1);

      }

      // Start tiling at time
      const auto tilingStartTime = eventTimes.empty() ? lowerBoundTime : eventTimes.back();

      // delete the last default stance phase
      eventTimes.erase(eventTimes.end() - 1, eventTimes.end());
      modeSequence.erase(modeSequence.end() - 1, modeSequence.end());

      // tile the template logic
      tileModeSequenceTemplate(tilingStartTime, upperBoundTime);
      return modeSchedule_;
    }

    void GaitPlanner::updateCurrentContacts(scalar_t time, contact_flags_t currentContacts)
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
        if(i % 2 == 0)
        {
          currentIndex -= i; // TODO SPRAWDZ TO
        }
        else
        {
          currentIndex += i; // TODO SPRAWDZ TO
        }
        if(modeSequence[currentIndex] == realMode) break;
      }
      
      if(currentIndex == 0)
      {
        return 0;
      }
      // TODO ZNALZAŁEM DOBRY INDEKS, ZUPDATUJ modeSchedule_

    }

    void GaitPlanner::updateWalkingGait(scalar_t startTime,
      scalar_t finalTime, 
      const GaitDynamicParameters& dynamicParams)
    {
      const auto &eventTimes = modeSchedule_.eventTimes;

      // find the index on which the new gait should be added
      const size_t index = std::lower_bound(eventTimes.begin(), eventTimes.end(),
       startTime) - eventTimes.begin();

      // Change current phase 
      if (index < eventTimes.size()) 
      {
         currentPhase_ -= (eventTimes.back() - startTime) / publicStaticParams_.plannerFrequency;
      }
      else
      {
        currentPhase_ += (startTime - eventTimes.back()) / publicDynamicParams_.steppingFrequency;
      }

      // Update params
      publicDynamicParams_ = dynamicParams;
      ModeSequenceTemplate newModeSequenceTemplate = getDynamicModeSequenceTemplate(currentPhase_,
        finalTime - startTime, publicStaticParams_, publicDynamicParams_);

      insertModeSequenceTemplate(startTime, finalTime, newModeSequenceTemplate);

      updateCurrentPhase(startTime, finalTime);
    }

    const GaitStaticParameters& GaitPlanner::getStaticParameters()
    {
      return publicStaticParams_;
    }

    const GaitDynamicParameters& GaitPlanner::getDynamicParameters()
    {
      return publicDynamicParams_;
    }

    void GaitPlanner::updateState();

    GaitStaticPrivateInfo GaitPlanner::getPrivateStaticParams(const GaitStaticParams& params)
    {
      GaitStaticParameters privateStaticParams;
      privateStaticParams.numEndEffectors = Params.threeDofendEffectorNames.size() + Params.sixDofendEffectorNames.size();
      privateStaticParams.plannerDeltaTime = 1 / Params.plannerFrequency;
      privateStaticParams.timeHorizonLentgh = Params.timeHorizion * Params.plannerFrequency;
      return privateStaticParams;
    }

    void GaitPlanner::insertModeSequenceTemplate(ocs2::scalar_t startTime,
      ocs2::scalar_t finalTime,
      const ocs2::legged_robot::ModeSequenceTemplate& modeSequenceTemplate)
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

      // tile the mode sequence template from startTime to finalTime.
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
      modeSequence.push_back(ModeNumber::STANCE);
    }

    void GaitPlanner::updateCurrentPhase(scalar_t startTime, scalar_t finalTime)
    {
      currentPhase_ += (finalTime - startTime) / publicDynamicParams_.steppingFrequency;
    }

    GaitPrivateDynamicParams GaitPlanner::updatePrivateDynamicParams(const GaitDynamicParameters& Params)
    {
      assert(privateDynamicParams_.phaseIndexOffsets.size() == Params.phaseOffsets.size());

      privateDynamicParams_.cacheLength = 1 / (Params.steppingFrequency * privateStaticParams_.plannerDeltaTime);
      privateDynamicParams_.swingStartIndex = Params.swingRatio * privateDynamicParams_.cacheLength;

      for(int i = 0; i < privateStaticParams_.numEndEffectors; ++i)
      {
        privateDynamicParams_.phaseIndexOffsets[i] = static_cast<size_t>(Params.phaseOffsets[i] / (privateDynamicParams_.cacheLength * 2 * M_PI));
      }
    }
  }
}