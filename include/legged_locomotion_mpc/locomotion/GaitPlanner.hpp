// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */


#ifndef __GAIT_PLANNER_LEGGED_LOCOMOTION_MPC__
#define __GAIT_PLANNER_LEGGED_LOCOMOTION_MPC__

#include <vector>
#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/ModeSchedule.h>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    class GaitPlanner
    {
      public:
        struct GaitStaticInfo
        {
          ocs2::scalar_t plannerFrequency;
          ocs2::scalar_t timeHorizion;
          std::vector<std::string> endEffectorNames;

        };

        struct GaitDynamicInfo
        {
          ocs2::scalar_t steppingFrequency;
          ocs2::scalar_t swingRatio; // val < swingRatio -> STANCE, else SWING
          std::vector<bool> endEffectorsInContact;
          std::vector<ocs2::scalar_t> phaseOffsets;
        };

        GaitPlanner(const GaitStaticInfo& staticInfo, const GaitDynamicInfo& initDynamicInfo);

        void setModeSchedule(const ocs2::ModeSchedule &modeSchedule);

        ocs2::ModeSchedule getModeSchedule(ocs2::scalar_t initTime, ocs2::scalar_t finalTime);

        void setDynamicInfo(const GaitDynamicInfo& dynamicInfo);

        const GaitStaticInfo& getStaticInfo();
        const GaitDynamicInfo& getDynamicInfo();

        void updateState();


      private:
        struct GaitStaticPrivateInfo
        {
          size_t numEndEffectors;
          size_t timeHorizonLentgh;
          ocs2::scalar_t plannerDeltaTime;
        };

        struct GaitDynamicPrivateInfo
        {
          size_t cacheLength; // 1 / (f * delta_t)
          size_t swingStartIndex;
          

          std::vector<size_t> phaseIndexOffsets;
        };

        GaitStaticPrivateInfo getPrivateStaticInfo(const GaitStaticInfo& info);
        void updatePrivateDynamicInfo(const GaitDynamicInfo& info);
        void updateCachedPhaseVector();



        std::vector<bool> cachedPhaseVector_; // 0 index -> 0 (contact, true), max index -> 2pi (swing, false)
        std::vector<bool> currentPlannedState_;

        const GaitStaticInfo publicStaticInfo_;
        GaitDynamicInfo publicDynamicInfo_;
        const GaitStaticPrivateInfo privateStaticInfo_;
        GaitDynamicPrivateInfo privateDynamicInfo_;

        ocs2::ModeSchedule modeSchedule_;
    };
  };
};

#endif