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

#include <legged_locomotion_mpc/locomotion/GaitParameters.hpp>
#include <legged_locomotion_mpc/locomotion/MotionPhaseDefinitions.hpp>
#include <legged_locomotion_mpc/locomotion/ModeDynamicSequenceTemplate.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    class GaitPlanner
    {
      public:

        GaitPlanner(const GaitStaticParameters& staticParams,
          const GaitDynamicParameters initDynamicParams);

        void setModeSchedule(const ocs2::ModeSchedule &modeSchedule);

        ocs2::ModeSchedule getModeSchedule(ocs2::scalar_t startTime, ocs2::scalar_t finalTime);

        void updateWalkingGait(scalar_t startTime,
          scalar_t finalTime, 
          const GaitDynamicParameters& dynamicParams);
        
        void updateCurrentContacts(scalar_t time, contact_flags_t currentContacts);

        void insertModeSequenceTemplate(ocs2::scalar_t startTime,
          ocs2::scalar_t finalTime,
          const ocs2::legged_robot::ModeSequenceTemplate& modeSequenceTemplate);


        const GaitStaticParameters& getStaticParameters();
        const GaitDynamicParameters& getDynamicParameters();

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

        GaitStaticPrivateInfo getPrivateStaticParams(const GaitStaticParams& params);

        /**
         * Extends the switch information from startTime to finalTime based on the internal template mode sequence.
         *
         * @param [in] startTime: The initial time from which the mode schedule should be appended with the template.
         * @param [in] finalTime: The final time to which the mode schedule should be appended with the template.
         */
        void tileModeSequenceTemplate(scalar_t startTime, scalar_t finalTime);

        void updateCurrentPhase(scalar_t startTime, scalar_t finalTime);


        ocs2::scalar_t currentPhase_;
        
        const GaitStaticParameters publicStaticParams_;
        GaitDynamicParameters publicDynamicParams_;
        const GaitStaticPrivateInfo privateStaticParams_;
        GaitDynamicPrivateInfo privateDynamicParams_;

        ocs2::ModeSchedule modeSchedule_;
        ocs2::legged_robot::ModeSequenceTemplate modeSequenceTemplate_;
    };
  };
};

#endif