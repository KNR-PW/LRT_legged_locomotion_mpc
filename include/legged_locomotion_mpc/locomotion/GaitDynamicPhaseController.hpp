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


#ifndef __GAIT_DYNAMIC_PHASE_LEGGED_LOCOMOTION_MPC__
#define __GAIT_DYNAMIC_PHASE_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/GaitParameters.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    class GaitDynamicPhaseController
    {
      public:
        GaitDynamicPhaseController(ocs2::scalar_t initPhase, ocs2::scalar_t initTime,
          const GaitStaticParameters& initStaticParams,
          const GaitDynamicParameters& initDynamicParams);

        /**
         * Get phase values at given query time
         */
        std::vector<ocs2::scalar_t> getPhasesAtTime(ocs2::scalar_t time);

        /**
         * Get contact flags at given query time
         */
        contact_flags_t getContactFlagsAtTime(ocs2::scalar_t time);

        /**
         * Remove event times, frequencies and offsets before query time
         */
        void remove(ocs2::scalar_t time);

        /**
         * Add new frequency and offsets, starting from query time
         */
        void update(ocs2::scalar_t newTime, const GaitDynamicParameters& newDynamicParams);
        
      private:

        GaitStaticParameters staticParams_;

        /**
         * 
         *  Frequencies are counted as follows:
         *          ------ | ------ | ------ | ...  ------ | ------ | ------
         *  time:         t[0]     t[1]     t[2]       t[n-1]     t[n]
         *  frequency:        f[0]     f[1]     f[2] ...    f[n-1]     f[n]
         *  offsets:        o[0]][:]  o[1]][:] o[2]][:]   o[n-1]][:]  o[n]][:]
         *  swing ratio:      s[0]     s[1]     s[2]        s[n-1]     s[n]
         *
         * Where t[0] should have same value as ModeSchedule.eventTimes[0]!
         */
        std::vector<ocs2::scalar_t> eventTimes_;
        std::vector<GaitDynamicParameters> dynamicParamsVec_;
        
        /**
         * Phase value at time t[0] 
         */
        ocs2::scalar_t currentPhase_;
    };
  }; // namespace locomotion
}; // namespace legged_locomotion_mpc

#endif