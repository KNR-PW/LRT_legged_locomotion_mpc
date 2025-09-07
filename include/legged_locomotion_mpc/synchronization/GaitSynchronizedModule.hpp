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

#ifndef __GAIT_SYNCHRONIZED_MODULE_LEGGED_LOCOMOTION_MPC__
#define __GAIT_SYNCHRONIZED_MODULE_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace legged_locomotion_mpc 
{
  namespace synchronization
  {
    class GaitSynchronizedModule: public ocs2::SolverSynchronizedModule 
    {
      public:

        GaitSynchronizedModule(ocs2::Synchronized<locomotion::GaitPlanner>& gaitPlanner);

        void preSolverRun(ocs2::scalar_t initTime, ocs2::scalar_t finalTime,
          const ocs2::vector_t& initState,
          const ocs2::ReferenceManagerInterface& referenceManager) override;

        void postSolverRun(const ocs2::PrimalSolution& primalSolution) override;

        void updateDynamicParameters(ocs2::scalar_t time, 
          const locomotion::GaitDynamicParameters& dynamicParams);

      private:

        locomotion::GaitDynamicParameters newDynamicParameters_;

        ocs2::scalar_t newParametersTime_;

        ocs2::Synchronized<locomotion::GaitPlanner>* gaitPlannerPtr_;

        std::atomic_bool parametersUpdated_;

        std::mutex receivedDynamicParametersMutex_; 
    };
  }; // namespace synchronization
}; // namespace legged_locomotion_mpc 

#endif
