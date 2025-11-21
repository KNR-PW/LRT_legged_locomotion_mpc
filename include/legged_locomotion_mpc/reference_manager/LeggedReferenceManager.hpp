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


#ifndef __LEGGED_REFERENCE_MANAGER_LEGGED_LOCOMOTION_MPC__
#define __LEGGED_REFERENCE_MANAGER_LEGGED_LOCOMOTION_MPC__

#include <future>

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_core/thread_support/BufferedPointer.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <legged_locomotion_mpc/common/Types.hpp>

#include <legged_locomotion_mpc/locomotion/GaitPlanner.hpp>
#include <legged_locomotion_mpc/locomotion/SwingTrajectoryPlanner.hpp>

#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/trajectory_planners/JointTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/trajectory_planners/ContactForceWrenchTrajectoryPlanner.hpp>

namespace legged_locomotion_mpc
{
  /**
  * Manages the ModeSchedule and the TargetTrajectories for switched model.
  */
  class LeggedReferenceManager: public ocs2::ReferenceManager
  {
    public:

    struct Settings
    {
      ocs2::scalar_t maximumReferenceSampleInterval = 0.05;
    };
  
    LeggedReferenceManager(LeggedReferenceManager::Settings settings,
      std::shared_ptr<locomotion::GaitPlanner> gaitPlannerPtr,
      std::shared_ptr<locomotion::SwingTrajectoryPlanner> swingTrajectoryPtr,
      std::shared_ptr<planners::BaseTrajectoryPlanner> baseTrajectoryPtr,
      std::shared_ptr<planners::JointTrajectoryPlanner> jointTrajectoryPtr,
      std::shared_ptr<planners::ContactForceWrenchTrajectoryPlanner> forceTrajectoryPtr);

    ~LeggedReferenceManager() override;

    void initalize(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, 
      const state_vector_t& currenState, const contact_flags_t& currentContactFlags,
      locomotion::GaitDynamicParameters&& currentGaitParameters,
      std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel);

    void updateState(const state_vector_t& currenState);

    void updateContactFlags(const contact_flags_t& currentContactFlags);

    void updateGaitParemeters(
      locomotion::GaitDynamicParameters&& currentGaitParameters);

    void updateTerrainModel(
      std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel);

    void preSolverRun(ocs2::scalar_t initTime, ocs2::scalar_t finalTime, 
      const ocs2::vector_t& initState) override;

    const contact_flags_t& getContactFlags(ocs2::scalar_t time) const;

    const terrain_model::TerrainModel& getTerrainModel() const;

    private:
    
      void generateNewTargetTrajectories(ocs2::scalar_t initTime, ocs2::scalar_t finalTime);

      Settings settings_;

      std::future<void> newTrajectories_;

      ocs2::BufferedValue<state_vector_t> currentState_;
      ocs2::BufferedValue<contact_flags_t> currentContactFlags_;
      ocs2::BufferedValue<locomotion::GaitDynamicParameters> currentGaitParameters_;
      ocs2::BufferedValue<planners::BaseTrajectoryPlanner::BaseReferenceCommand> currentCommand_;
      
      std::unique_ptr<terrain_model::TerrainModel> currentTerrainModel_;
      ocs2::BufferedPointer<terrain_model::TerrainModel> bufferedTerrainModel_;

      std::shared_ptr<locomotion::GaitPlanner> gaitPlannerPtr_;
      std::shared_ptr<locomotion::SwingTrajectoryPlanner> swingTrajectoryPtr_;
      std::shared_ptr<planners::BaseTrajectoryPlanner> baseTrajectoryPtr_;
      std::shared_ptr<planners::JointTrajectoryPlanner> jointTrajectoryPtr_;
      std::shared_ptr<planners::ContactForceWrenchTrajectoryPlanner> forceTrajectoryPtr_;
  };
} // namespace legged_locomotion_mpc

#endif
