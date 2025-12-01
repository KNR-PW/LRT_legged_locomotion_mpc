//
// Created by rgrandia on 13.03.20.
// Modified by Bartłomiej Krajewski (://github.com/BartlomiejK2) on 01.09.2025 
//

#ifndef __SWING_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__
#define __SWING_TRAJECTORY_PLANNER_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/locomotion/SingleLegLogic.hpp>
#include <legged_locomotion_mpc/locomotion/FootPhase.hpp>

#include <terrain_model/core/ConvexTerrain.hpp>
#include <terrain_model/core/TerrainModel.hpp>
#include <terrain_model/core/TerrainPlane.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    class SwingTrajectoryPlanner 
    {
      public:

        using forwardKinematics = legged_locomotion_mpc::PinocchioForwardEndEffectorKinematicsCppAd;

        struct StaticSettings 
        {
          /** Velocity at the start of swing */
          ocs2::scalar_t liftOffVelocity = 0.0;

          /** Velocity at the end of swing */
          ocs2::scalar_t touchDownVelocity = 0.0;

          /** Maximum height of foot at midpoint of swing phase */
          ocs2::scalar_t swingHeight = 0.075;
          
          /** 
           * Proportional gain for returning to the planned swing trajectory.
           * 10-90%-rise_time ~= 2.2 / errorGain
           * Alternatively can be measured as:
           * (velocity feedback) / (tracking error) ([m/s] / [m])
           */ 
          ocs2::scalar_t errorGain = 0.0;
          
          /** Swing phases shorter than this time will be scaled down in height and velocity */ 
          ocs2::scalar_t swingTimeScale = 0.15;

          /** Desired sdf based clearance in the middle of the swing phase [m] */
          ocs2::scalar_t sdfMidswingMargin = 0.0; 

          /** Shrinkage of the convex terrain constrains in [m] */
          ocs2::scalar_t terrainMargin = 0.0; 
          
          /** Factor in [0, 1] with which to take previous foothold into account */ 
          ocs2::scalar_t previousFootholdFactor = 0.0;

          /** Previous foothold is taken if the new reference is within this threshold. [m] */
          ocs2::scalar_t previousFootholdDeadzone = 0.0;

          /** Previous foothold is taken if the contact phase is starting withing this time. [s] */
          ocs2::scalar_t previousFootholdTimeDeadzone = 0.0; 
          
          /** Leg extension beyond this length [m] will be penalized in terrain selection */
          ocs2::scalar_t nominalLegExtension = 0.55;

          /** Weight of the leg overextension penalty */
          ocs2::scalar_t legOverExtensionPenalty = 5.0; 
          
          /** 
           * Base and foot references generated for this amount of seconds
           * after the horizon ends. 
           */
          ocs2::scalar_t referenceExtensionAfterHorizon = 0.2;
        };

        struct DynamicSettings
        {
          /** Height used for the inverted pendulum foothold adjustment */
          ocs2::scalar_t invertedPendulumHeight = 0.35;
        };

        SwingTrajectoryPlanner(floating_base_model::FloatingBaseModelInfo info,
          StaticSettings staticSettings,
          DynamicSettings initDynamicSettings,
          const forwardKinematics &kinematicsModel);

        /** Update terrain model */
        void updateTerrain(const terrain_model::TerrainModel& terrainModel);
        
        /** Update dynamic settings */
        void updateDynamicSettings(const DynamicSettings& newDynamicSettings);

        /** Access the SDF of the current terrain model */ 
        const terrain_model::SignedDistanceField* getSignedDistanceField() const;

        /** 
         * Main interface preparing all swing trajectories in cartesian
         * space (called by reference manager). 
         * Target trajectories should be subsampled already.
         * */ 
        void updateSwingMotions(ocs2::scalar_t initTime,
          ocs2::scalar_t finalTime, const state_vector_t& currentState,
          const ocs2::TargetTrajectories& targetTrajectories,
          const ocs2::ModeSchedule& modeSchedule);

        /** Main access method for the generated cartesian references. */ 
        const FootPhase& getFootPhase(size_t endEffectorIndex, ocs2::scalar_t time) const;

        /** Main access method for generated cartesian reference positions */
        std::vector<vector3_t> getEndEffectorPositions(ocs2::scalar_t time) const;

        /** Main access method for generated cartesian reference velocities */
        std::vector<vector3_t> getEndEffectorVelocities(ocs2::scalar_t time) const;

        /** Main access method for generated cartesian reference terrain clerance */
        std::vector<ocs2::scalar_t> getEndEffectorClearances(ocs2::scalar_t time) const;
        
        /** 
         * Main access method for generated cartesian reference position trajectories 
         * 3D position of leg i in time (index) t: pos[t][i]
         */
        using position_trajectories = std::vector<std::vector<vector3_t>>;
        position_trajectories getEndEffectorPositionTrajectories(
          std::vector<ocs2::scalar_t> times) const;

        /** 
         * Main access method for generated cartesian reference velocity trajectories 
         * 3D velocity of leg i in time (index) t: vel[t][i]
         */
        using velocity_trajectories = std::vector<std::vector<vector3_t>>;
        velocity_trajectories getEndEffectorVelocityTrajectories(
          std::vector<ocs2::scalar_t> times) const;
        
        /** 
         * Main access method for generated cartesian reference terrain clearance 
         * trajectories of leg i in time (index) t: vel[t][i]
         */
        using foot_clearance_trajectory = std::vector<std::vector<ocs2::scalar_t>>;
        foot_clearance_trajectory getEndEffectorClearanceTrajectories(
          std::vector<ocs2::scalar_t> times) const;
        
        /**
         * Complete foot trajectories for single timestamp
         */
        struct EndEffectorTrajectoriesPoint
        {
          std::vector<vector3_t> positions;
          std::vector<vector3_t> velocities;
          std::vector<ocs2::scalar_t> clearances;
        };

        /**
         * Complete foot trajectories for all timestamps
         */
        struct EndEffectorTrajectories
        {
          position_trajectories positions;
          velocity_trajectories velocities;
          foot_clearance_trajectory clearances;
        };

        /** Main access method for generated end effector trajectory point */
        EndEffectorTrajectoriesPoint getEndEffectorTrajectoryPoint(
          ocs2::scalar_t time) const;

        /** 
         * Main, optimized version of upper 3 algorithms for generating reference foot
         * trajectories. Access of leg i in time (index) t: points[t] -> pos[i], vel[i], cl[i]
         */
        EndEffectorTrajectories getEndEffectorTrajectories(
          std::vector<ocs2::scalar_t> times) const;

        struct FootTangentialConstraintTrajectories
        {
          /**
           * Times where at least one constraint matrix changes 
           */
          std::vector<ocs2::scalar_t> times;
          /**
           * Vector with constraints matrixes.
           * Get constraint matrix of leg i in time (index) t: constraints[t][i]
           */
          std::vector<std::vector<FootTangentialConstraintMatrix>> constraints;
        };

        FootTangentialConstraintTrajectories getFootTangentialConstraintTrajectories(
          std::vector<contact_flags_t> contactFlags, std::vector<ocs2::scalar_t> times);

        /** Accessed by the controller for visualization */
        std::vector<terrain_model::ConvexTerrain> getNominalFootholds(
          size_t endEffectorIndex) const;

        /** Accessed by the controller for visualization */
        std::vector<vector3_t> getHeuristicFootholds(size_t endEffectorIndex) const;

        /** Read static settings */
        const StaticSettings& getStaticSettings() const;

        /** Read dynamic settings */
        const DynamicSettings& getDynamicSettings() const;

      private:

        void updateLastContact(size_t endEffectorIndex, ocs2::scalar_t expectedLiftOff,
          const vector3_t& currentFootPosition, const terrain_model::TerrainModel& terrainModel);
        
        using FootPhasesStamped =  std::pair<std::vector<ocs2::scalar_t>, std::vector<std::unique_ptr<FootPhase>>>; 
        FootPhasesStamped generateSwingTrajectories(size_t endEffectorIndex, 
          const std::vector<ContactTiming> &contactTimings, ocs2::scalar_t finalTime) const;

        std::vector<vector3_t> selectHeuristicFootholds(size_t endEffectorIndex,
          const std::vector<ContactTiming> &contactTimings,
          const ocs2::TargetTrajectories &targetTrajectories, ocs2::scalar_t initTime,
          const state_vector_t& currentState, ocs2::scalar_t finalTime) const;

        std::vector<terrain_model::ConvexTerrain> selectNominalFootholdTerrain(
          size_t endEffectorIndex, const std::vector<ContactTiming> &contactTimings,
          const std::vector<vector3_t> &heuristicFootholds,
          const ocs2::TargetTrajectories &targetTrajectories, ocs2::scalar_t initTime,
          const state_vector_t &currentState, ocs2::scalar_t finalTime,
          const terrain_model::TerrainModel &terrainModel) const;

        void applySwingMotionScaling(SwingPhase::SwingEvent &liftOff, 
          SwingPhase::SwingEvent &touchDown, SwingPhase::SwingProfile &swingProfile) const;

        SwingPhase::SwingProfile getDefaultSwingProfile() const;

        ocs2::scalar_t getContactEndTime(const ContactTiming &contactPhase,
          ocs2::scalar_t finalTime) const;

        const FootPhase* getFootPhaseIfInContact(size_t endEffectorIndex, 
          ocs2::scalar_t time) const;

        vector3_t filterFoothold(const vector3_t &newFoothold,
           const vector3_t &previousFoothold) const;

        const floating_base_model::FloatingBaseModelInfo modelInfo_;
        const StaticSettings staticSettings_;
        DynamicSettings dynamicSettings_;
        const forwardKinematics* kinematicsModel_;

        std::vector<std::pair<ocs2::scalar_t, terrain_model::TerrainPlane>> lastContacts_;
        std::vector<std::vector<std::unique_ptr<FootPhase>>> feetNormalTrajectories_;
        std::vector<std::vector<ocs2::scalar_t>> feetNormalTrajectoriesEvents_;

        std::vector<std::vector<terrain_model::ConvexTerrain>> nominalFootholdsPerLeg_;
        std::vector<std::vector<vector3_t>> heuristicFootholdsPerLeg_;
        const terrain_model::TerrainModel* terrainModel_;
    };

    /** Load static and dynamic settings from file */
    SwingTrajectoryPlanner::StaticSettings loadSwingStaticTrajectorySettings(
      const std::string &filename, bool verbose = true);
    SwingTrajectoryPlanner::DynamicSettings loadSwingDynamicTrajectorySettings(
      const std::string &filename, bool verbose = true);
  } // namespace locomotion
} //  namespace legged_locomotion_mpc

#endif