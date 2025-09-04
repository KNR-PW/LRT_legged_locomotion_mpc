//
// Created by rgrandia on 13.03.20.
// Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2) on 02.09.2025 
//

#include <legged_locomotion_mpc/locomotion/SwingTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/common/AccessHelperFunctions.hpp>
#include <legged_locomotion_mpc/common/Utils.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <ocs2_core/misc/Lookup.h>
#include <ocs2_core/misc/LoadData.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {
    using namespace ocs2;
    using namespace terrain_model;
    using namespace floating_base_model;

    SwingTrajectoryPlanner::SwingTrajectoryPlanner(FloatingBaseModelInfo info,
      StaticSettings staticSettings,
      DynamicSettings initDynamicSettings,
      const forwardKinematics &kinematicsModel): modelInfo_(std::move(info)),
        staticSettings_(std::move(staticSettings)), 
        dynamicSettings_(std::move(initDynamicSettings)),
        kinematicsModel_(kinematicsModel)
    {
      lastContacts_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);
      feetNormalTrajectories_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);
      feetNormalTrajectoriesEvents_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);

      nominalFootholdsPerLeg_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);
      heuristicFootholdsPerLeg_.resize(modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts);
    }

    void SwingTrajectoryPlanner::updateTerrain(std::unique_ptr<TerrainModel> terrainModel) 
    {
      terrainModel_ = std::move(terrainModel);
    }
    
    void SwingTrajectoryPlanner::updateDynamicSettings(const DynamicSettings& newDynamicSettings)
    {
      dynamicSettings_ = newDynamicSettings;
    }

    const SignedDistanceField* SwingTrajectoryPlanner::getSignedDistanceField() const 
    {
      if (terrainModel_) 
      {
        return terrainModel_->getSignedDistanceField();
      } 
      else 
      {
        return nullptr;
      }
    }

    const ocs2::TargetTrajectories& SwingTrajectoryPlanner::getTargetTrajectories() const
    { 
      return internalTargetTrajectories_; 
    }

    void SwingTrajectoryPlanner::updateSwingMotions(scalar_t initTime, scalar_t finalTime,
     const ocs2::vector_t& currentState, const TargetTrajectories& targetTrajectories,
      const ModeSchedule& modeSchedule) 
    {
      if (!terrainModel_) 
      {
        throw std::runtime_error("[SwingTrajectoryPlanner] terrain cannot be null. " 
          "Update the terrain before planning swing motions");
      }

      const std::vector<std::vector<ContactTiming>> contactTimingsPerLeg =
        extractContactTimingsPerLeg(modeSchedule);

      const auto [optimState, optimInput] = utils::robotStateToOptimizationStateAndInput(currentState);
      const auto feetPositions = kinematicsModel_->getPosition(optimState);

      size_t numEndEffectors = modelInfo_.numThreeDofContacts + modelInfo_.numSixDofContacts;

      for (int i = 0; i < numEndEffectors; ++i) 
      {
        const auto &contactTimings = contactTimingsPerLeg[i];

        // Update last contacts
        if (!contactTimings.empty()) 
        {
          scalar_t expectedLiftOffTime;
          if (startsWithStancePhase(contactTimings)) 
          {
            // If currently in contact -> update expected liftoff.
            if (hasEndTime(contactTimings.front())) 
            {
              expectedLiftOffTime = contactTimings.front().end;
            } 
            else 
            {
              // Expected liftoff unknown, set to end of horizon
              expectedLiftOffTime = finalTime;
            }
          } 
          else 
          {
            /**
             * If currently in swing -> verify that liftoff was before the horizon.
             *  If not, assume liftoff happened exactly at initTime 
             */ 
            if (lastContacts_[i].first > initTime) 
            {
              expectedLiftOffTime = initTime;
            }
          }
          updateLastContact(i, expectedLiftOffTime, feetPositions[i], *terrainModel_);
        }

        // Select heuristic footholds.
        heuristicFootholdsPerLeg_[i] = selectHeuristicFootholdsPerLeg(i, contactTimings, 
          targetTrajectories, initTime,
          currentState, finalTime);

        // Select terrain constraints based on the heuristic footholds.
        nominalFootholdsPerLeg_[i] = selectNominalFootholdTerrain(
          i, contactTimings, heuristicFootholdsPerLeg_[i], targetTrajectories,
          initTime, currentState, finalTime, *terrainModel_);

        // Create swing trajectories
        std::tie(feetNormalTrajectoriesEvents_[i], feetNormalTrajectories_[i]) =
          generateSwingTrajectories(i, contactTimings, finalTime);
      }
    }

    const FootPhase &SwingTrajectoryPlanner::getFootPhase(size_t endEffectorIndex, 
      scalar_t time) const 
    {
      const auto timeIndex = lookup::findIndexInTimeArray(
        feetNormalTrajectoriesEvents_[endEffectorIndex], time);
      if(timeIndex == feetNormalTrajectoriesEvents_.size())
      {
        throw std::runtime_error("[SwingTrajectoryPlanner]: Error looking for foot phase!");
      }
      return *feetNormalTrajectories_[endEffectorIndex][timeIndex];
    }

    FootPhasesStamped SwingTrajectoryPlanner::generateSwingTrajectories(size_t endEffectorIndex, 
      const std::vector<ContactTiming> &contactTimings, 
      scalar_t finalTime) const
    {
      std::vector<scalar_t> eventTimes;
      std::vector<std::unique_ptr<FootPhase>> footPhases;

      // First swing phase
      if (startsWithSwingPhase(contactTimings)) 
      {
        SwingPhase::SwingEvent liftOff{lastContacts_[endEffectorIndex].first, 
          settings_.liftOffVelocity, &lastContacts_[endEffectorIndex].second};
        SwingPhase::SwingEvent touchDown = [&] 
        {
          if (touchesDownAtLeastOnce(contactTimings)) 
          {
            return SwingPhase::SwingEvent{contactTimings.front().start,
               settings_.touchDownVelocity, 
               &nominalFootholdsPerLeg_[endEffectorIndex].front().getTerrainPlane()};
          } 
          else 
          {
            return SwingPhase::SwingEvent{finalTime + settings_.referenceExtensionAfterHorizon,
              0.0, nullptr};
          }}();

        SwingPhase::SwingProfile swingProfile = getDefaultSwingProfile();
        applySwingMotionScaling(liftOff, touchDown, swingProfile);

        footPhases.emplace_back(new SwingPhase(liftOff, touchDown, 
          swingProfile, terrainModel_.get()));
      }

        // Loop through contact phases
        for (int i = 0; i < contactTimings.size(); ++i) 
        {
          const auto &currentContactTiming = contactTimings[i];
          const ConvexTerrain &nominalFoothold = nominalFootholdsPerLeg_[endEffectorIndex][i];

          // If phase starts after the horizon, we don't need to plan for it
          if (currentContactTiming.start > finalTime) 
          {
            break;
          }

          // generate contact phase
          if (hasStartTime(currentContactTiming)) 
          {
            eventTimes.push_back(currentContactTiming.start);
          }

          footPhases.emplace_back(new StancePhase(nominalFoothold, settings_.terrainMargin));

          // If contact phase extends beyond the horizon, we can stop planning.
          if (!hasEndTime(currentContactTiming) || currentContactTiming.end > finalTime) 
          {
            break;
          }

          // generate swing phase afterwards
          SwingPhase::SwingEvent liftOff{currentContactTiming.end, 
            settings_.liftOffVelocity, &nominalFoothold.plane};
          SwingPhase::SwingEvent touchDown = [&] 
          {
            const bool nextContactExists = (i + 1) < contactTimings.size();
            if (nextContactExists) 
            {
              return SwingPhase::SwingEvent{contactTimings[i + 1].start, 
                  settings_.touchDownVelocity, 
                  &nominalFootholdsPerLeg_[endEffectorIndex][i + 1].getTerrainPlane()};
            } 
            else 
            {
              return SwingPhase::SwingEvent{finalTime + settings_.referenceExtensionAfterHorizon, 0.0, nullptr};
            }}();

          SwingPhase::SwingProfile swingProfile = getDefaultSwingProfile();
          applySwingMotionScaling(liftOff, touchDown, swingProfile);

          eventTimes.push_back(currentContactTiming.end);
          footPhases.emplace_back(new SwingPhase(liftOff, touchDown, swingProfile, terrainModel_.get()));
        }
      return std::make_pair(eventTimes, std::move(footPhases));
    }

    void SwingTrajectoryPlanner::applySwingMotionScaling(SwingPhase::SwingEvent &liftOff, 
      SwingPhase::SwingEvent &touchDown, 
      SwingPhase::SwingProfile &swingProfile) const 
    {
      const scalar_t scaling = [&]() 
      {
        if (std::isnan(liftOff.time) || std::isnan(touchDown.time)) 
        {
          return 1.0;
        } 
        else 
        {
          return std::min(1.0, (touchDown.time - liftOff.time) / settings_.swingTimeScale);
        }
      }();

      if (scaling < 1.0) 
      {
        liftOff.velocity *= scaling;
        touchDown.velocity *= scaling;
        swingProfile.sdfMidswingMargin = scaling * settings_.sdfMidswingMargin;
        for (auto &node: swingProfile.nodes) 
        {
          node.swingHeight *= scaling;
          node.normalVelocity *= scaling;
        }
      }
    }

    std::vector<vector3_t> selectHeuristicFootholds(size_t endEffectorIndex,
      const std::vector<ContactTiming> &contactTimings,
      const ocs2::TargetTrajectories &targetTrajectories, ocs2::scalar_t initTime,
      const comkino_state_t &currentState, ocs2::scalar_t finalTime) const
    {
      // Zmp preparation : measured state
      const auto initBaseOrientation = legged_locomotion_mpc::
        access_helper_functions::getBaseOrientationZyx(currentState);
      const auto initBaseLinearVelocityInBase =legged_locomotion_mpc::
        access_helper_functions::getBaseLinearVelocity(currentState);
      const matrix3_t initBaseRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        initBaseOrientation);
      const vector3_t initBaseLinearVelocityInWorld = initBaseRotationMatrix 
        * initBaseLinearVelocityInBase;

      // Zmp preparation : desired state
      const vector_t initDesiredState = targetTrajectories.getDesiredState(initTime);
      const auto initBaseDesiredOrientation = floating_base_model::
        access_helper_functions::getBaseOrientationZyx(initDesiredBasePose);
      const auto initBaseDesiredLinearVelocity = floating_base_model::
        access_helper_functions::getBaseLinearVelocity(initDesiredState);
      const matrix3_t initBaseRotationMatrix = getRotationMatrixFromZyxEulerAngles(
      initBaseDesiredOrientation);
      const vector3_t initDesiredBaseLinearVelocityInWorld = initBaseRotationMatrix 
        * initBaseDesiredLinearVelocity;

      /** 
       * Compute zmp / inverted pendulum foot placement offset: 
       * delta p = sqrt(h / g) * (v - v_des) 
       */ 
      scalar_t pendulumFrequency = std::sqrt(dynamicSettings_.invertedPendulumHeight / 9.81);
      scalar_t zmpX = pendulumFrequency * (initBaseLinearVelocityInWorld.x() 
        - initDesiredBaseLinearVelocityInWorld.x());
      scalar_t zmpY = pendulumFrequency * (initBaseLinearVelocityInWorld.y() 
        - initDesiredBaseLinearVelocityInWorld.y());
      const vector3_t zmpReactiveOffset = {zmpX, zmpY, 0.0};

      // Heuristic footholds to fill
      std::vector<vector3_t> heuristicFootholds;

      // Heuristic foothold is equal to current foothold for legs in contact
      if (startsWithStancePhase(contactTimings)) 
      {
        heuristicFootholds.push_back(lastContacts_[endEffectorIndex].second.positionInWorld);
      }

      // For future contact phases, use TargetTrajectories at halve the contact phase
      size_t contactCount = 0;
      for (const auto &contactPhase: contactTimings) 
      {
        if (hasStartTime(contactPhase)) 
        {
          const scalar_t contactEndTime = getContactEndTime(contactPhase, finalTime);
          const scalar_t middleContactTime = 0.5 * (contactEndTime + contactPhase.start);

          // Compute foot position from cost desired trajectory
          const vector_t state = targetTrajectories.getDesiredState(middleContactTime);

          /**
           * IMPORTANT: URDF NEEDS TO HAVE STRAIGHT LEGS WITH ZERO JOINT POSITIONS 
           * IN ORDER TO GET POSITIONS BELOW HIP HERE!!!!
           * ALSO JOINT POSITIONS IN THIS TARGET TRAJECTORIES NEEDS TO BE ZERO!!!
           */
          vector3_t referenceFootholdPositionInWorld = kinematicsModel_->getPosition(state)[endEffectorIndex];

          // Add ZMP offset to the first upcoming foothold.
          if (contactCount == 0) 
          {
            referenceFootholdPositionInWorld += zmpReactiveOffset;
          }

          // One foothold added per contactPhase
          heuristicFootholds.push_back(referenceFootholdPositionInWorld);

          /**
           * Can stop for this end effector if we have processed one contact
           * phase after (or extending across) the horizon
           */ 
          if (contactEndTime > finalTime) 
          {
            break;
          }
        }
          ++contactCount;
      }

      return heuristicFootholds;
    }

    std::vector<ConvexTerrain> SwingTrajectoryPlanner::selectNominalFootholdTerrain(
      size_t endEffectorIndex, const std::vector<ContactTiming> &contactTimings, 
      const std::vector<vector3_t> &heuristicFootholds, 
      const TargetTrajectories &targetTrajectories, scalar_t initTime, 
      const comkino_state_t &currentState, scalar_t finalTime, 
      const TerrainModel &terrainModel) const 
    {
      // Will increment the heuristic each time after selecting a nominalFootholdTerrain
      auto heuristicFootholdIt = heuristicFootholds.cbegin();
      std::vector<ConvexTerrain> nominalFootholdTerrain;

      // Nominal foothold is equal to current foothold for end effectors in contact
      if (startsWithStancePhase(contactTimings)) 
      {
        std::vector<vector2_t> emptyBoundry;
        const auto plane = lastContacts_[endEffectorIndex].second;
        ConvexTerrain convexTerrain(plane, emptyBoundry);
        nominalFootholdTerrain.push_back(convexTerrain);
        ++heuristicFootholdIt; // Skip this heuristic. Use lastContact directly
      }

      // For future contact phases
      for (const auto &contactPhase: contactTimings) 
      {
        if (hasStartTime(contactPhase)) 
        {
          const scalar_t timeTillContact = contactPhase.start - initTime;
          const scalar_t contactEndTime = getContactEndTime(contactPhase, finalTime);
          const scalar_t middleContactTime = 0.5 * (contactEndTime + contactPhase.start);

          // Get previous foothold if there was one at this time
          const FootPhase *previousIterationContact = getFootPhaseIfInContact(endEffectorIndex, 
            middleContactTime);

          if (timeTillContact < staticSettings_.previousFootholdTimeDeadzone 
            && previousIterationContact != nullptr) 
          {
            // Simply copy the information out of the previous iteration
            nominalFootholdTerrain.push_back(*previousIterationContact->nominalFootholdConstraint());
            ++heuristicFootholdIt; // Skip this heuristic. Using the previous terrain instead
          } 
          else 
          {
            // Select the terrain base on the heuristic
            vector3_t referenceFootholdPositionInWorld = *heuristicFootholdIt;

            // Filter w.r.t. previous foothold
            if (previousIterationContact != nullptr) 
            {
              referenceFootholdPositionInWorld = filterFoothold(
                referenceFootholdPositionInWorld, 
                previousIterationContact->nominalFootholdLocation());
            }

              // // Kinematic penalty
              // const base_coordinate_t basePoseAtTouchdown = getBasePose(
              //     targetTrajectories.getDesiredState(contactPhase.start));
              // const auto hipPositionInWorldTouchdown = kinematicsModel_->legRootInOriginFrame(
              //     endEffectorIndex, basePoseAtTouchdown);
              // const auto hipOrientationInWorldTouchdown = kinematicsModel_->orientationLegRootToOriginFrame(
              //     endEffectorIndex, basePoseAtTouchdown);
              // const base_coordinate_t basePoseAtLiftoff = getBasePose(
              //     targetTrajectories.getDesiredState(contactEndTime));
              // const auto hipPositionInWorldLiftoff = kinematicsModel_->legRootInOriginFrame(
              //     endEffectorIndex, basePoseAtLiftoff);
              // const auto hipOrientationInWorldLiftoff = kinematicsModel_->orientationLegRootToOriginFrame(
              //     endEffectorIndex, basePoseAtLiftoff);
              // ApproximateKinematicsConfig config;
              // config.kinematicPenaltyWeight = settings_.legOverExtensionPenalty;
              // config.maxLegExtension = settings_.nominalLegExtension;
              // auto scoringFunction = [&](const vector3_t &footPositionInWorld) 
              // {
              //     return computeKinematicPenalty(footPositionInWorld, hipPositionInWorldTouchdown,
              //                                    hipOrientationInWorldTouchdown, config) +
              //            computeKinematicPenalty(footPositionInWorld, hipPositionInWorldLiftoff,
              //                                    hipOrientationInWorldLiftoff, config);
              // };

            if (contactPhase.start < finalTime) 
            {
              ConvexTerrain convexTerrain = terrainModel.getConvexTerrainAtPositionInWorld(
                  referenceFootholdPositionInWorld);
              nominalFootholdTerrain.push_back(convexTerrain);
              ++heuristicFootholdIt;
            } 
            else 
            {
              // After the horizon -> we are only interested in the position and orientation
              std::vector<vector2_t> emptyBoundry;
              const auto plane = terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(
                referenceFootholdPositionInWorld);
              ConvexTerrain convexTerrain(plane, emptyBoundry);
              nominalFootholdTerrain.push_back(convexTerrain);
              ++heuristicFootholdIt;
            }
          }

          /** 
           * Can stop for this end effector if we have processed one contact 
           * phase after (or extending across) the horizon
           */
          if (contactEndTime > finalTime) 
          {
            break;
          }
        }
      }

      return nominalFootholdTerrain;
    }

    // void SwingTrajectoryPlanner::adaptJointReferencesWithInverseKinematics(scalar_t finalTime) {
    //     const scalar_t damping = 0.01; // Quite some damping on the IK to get well conditions references.

    //     for (int k = 0; k < internalTargetTrajectories_.timeTrajectory.size(); ++k) {
    //         const scalar_t t = internalTargetTrajectories_.timeTrajectory[k];

    //         const base_coordinate_t basePose = getBasePose(comkino_state_t(internalTargetTrajectories_.stateTrajectory[k]));
    //         const vector3_t basePositionInWorld = getPositionInOrigin(basePose);
    //         const vector3_t eulerXYZ = getOrientation(basePose);

    //         for (size_t endEffectorIndex = 0; endEffectorIndex < NUM_CONTACT_POINTS; ++endEffectorIndex) {
    //             const auto &footPhase = this->getFootPhase(endEffectorIndex, t);

    //             // Joint positions
    //             const vector3_t positionBaseToFootInWorldFrame = footPhase.getPositionInWorld(t) - basePositionInWorld;
    //             const vector3_t positionBaseToFootInBaseFrame = rotateVectorOriginToBase(
    //                 positionBaseToFootInWorldFrame, eulerXYZ);

    //             const size_t stateOffset = 2 * BASE_COORDINATE_SIZE + 3 * endEffectorIndex;
    //             internalTargetTrajectories_.stateTrajectory[k].segment(stateOffset, 3) =
    //                     inverseKinematicsModelPtr_->getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
    //                         endEffectorIndex, positionBaseToFootInBaseFrame);

    //             // Joint velocities
    //             auto jointPositions = getJointPositions(internalTargetTrajectories_.stateTrajectory[k]);
    //             auto baseTwistInBaseFrame = getBaseLocalVelocities(internalTargetTrajectories_.stateTrajectory[k]);

    //             const vector3_t b_baseToFoot = kinematicsModel_->positionBaseToFootInBaseFrame(endEffectorIndex, jointPositions);
    //             const vector3_t footVelocityInBaseFrame = rotateVectorOriginToBase(
    //                 footPhase.getVelocityInWorld(t), eulerXYZ);
    //             const vector3_t footRelativeVelocityInBaseFrame =
    //                     footVelocityInBaseFrame - getLinearVelocity(baseTwistInBaseFrame) - getAngularVelocity(
    //                         baseTwistInBaseFrame).cross(b_baseToFoot);

    //             const size_t inputOffset = 3 * NUM_CONTACT_POINTS + 3 * endEffectorIndex;
    //             internalTargetTrajectories_.inputTrajectory[k].segment(inputOffset, 3) =
    //                     inverseKinematicsModelPtr_->getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(
    //                         endEffectorIndex, footRelativeVelocityInBaseFrame,
    //                         kinematicsModel_->baseToFootJacobianBlockInBaseFrame(endEffectorIndex, jointPositions), damping);
    //         }

    //         // Can stop adaptation as soon as we have processed a point beyond the horizon.
    //         if (t > finalTime) {
    //             break;
    //         }
    //     }
    // }

    void SwingTrajectoryPlanner::updateLastContact(size_t endEffectorIndex, scalar_t expectedLiftOff,
      const vector3_t &currentFootPosition,
      const TerrainModel &terrainModel) 
    {
      // Get orientation from terrain model, position from the kinematics
      auto lastContactTerrain = terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(currentFootPosition);
      lastContactTerrain.positionInWorld = currentFootPosition;
      lastContacts_[legIndex] = {expectedLiftOff, lastContactTerrain};
    }

    SwingPhase::SwingProfile SwingTrajectoryPlanner::getDefaultSwingProfile() const 
    {
      SwingPhase::SwingProfile defaultSwingProfile;
      defaultSwingProfile.sdfMidswingMargin = settings_.sdfMidswingMargin;
      defaultSwingProfile.maxSwingHeightAdaptation = 2.0 * settings_.swingHeight;

      SwingPhase::SwingProfile::Node midPoint;
      midPoint.phase = 0.5;
      midPoint.swingHeight = settings_.swingHeight;
      midPoint.normalVelocity = 0.0;
      midPoint.tangentialProgress = 0.6;
      midPoint.tangentialVelocityFactor = 2.0;
      defaultSwingProfile.nodes.push_back(midPoint);
      return defaultSwingProfile;
    }

    scalar_t SwingTrajectoryPlanner::getContactEndTime(const ContactTiming &contactPhase, 
      scalar_t finalTime) const 
    {
      return hasEndTime(contactPhase) ? contactPhase.end 
        : std::max(finalTime + settings_.referenceExtensionAfterHorizon, contactPhase.start);
    }

    const FootPhase *SwingTrajectoryPlanner::getFootPhaseIfInContact(size_t endEffectorIndex, 
      scalar_t time) const 
    {
      const FootPhase *previousIterationContact = nullptr;
      if (!feetNormalTrajectories_[endEffectorIndex].empty()) 
      {
        const auto &footPhase = getFootPhase(endEffectorIndex, time);
        if (footPhase.contactFlag()) 
        {
          previousIterationContact = &footPhase;
        }
      }
      return previousIterationContact;
    }

    vector3_t SwingTrajectoryPlanner::filterFoothold(const vector3_t &newFoothold,
      const vector3_t &previousFoothold) const 
    {
      // Apply Position deadzone and low pass filter
      if ((newFoothold - previousFoothold).norm() < settings_.previousFootholdDeadzone) 
      {
        return previousFoothold;
      } 
      else 
      {
        // low pass filter
        const scalar_t lambda = settings_.previousFootholdFactor;
        return lambda * previousFoothold + (1.0 - lambda) * newFoothold;
      }
    }

    std::vector<ConvexTerrain> SwingTrajectoryPlanner::getNominalFootholds(
      size_t endEffectorIndex) const 
    { 
      return nominalFootholdsPerLeg_[endEffectorIndex]; 
    }

    std::vector<vector3_t> SwingTrajectoryPlanner::getHeuristicFootholds(
      size_t endEffectorIndex) const 
    { 
      return heuristicFootholdsPerLeg_[endEffectorIndex]; 
    }
   
    const StaticSettings& SwingTrajectoryPlanner::getStaticSettings() const 
    { 
      return staticSettings_; 
    }
        
    const DynamicSettings& SwingTrajectoryPlanner::getDynamicSettings() const 
    { 
      return dynamicSettings_; 
    }

     SwingTrajectoryPlanner::StaticSettings loadSwingStaticTrajectorySettings(
      const std::string &filename, bool verbose = true)
    {
      SwingTrajectoryPlanner::StaticSettings settings{};

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      const std::string prefix{"model_settings.static_swing_trajectory_settings."};

      if (verbose) 
      {
        std::cerr << "\n #### Static Swing trajectory Settings:" << std::endl;
        std::cerr << " #### ==================================================" << std::endl;
      }

      loadData::loadPtreeValue(pt, settings.liftOffVelocity, 
        prefix + "liftOffVelocity", verbose);
      loadData::loadPtreeValue(pt, settings.touchDownVelocity, 
        prefix + "touchDownVelocity", verbose);
      loadData::loadPtreeValue(pt, settings.swingHeight, 
        prefix + "swingHeight", verbose);
      loadData::loadPtreeValue(pt, settings.errorGain, 
        prefix + "errorGain", verbose);
      loadData::loadPtreeValue(pt, settings.swingTimeScale, 
        prefix + "swingTimeScale", verbose);
      loadData::loadPtreeValue(pt, settings.sdfMidswingMargin, 
        prefix + "sdfMidswingMargin", verbose);
      loadData::loadPtreeValue(pt, settings.terrainMargin, 
        prefix + "terrainMargin", verbose);
      loadData::loadPtreeValue(pt, settings.previousFootholdFactor, 
        prefix + "previousFootholdFactor", verbose);
      loadData::loadPtreeValue(pt, settings.previousFootholdDeadzone, 
        prefix + "previousFootholdDeadzone",
        verbose);
      loadData::loadPtreeValue(pt, settings.previousFootholdTimeDeadzone, 
        prefix + "previousFootholdTimeDeadzone", verbose);
      loadData::loadPtreeValue(pt, settings.nominalLegExtension, 
        prefix + "nominalLegExtension", verbose);
      loadData::loadPtreeValue(pt, settings.legOverExtensionPenalty, 
        prefix + "legOverExtensionPenalty", verbose);
      loadData::loadPtreeValue(pt, settings.referenceExtensionAfterHorizon, 
        prefix + "referenceExtensionAfterHorizon", verbose);
      loadData::loadPtreeValue(pt, settings.maximumReferenceSampleTime, 
        prefix + "maximumReferenceSampleTime", verbose);
      loadData::loadPtreeValue(pt, settings.swingTrajectoryFromReference, 
        prefix + "swingTrajectoryFromReference", verbose);

      if (verbose) 
      {
        std::cerr << " #### ==================================================" << std::endl;
      }

      return settings;
    }
    SwingTrajectoryPlanner::DynamicSettings loadSwingDynamicTrajectorySettings(
      const std::string &filename, bool verbose = true)
    {
      SwingTrajectoryPlanner::DynamicSettings settings{};

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      const std::string prefix{"model_settings.dynamic_swing_trajectory_settings."};

      if (verbose) 
      {
        std::cerr << "\n #### Dynamic Swing trajectory Settings:" << std::endl;
        std::cerr << " #### ==================================================" << std::endl;
      }

      loadData::loadPtreeValue(pt, settings.invertedPendulumHeight, 
        prefix + "invertedPendulumHeight", verbose);

      if (verbose) 
      {
        std::cerr << " #### ==================================================" << std::endl;
      }

      return settings;
    }
  } // namespace locomotion
} // namespace legged_locomotion_mpc
  