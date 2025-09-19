
#include <legged_locomotion_mpc/locomotion/BaseTrajectoryPlanner.hpp>

#include <algorithm>

#include <terrain_model/planar_model/PlanarFactoryFunctions.hpp>

#include <legged_locomotion_mpc/common/AccessHelperFunctions.hpp>

#include <ocs2_core/misc/LinearInterpolation.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>

namespace legged_locomotion_mpc
{
  namespace locomotion
  {

    using namespace ocs2;
    using namespace terrain_model;
    using namespace floating_base_model;

    void addVelocitiesFromFiniteDifference(
      BaseTrajectoryPlanner::BaseReferenceTrajectory& baseReference) 
    {
      const size_t referenceSize = baseReference.time.size();
      if (referenceSize <= 1) 
      {
        return;
      }

      baseReference.linearVelocityInWorld.clear();
      baseReference.angularVelocityInWorld.clear();
      baseReference.linearVelocityInWorld.reserve(referenceSize);
      baseReference.angularVelocityInWorld.reserve(referenceSize);

      for (size_t i = 0; (i + 1) < referenceSize; ++i) 
      {
        scalar_t inv_dt = 1 / (baseReference.time[i + 1] - baseReference.time[i]);
        const matrix3_t newRotationMatrix = getRotationMatrixFromZyxEulerAngles(baseReference.eulerZyx[i + 1]);
        const matrix3_t oldRotationMatrix = getRotationMatrixFromZyxEulerAngles(baseReference.eulerZyx[i]);
        baseReference.angularVelocityInWorld.push_back(
          rotationErrorInWorld(newRotationMatrix, oldRotationMatrix) * inv_dt);
        baseReference.linearVelocityInWorld.push_back(
          (baseReference.positionInWorld[i + 1] - baseReference.positionInWorld[i]) * inv_dt);
      }

      // Add last velocities same as previous ones
      baseReference.angularVelocityInWorld.push_back(baseReference.angularVelocityInWorld.back());
      baseReference.linearVelocityInWorld.push_back(baseReference.linearVelocityInWorld.back());
    }

    matrix3_t projectRotationMatrixOnPlane(const matrix3_t& rotationMatrixToWorld, const TerrainPlane& plane)
    {
      const vector3_t headingVector = rotationMatrixToWorld.col(0);
      const vector3_t xAxis = plane.projectVectorInWorldOntoPlaneAlongGravity(headingVector).normalized();
      const vector3_t zAxis = plane.getSurfaceNormalInWorld();
      const vector3_t yAxis = zAxis.cross(xAxis);

      matrix3_t projectedRotationMatrix;
      projectedRotationMatrix.col(0) = xAxis;
      projectedRotationMatrix.col(1) = yAxis;
      projectedRotationMatrix.col(2) = zAxis;
        
      return projectedRotationMatrix;
    }

    vector3_t projectEulerZyxToFrame(const vector3_t& eulerZyx, 
      const TerrainPlane& plane)
    {
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(eulerZyx);

      const matrix3_t projectedRotation = projectRotationMatrixOnPlane(rotationMatrix, 
        plane);
        
      vector3_t newEuler = quaterion_euler_transforms::getEulerAnglesFromRotationMatrix(
        projectedRotation);

      makeEulerAnglesUnique(newEuler);
      newEuler.x() = moduloAngleWithReference(newEuler.x(), eulerZyx.x());

      return newEuler;
    }

    vector2_t rotateVector2D(const vector2_t& vector, scalar_t angle)
    {
      vector2_t returnVector;
      const scalar_t sinValue = sin(angle);
      const scalar_t cosValue = cos(angle);
      returnVector.x() = vector.x() * cosValue - vector.y() * sinValue;
      returnVector.y() = vector.x() * sinValue + vector.y() * cosValue;
      return returnVector;
    }

    BaseTrajectoryPlanner::BaseTrajectoryPlanner(FloatingBaseModelInfo modelInfo,
      BaseTrajectoryPlanner::StaticSettings settings): 
        modelInfo_(std::move(modelInfo)), settings_(std::move(settings))
    {
      currentBaseHeight_ = settings.initialBaseHeight;
    }
      
    void BaseTrajectoryPlanner::updateTerrain(const terrain_model::TerrainModel& terrainModel)
    {
      terrainModel_ = &terrainModel;
    }

    void BaseTrajectoryPlanner::updateTargetTrajectory(scalar_t initTime,
      scalar_t finalTime, const BaseTrajectoryPlanner::BaseReferenceCommand& command,
      const state_vector_t& initialState,
      TargetTrajectories& targetTrajectories)
    {
      BaseTrajectoryPlanner::BaseReferenceTrajectory baseReference = generateExtrapolatedBaseReference(
        initTime, finalTime, initialState, command);
      
      const size_t referenceSize = baseReference.time.size();
      targetTrajectories.timeTrajectory = std::move(baseReference.time);
      
      for(size_t i = 0; i < referenceSize; ++i)
      {
        vector_t& currentState = targetTrajectories.stateTrajectory[i];
        using namespace floating_base_model::access_helper_functions;
        getBasePosition(currentState, modelInfo_) = 
          baseReference.positionInWorld[i];
        getBaseOrientationZyx(currentState, modelInfo_) = 
          baseReference.eulerZyx[i];

        const matrix3_t rotationMatrixToBase = getRotationMatrixFromZyxEulerAngles(
          baseReference.eulerZyx[i]).transpose();
        getBaseLinearVelocity(currentState, modelInfo_) = 
          rotationMatrixToBase * baseReference.linearVelocityInWorld[i];
        getBaseAngularVelocity(currentState, modelInfo_) = 
          rotationMatrixToBase * baseReference.angularVelocityInWorld[i];
      }
    }

    vector2_t BaseTrajectoryPlanner::map2DVelocityCommandToWorld(
      scalar_t headingVelocity, scalar_t lateralVelocity, scalar_t yaw)
    {
      return rotateVector2D({headingVelocity, lateralVelocity}, yaw);
    }

    BaseTrajectoryPlanner::BaseFlatReferenceTrajectory BaseTrajectoryPlanner::generate2DExtrapolatedBaseReference(
      scalar_t initTime, scalar_t finalTime, const state_vector_t& initialState,
      const BaseTrajectoryPlanner::BaseReferenceCommand& command)
    {
      const size_t referenceSize = (finalTime - initTime) / settings_.deltaTime + 1;

      BaseTrajectoryPlanner::BaseFlatReferenceTrajectory reference;
      reference.time.reserve(referenceSize);
      reference.positionInWorld.reserve(referenceSize);
      reference.yaw.reserve(referenceSize);
      
      using namespace legged_locomotion_mpc::access_helper_functions;

      // Starting from real position and real projected yaw
      reference.time.push_back(initTime);

      const vector3_t& currentBasePosition = getBasePosition(initialState, modelInfo_);
      reference.positionInWorld.emplace_back(currentBasePosition.x(), currentBasePosition.y());
      
      reference.baseRelativeHeight.push_back(currentBaseHeight_); // Terrain height will be added later

      reference.yaw.push_back(projectEulerZyxToFrame(
        getBaseOrientationZyx(initialState, modelInfo_), TerrainPlane()).x());

      for(size_t i = 1; i < referenceSize; ++i)
      { 
        reference.time.push_back(reference.time.back() + settings_.deltaTime);

        const vector2_t commandedVelocityInWorld = map2DVelocityCommandToWorld(
          command.baseHeadingVelocity, command.baseLateralVelocity, 
          reference.yaw.back() + 0.5 * command.yawRate * settings_.deltaTime);
        reference.positionInWorld.emplace_back(
          reference.positionInWorld.back().x() + 
            commandedVelocityInWorld.x() * settings_.deltaTime, 
          reference.positionInWorld.back().y() + 
            commandedVelocityInWorld.y() * settings_.deltaTime);

        reference.baseRelativeHeight.push_back(
          std::clamp(reference.baseRelativeHeight.back() + 
            command.baseVerticalVelocity * settings_.deltaTime, settings_.minimumBaseHeight, 
            settings_.maximumBaseHeight));

        reference.yaw.push_back(reference.yaw.back() + command.yawRate * settings_.deltaTime);
      }

      // Update current base height
      currentBaseHeight_ = reference.baseRelativeHeight.back();

      return reference;
    }

    BaseTrajectoryPlanner::BaseReferenceTrajectory BaseTrajectoryPlanner::generateExtrapolatedBaseReference(
      scalar_t initTime, scalar_t finalTime, const state_vector_t& initialState,
      const BaseReferenceCommand& command)
    {
      // Helper to get a projected heading frame derived from the terrain.
      auto projectedHeadingPlane = [&](const vector2_t& baseXYPosition, scalar_t yaw) 
      {
        vector2_t lfOffsetLocal(settings_.nominalBaseWidthHeading / 2.0, 
          settings_.nominalBaseWidtLateral / 2.0);
        vector2_t rfOffsetLocal(settings_.nominalBaseWidthHeading / 2.0, 
          -settings_.nominalBaseWidtLateral / 2.0);
        vector2_t lhOffsetLocal(-settings_.nominalBaseWidthHeading / 2.0, 
          settings_.nominalBaseWidtLateral / 2.0);
        vector2_t rhOffsetLocal(-settings_.nominalBaseWidthHeading / 2.0, 
          -settings_.nominalBaseWidtLateral / 2.0);
        
        // Rotate from heading to world frame
        vector2_t lfOffsetGlobal = rotateVector2D(lfOffsetLocal, yaw);
        vector2_t rfOffsetGlobal = rotateVector2D(rfOffsetLocal, yaw);
        vector2_t lhOffsetGlobal = rotateVector2D(lhOffsetLocal, yaw);
        vector2_t rhOffsetGlobal = rotateVector2D(rhOffsetLocal, yaw);

        // shift by base center
        lfOffsetGlobal += baseXYPosition;
        rfOffsetGlobal += baseXYPosition;
        lhOffsetGlobal += baseXYPosition;
        rhOffsetGlobal += baseXYPosition;

        // Get position with smoothed terrain height
        vector3_t lfVerticalProjection = terrainModel_->getSmoothedPositon(lfOffsetGlobal);
        vector3_t rfVerticalProjection = terrainModel_->getSmoothedPositon(rfOffsetGlobal);
        vector3_t lhVerticalProjection = terrainModel_->getSmoothedPositon(lhOffsetGlobal);
        vector3_t rhVerticalProjection = terrainModel_->getSmoothedPositon(rhOffsetGlobal);

        const TerrainPlane basePlane = computeTerrainPlane({lfVerticalProjection, 
          rfVerticalProjection, lhVerticalProjection, rhVerticalProjection});
        
        const vector3_t eulerAngles(yaw, 0.0, 0.0);
        
        const matrix3_t oldRotation = getRotationMatrixFromZyxEulerAngles(eulerAngles);

        const matrix3_t newRotationMatrix = projectRotationMatrixOnPlane(oldRotation, basePlane);

        return TerrainPlane(basePlane.getPosition(), newRotationMatrix.transpose());
      };

      const auto flatReference = generate2DExtrapolatedBaseReference(initTime, finalTime, 
        initialState, command);

      BaseTrajectoryPlanner::BaseReferenceTrajectory baseReference;
      baseReference.time = std::move(flatReference.time);
      size_t referenceSize = baseReference.time.size();

      baseReference.eulerZyx.reserve(referenceSize);
      baseReference.positionInWorld.reserve(referenceSize);

      // Adapt poses
      for (size_t i = 0; i < referenceSize; ++i) 
      {
        const auto projectedHeadingFrame = projectedHeadingPlane(flatReference.positionInWorld[i], 
          flatReference.yaw[i]);
        
        vector3_t basePosition = projectedHeadingFrame.projectPositionInWorldOntoPlaneAlongGravity(
          flatReference.positionInWorld[i]);
        basePosition.z() += flatReference.baseRelativeHeight[i];

        baseReference.positionInWorld.push_back(std::move(basePosition));

        baseReference.eulerZyx.emplace_back(projectEulerZyxToFrame(
          {flatReference.yaw[i], 0.0, 0.0}, projectedHeadingFrame));
      }

      addVelocitiesFromFiniteDifference(baseReference);
      return baseReference;
    }
	} // namespace locomotion
} // namespace legged_locomotion_mpc

