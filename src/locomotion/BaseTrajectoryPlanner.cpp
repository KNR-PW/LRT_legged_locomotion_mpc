
#include <legged_locomotion_mpc/locomotion/BaseTrajectoryPlanner.hpp>

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

      scalar_t getYawFromCurrentOrientation(const vector3_t& eulerZyx, const TerrainPlane& plane)
      {
        vector3_t headingVector = getRotationMatrixFromZyxEulerAngles(eulerZyx).col(0);
        vector3_t xAxis = plane.projectVectorInWorldOntoPlaneAlongGravity(headingVector).normalized();
        vector3_t zAxis = plane.getSurfaceNormalInWorld();
        vector3_t yAxis = zVector.cross(xAxis);

        matrix3_t projectedRotation;
        projectedRotation.col(0) = xAxis;
        projectedRotation.col(1) = yAxis;
        projectedRotation.col(2) = zAxis;
        vector3_t newEuler = getEulerAnglesFromRotationMatrix(projectedRotation);
        makeEulerAnglesUnique(newEuler);

        return newEuler.x();
      }

      scalar_t inv_dt = 1 / (baseReference.time[referenceSize - 1] - baseReference.time[referenceSize - 2]);
      const matrix3_t newRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        baseReference.eulerZyx[referenceSize - 1]);
      const matrix3_t oldRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        baseReference.eulerZyx[referenceSize - 2]);
      baseReference.angularVelocityInWorld.push_back(
        rotationErrorInWorld(newRotationMatrix, oldRotationMatrix) * inv_dt);
      baseReference.linearVelocityInWorld.push_back(
        (baseReference.positionInWorld[referenceSize - 1] - 
          baseReference.positionInWorld[referenceSize - 2]) * inv_dt);
    }

    BaseTrajectoryPlanner::BaseTrajectoryPlanner(FloatingBaseModelInfo modelInfo,
      BaseTrajectoryPlanner::StaticSettings settings): 
        modelInfo_(std::move(modelInfo)), settings_(std::move(settings)) {}
      
    void BaseTrajectoryPlanner::updateTerrain(const terrain_model::TerrainModel& terrainModel)
    {
      terrainModel_ = &terrainModel;
    }

    void BaseTrajectoryPlanner::updateTargetTrajectory(scalar_t initTime,
      scalar_t finalTime, const BaseTrajectoryPlanner:BaseReferenceCommand& command,
      const state_vector_t& initialState,
      TargetTrajectories& targetTrajectories)
    {
      BaseTrajectoryPlanner::BaseReferenceTrajectory baseReference = 
        generateExtrapolatedBaseReference(initTime, finalTime, initialState, command);
      
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
        getBaseLinearVelocity(currentState, modelInfo_) = 
          baseReference.linearVelocityInWorld[i];
        getBaseAngularVelocity(currentState, modelInfo_) = 
          baseReference.angularVelocityInWorld[i];
      }
    }

    Eigen::Vector2d BaseTrajectoryPlanner::map2DVelocityCommandToWorld(
      scalar_t headingVelocity, scalar_t lateralVelocity, scalar_t yaw)
    {
      Eigen::Vector2d velocityInWorld;
      const scalar_t sinYaw = sin(yaw);
      const scalar_t cosYaw = cos(yaw);
      velocityInWorld.x() = headingVelocity * cosYaw - lateralVelocity * sinYaw;
      velocityInWorld.y() = headingVelocity * sinYaw + lateralVelocity * cosYaw;

      return velocityInWorld;
    }

    BaseFlatReferenceTrajectory BaseTrajectoryPlanner::generate2DExtrapolatedBaseReference(
      scalar_t initTime, scalar_t finalTime, const state_vector_t& initialState,
      const BaseReferenceCommand& command)
    {
      const size_t referenceSize = (finalTime - initTime) / settings_.deltaTime + 1;

      BaseFlatReferenceTrajectory reference;
      reference.time.reserve(referenceSize);
      reference.positionInWorld.reserve(referenceSize);
      reference.yaw.reserve(referenceSize);
      
      using namespace legged_locomotion_mpc::access_helper_functions;

      // Starting from real position and real projected yaw
      reference.time.push_back(initTime);
      reference.positionInWorld.push_back(getBasePosition(initialState, modelInfo_));
      reference.yaw.push_back(getYawFromCurrentOrientation(getBasePosition(initialState, 
        modelInfo_), TerrainPlane()));

      for(size_t i = 1; i < referenceSize; ++i)
      { 
        reference.time.push_back(reference.time.back() + settings_.deltaTime);
        const vector2_t commandedVelocityInWorld = map2DVelocityCommandToWorld(
          command.baseHeadingVelocity, command.baseLateralVelocity, 
          reference.yaw.back() + 0.5 * command.yawRate * settings_.deltaTime);
        reference.positionInWorld.emplace_back(
          reference.positionInWorld[i - 1].x() + 
          commandedVelocityInWorld.x() * settings_.deltaTime, 
          reference.positionInWorld[i - 1].y() + 
          commandedVelocityInWorld.y() * settings_.deltaTime, 
          reference.positionInWorld[i - 1].z() + 
          command.baseVerticalVelocity * settings_.deltaTime);

        reference.yaw.push_back(reference.yaw.back() + command.yawRate * settings_.deltaTime);
      }
      return reference;
    }

	} // namespace locomotion
} // namespace legged_locomotion_mpc

