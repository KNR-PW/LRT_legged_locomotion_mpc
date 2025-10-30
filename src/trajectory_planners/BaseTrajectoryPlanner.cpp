
#include <legged_locomotion_mpc/trajectory_planners/BaseTrajectoryPlanner.hpp>

#include <algorithm>

#include <terrain_model/planar_model/PlanarFactoryFunctions.hpp>

#include <legged_locomotion_mpc/common/AccessHelperFunctions.hpp>

#include <ocs2_core/misc/LinearInterpolation.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/QuaterionEulerTransforms.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {

    using namespace ocs2;
    using namespace terrain_model;
    using namespace floating_base_model;
    using namespace quaterion_euler_transforms;

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
        
      vector3_t newEuler = getEulerAnglesFromRotationMatrix(
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

    void BaseTrajectoryPlanner::updateBaseHeight(ocs2::scalar_t baseHeight)
    {
      currentBaseHeight_ = baseHeight;
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
      const size_t referenceSize = (finalTime - initTime) / settings_.deltaTime + 1;

      targetTrajectories.timeTrajectory.reserve(referenceSize);

      targetTrajectories.stateTrajectory.resize(referenceSize, 
        vector_t::Zero(modelInfo_.stateDim));

      targetTrajectories.inputTrajectory.resize(referenceSize, 
        vector_t::Zero(modelInfo_.inputDim));

      const vector3_t linearDelta{
        command.baseHeadingVelocity * settings_.deltaTime, 
        command.baseLateralVelocity * settings_.deltaTime, 
        command.baseVerticalVelocity * settings_.deltaTime};

      const vector3_t angularDelta{0.0, 0.0, command.yawRate * settings_.deltaTime};

      const pinocchio::Motion twistDelta(linearDelta, angularDelta);

      const pinocchio::SE3 SE3Delta = pinocchio::exp6(
        twistDelta);

      // Helper to get a projected heading frame derived from the terrain.
      // TODO Think how to change it for robots with other number of legs
      auto projectedHeadingPlane = [&](vector2_t baseXYPosition, scalar_t yaw) 
      {
        const vector2_t lfOffsetLocal(settings_.nominalBaseWidthHeading / 2.0, 
          settings_.nominalBaseWidtLateral / 2.0);
        const vector2_t rfOffsetLocal(settings_.nominalBaseWidthHeading / 2.0, 
          -settings_.nominalBaseWidtLateral / 2.0);
        const vector2_t lhOffsetLocal(-settings_.nominalBaseWidthHeading / 2.0, 
          settings_.nominalBaseWidtLateral / 2.0);
        const vector2_t rhOffsetLocal(-settings_.nominalBaseWidthHeading / 2.0, 
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
        const vector3_t lfVerticalProjection = terrainModel_->getSmoothedPositon(lfOffsetGlobal);
        const vector3_t rfVerticalProjection = terrainModel_->getSmoothedPositon(rfOffsetGlobal);
        const vector3_t lhVerticalProjection = terrainModel_->getSmoothedPositon(lhOffsetGlobal);
        const vector3_t rhVerticalProjection = terrainModel_->getSmoothedPositon(rhOffsetGlobal);

        const TerrainPlane basePlane = computeTerrainPlane({lfVerticalProjection, 
          rfVerticalProjection, lhVerticalProjection, rhVerticalProjection});
        
        const vector3_t eulerAngles(yaw, 0.0, 0.0);
        
        const matrix3_t oldRotation = getRotationMatrixFromZyxEulerAngles(eulerAngles);

        const matrix3_t newRotationMatrix = projectRotationMatrixOnPlane(oldRotation, basePlane);

        return TerrainPlane(basePlane.getPosition(), newRotationMatrix.transpose());
      };
      
      const vector3_t currentBasePosition = legged_locomotion_mpc::
        access_helper_functions::getBasePosition(initialState, modelInfo_);

      const vector3_t currentBaseOrientationZyx = legged_locomotion_mpc::
        access_helper_functions::getBaseOrientationZyx(initialState, 
        modelInfo_);

      const matrix3_t currentBaseRotation = getRotationMatrixFromZyxEulerAngles(
        currentBaseOrientationZyx);

      const scalar_t invDeltaTime = 1 / settings_.deltaTime;

      pinocchio::SE3 currentTransform(currentBaseRotation, currentBasePosition);


      targetTrajectories.timeTrajectory.push_back(initTime);

      vector_t& initStateVector = targetTrajectories.stateTrajectory[0];

      floating_base_model::access_helper_functions::
        getBasePosition(initStateVector, modelInfo_) = currentBasePosition;

      floating_base_model::access_helper_functions::
        getBaseOrientationZyx(initStateVector, modelInfo_) = currentBaseOrientationZyx;

      for (size_t i = 1; i < referenceSize; ++i) 
      {
        // New base height normal to plane 
        currentBaseHeight_ += command.baseVerticalVelocity * settings_.deltaTime;
        
        // Get new position and orientation with current body velocities
        pinocchio::SE3 newTransform = currentTransform * SE3Delta;

        vector3_t newBasePosition = newTransform.translation();
        matrix3_t newBaseRotation = newTransform.rotation();

        vector3_t newBaseOrientationZyx = getEulerAnglesFromRotationMatrix(
          newBaseRotation);

        // Get new position and orientation on new plane
        const TerrainPlane newBaseTerrain = projectedHeadingPlane({newBasePosition.x(), 
          newBasePosition.y()}, newBaseOrientationZyx.x());

        newBaseRotation = projectRotationMatrixOnPlane(newBaseRotation, 
          newBaseTerrain);

        newBasePosition = newBaseTerrain.getPosition();

        // Add new height to position, taking into accoount fact that base is on terrain plane
        newBasePosition.z() += currentBaseHeight_ / 
          newBaseTerrain.getSurfaceNormalInWorld().z();;
      
        newBaseOrientationZyx = getEulerAnglesFromRotationMatrix(
          newBaseRotation);
        
        // Make new transform from base and rotation on plane
        newTransform = pinocchio::SE3(newBaseRotation, newBasePosition);
        
        // Get twist from current -> new transforms
        const pinocchio::Motion newTwist = pinocchio::log6(
          currentTransform.actInv(newTransform)) * invDeltaTime;

        vector_t& currentStateVector = targetTrajectories.stateTrajectory[i];

        floating_base_model::access_helper_functions::
          getBasePosition(currentStateVector, modelInfo_) = newBasePosition;

        floating_base_model::access_helper_functions::
          getBaseOrientationZyx(currentStateVector, modelInfo_) = 
          newBaseOrientationZyx;

        vector_t& previousStateVector = targetTrajectories.stateTrajectory[i - 1];

        floating_base_model::access_helper_functions::
          getBaseVelocity(previousStateVector, modelInfo_) = newTwist.toVector();

        targetTrajectories.timeTrajectory.push_back(
          targetTrajectories.timeTrajectory.back() + settings_.deltaTime);
        
        currentTransform = newTransform;
      }

      const vector_t& almostLastStateVector = targetTrajectories.stateTrajectory[referenceSize - 2];
      vector_t& lastStateVector = targetTrajectories.stateTrajectory[referenceSize - 1];

      floating_base_model::access_helper_functions::
        getBaseVelocity(lastStateVector, modelInfo_) = 
        floating_base_model::access_helper_functions::
        getBaseVelocity(almostLastStateVector, modelInfo_);
    }
	} // namespace planners
} // namespace legged_locomotion_mpc

