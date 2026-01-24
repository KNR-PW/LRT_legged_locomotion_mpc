#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

#include <terrain_model/core/TerrainPlane.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  using namespace floating_base_model;
  using namespace terrain_model;

  LeggedPrecomputation::LeggedPrecomputation(
    FloatingBaseModelInfo modelInfo,
    const LeggedReferenceManager& referenceManager,
    const PinocchioForwardEndEffectorKinematicsCppAd& forwardKinematics,
    const PinocchioForwardCollisionKinematicsCppAd& collisionKinematics,
    const PinocchioTorqueApproximationCppAd& torqueApproximator):
      PreComputation(), 
      endEffectorNumber_(modelInfo.numThreeDofContacts + modelInfo.numSixDofContacts),
      modelInfo_(std::move(modelInfo)), 
      collisionLinkNumber_(collisionKinematics.getCollisionNumber()),
      referenceManager_(referenceManager),forwardKinematics_(forwardKinematics), 
      collisionKinematics_(collisionKinematics), torqueApproximator_(torqueApproximator) 
  {
    rotationWorldToTerrains_.resize(endEffectorNumber_);
  }

  LeggedPrecomputation* LeggedPrecomputation::clone() const
  { 
    return new LeggedPrecomputation(*this); 
  }

  void LeggedPrecomputation::request(RequestSet request, scalar_t t, 
    const vector_t &x, const vector_t &u)
  {
    assert(x.size() == modelInfo_.stateDim);
    assert(u.size() == modelInfo_.inputDim);
    
    // Constraints || soft constraints || cost
    if(request.containsAny(Request::Constraint + Request::SoftConstraint + Request::Cost))
    {
      updateEndEffectorPositions(t, x);
      updateEndEffectorLinearVelocities(t, x, u);

      if(request.contains(Request::Approximation))
      {
        updateEndEffectorPositionDerviatives(t, x);
        updateEndEffectorLinearVelocityDerviatives(t, x, u);
      }
    }

    // Constraints || soft constraints
    if(request.containsAny(Request::Constraint + Request::SoftConstraint))
    {
      updateReferenceEndEffectorData(t);
      updateEndEffectorOrientations(t, x);

      if(request.contains(Request::Approximation))
      {
        updateEndEffectorOrientationDerviatives(t, x);
      }
    }

    // Constraints
    if(request.contains(Request::Constraint))
    {
      updateEndEffectorAngularVelocities(t, x, u);
      if(request.contains(Request::Approximation))
      {
        updateEndEffectorAngularVelocityDerviatives(t, x, u);
      }
    }

    // Soft constraints
    if(request.contains(Request::SoftConstraint))
    {
      updateCollisionKinematicsData(t, x);
      updateApproximatedTorquesData(t, x, u);
      if(request.contains(Request::Approximation))
      {
        updateCollisionKinematicsDerivatives(t, x);
        updateApproximatedTorquesDerivatives(t, x, u);
      }
    }
  }

  const vector3_t& LeggedPrecomputation::getEndEffectorPosition(size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);

    return endEffectorPositions_[endEffectorIndex];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorPositionDerivatives(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);

    return endEffectorPositionDerivaties_[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getEndEffectorOrientation(size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    assert(endEffectorIndex >= modelInfo_.numThreeDofContacts);

    return endEffectorEulerAngles_[endEffectorIndex - modelInfo_.numThreeDofContacts];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorOrientationDerivatives(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    assert(endEffectorIndex >= modelInfo_.numThreeDofContacts);

    return endEffectorEulerAngleDerivaties_[endEffectorIndex - modelInfo_.numThreeDofContacts];
  }

  const vector3_t& LeggedPrecomputation::getEndEffectorLinearVelocity(size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);

    return endEffectorLinearVelocities_[endEffectorIndex];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorLinearVelocityDerivatives(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    
    return endEffectorLinearVelocityDerivaties_[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getEndEffectorAngularVelocity(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    assert(endEffectorIndex >= modelInfo_.numThreeDofContacts);
    
    return endEffectorAngularVelocities_[endEffectorIndex - modelInfo_.numThreeDofContacts];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorAngularVelocityDerivatives(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    assert(endEffectorIndex >= modelInfo_.numThreeDofContacts);

    return endEffectorAngularVelocityDerivaties_[endEffectorIndex  - modelInfo_.numThreeDofContacts];
  }

  const vector3_t& LeggedPrecomputation::getCollisionLinkPosition(
    size_t collisionLinkIndex) const
  {
    assert(collisionLinkIndex < collisionLinkNumber_ + endEffectorNumber_);
    assert(collisionLinkIndex >= endEffectorNumber_);

    return collisionLinkPositions_[collisionLinkIndex - endEffectorNumber_];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getCollisionLinkPositionDerivatives(
    size_t collisionLinkIndex) const
  {
    assert(collisionLinkIndex < collisionLinkNumber_ + endEffectorNumber_);
    assert(collisionLinkIndex >= endEffectorNumber_);

    return collisionLinkPositionDerivaties_[collisionLinkIndex - endEffectorNumber_];
  }

  const vector3_t& LeggedPrecomputation::getCollisionLinkOrientation(
    size_t collisionLinkIndex) const
  {
    assert(collisionLinkIndex < collisionLinkNumber_ + endEffectorNumber_);
    assert(collisionLinkIndex >= endEffectorNumber_);

    return collisionLinkEulerAngles_[collisionLinkIndex - endEffectorNumber_];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getCollisionLinkOrientationDerivatives(
    size_t collisionLinkIndex) const
  {
    assert(collisionLinkIndex < collisionLinkNumber_ + endEffectorNumber_);
    assert(collisionLinkIndex >= endEffectorNumber_);

    return collisionLinkEulerAngleDerivaties_[collisionLinkIndex - endEffectorNumber_];
  }

  const vector_t& LeggedPrecomputation::getApproximatedJointTorques() const
  {
    return torqueApproximation_;
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getApproximatedJointTorquesDerivatives() const
  {
    return torqueApproximationDerivatives_;
  }

  const matrix3_t& LeggedPrecomputation::getRotationWorldToTerrain(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    return rotationWorldToTerrains_[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getSurfaceNormal(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    return referenceTrajectoryPoint_.surfaceNormals[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getReferenceEndEffectorPosition(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    return referenceTrajectoryPoint_.positions[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getReferenceEndEffectorLinearVelocity(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    return referenceTrajectoryPoint_.velocities[endEffectorIndex];
  }

  scalar_t LeggedPrecomputation::getReferenceEndEffectorTerrainClearance(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    return referenceTrajectoryPoint_.clearances[endEffectorIndex];
  }

  void LeggedPrecomputation::updateEndEffectorPositions(scalar_t time, 
    const vector_t& state)
  {
    endEffectorPositions_ = forwardKinematics_.getPosition(state);
  }
      
  void LeggedPrecomputation::updateEndEffectorOrientations(scalar_t time, 
    const vector_t& state)
  {
    endEffectorEulerAngles_ = forwardKinematics_.getOrientation(state);
  }
    
  void LeggedPrecomputation::updateEndEffectorLinearVelocities(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    endEffectorLinearVelocities_ = forwardKinematics_.getLinearVelocity(state, input);
  }
  
  void LeggedPrecomputation::updateEndEffectorAngularVelocities(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    endEffectorAngularVelocities_ = forwardKinematics_.getAngularVelocity(state, input);
  }

  void LeggedPrecomputation::updateEndEffectorPositionDerviatives(scalar_t time, 
    const vector_t& state)
  {
    endEffectorPositionDerivaties_ = forwardKinematics_.getPositionLinearApproximation(
      state);
  }
  
  void LeggedPrecomputation::updateEndEffectorOrientationDerviatives(scalar_t time, 
    const vector_t& state)
  {
    endEffectorEulerAngleDerivaties_ = forwardKinematics_.getOrientationLinearApproximation(
      state);
  }
    
  void LeggedPrecomputation::updateEndEffectorLinearVelocityDerviatives(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    endEffectorLinearVelocityDerivaties_ = forwardKinematics_.getLinearVelocityLinearApproximation(
      state, input);
  }
  
  void LeggedPrecomputation::updateEndEffectorAngularVelocityDerviatives(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    endEffectorAngularVelocityDerivaties_ = forwardKinematics_.getAngularVelocityLinearApproximation(
      state, input);
  }

  void LeggedPrecomputation::updateApproximatedTorquesData(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    torqueApproximation_ = torqueApproximator_.getValue(state, input);
  }

  void LeggedPrecomputation::updateApproximatedTorquesDerivatives(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    torqueApproximationDerivatives_ = torqueApproximator_.getLinearApproximation(state, 
      input);
  }

  void LeggedPrecomputation::updateReferenceEndEffectorData(scalar_t time)
  {
    referenceTrajectoryPoint_ = referenceManager_.getEndEffectorTrajectoryPoint(time);
    
    for(size_t i = 0; i < endEffectorNumber_; ++i)
    {
      const auto& normal = referenceTrajectoryPoint_.surfaceNormals[i];
      rotationWorldToTerrains_[i] = TerrainPlane::getOrientationWorldToTerrainFromSurfaceNormalInWorld(normal);
    }
  }

  void LeggedPrecomputation::updateCollisionKinematicsData(scalar_t time, 
    const vector_t& state)
  {
    collisionLinkPositions_ = collisionKinematics_.getPosition(state);
    collisionLinkEulerAngles_ = collisionKinematics_.getOrientation(state);
  }

  void LeggedPrecomputation::updateCollisionKinematicsDerivatives(scalar_t time, 
    const vector_t& state)
  {
    collisionLinkPositionDerivaties_ = collisionKinematics_.getPositionLinearApproximation(
      state);
    collisionLinkEulerAngleDerivaties_ = collisionKinematics_.getOrientationLinearApproximation(
      state);
  }
} // namespace legged_locomotion_mpc
