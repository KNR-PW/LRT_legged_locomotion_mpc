#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>


namespace legged_locomotion_mpc
{
  using namespace ocs2;
  using namespace floating_base_model;
  using namespace terrain_model;

  LeggedPrecomputation::LeggedPrecomputation(
    FloatingBaseModelInfo modelInfo,
    const LeggedReferenceManager& referenceManager,
    const PinocchioForwardEndEffectorKinematicsCppAd& forwardKinematics,
    const PinocchioTorqueApproximationCppAd& torqueApproximator):
      PreComputation(), 
      endEffectorNumber_(modelInfo.numThreeDofContacts + modelInfo.numSixDofContacts),
      modelInfo_(std::move(modelInfo)), 
      referenceManager_(referenceManager),forwardKinematics_(forwardKinematics), 
      torqueApproximator_(torqueApproximator) { }

  LeggedPrecomputation* LeggedPrecomputation::clone() const
  { 
    return new LeggedPrecomputation(*this); 
  }

  void LeggedPrecomputation::request(RequestSet request, scalar_t t, 
    const vector_t &x, const vector_t &u)
  {
    updateEndEffectorKinematicsData(t, x, u);

    if(request.contains(Request::Constraint))
    {
      updateContactData(t, x, u);
    }

    if(request.contains(Request::Cost))
    {
      updateApproximatedTorquesData(t, x, u);
    }

    if(request.contains(Request::Approximation))
    {
      updateEndEffectorKinematicsDerivatives(t, x, u);
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

  const ocs2::VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorOrientationDerivatives(
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

  const ocs2::VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorAngularVelocityDerivatives(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    assert(endEffectorIndex >= modelInfo_.numThreeDofContacts);

    return endEffectorAngularVelocityDerivaties_[endEffectorIndex  - modelInfo_.numThreeDofContacts];
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

    return surfaceNormals_[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getReferenceEndEffectorLinearVelocity(
    size_t endEffectorIndex) const
  {
    assert(endEffectorIndex < endEffectorNumber_);
    return referenceEndEffectorLinearVelocities_[endEffectorIndex];
  }

  LeggedPrecomputation::LeggedPrecomputation(const LeggedPrecomputation& other):
    PreComputation(other), 
    endEffectorNumber_(other.endEffectorNumber_),
    modelInfo_(other.modelInfo_), 
    referenceManager_(other.referenceManager_), 
    forwardKinematics_(other.forwardKinematics_),
    torqueApproximator_(other.torqueApproximator_) {}

  void LeggedPrecomputation::updateContactData(scalar_t time,
    const vector_t& state, const vector_t& input)
  {
    const TerrainModel& terrainModel = referenceManager_.getTerrainModel();

    for(size_t i = 0; i < endEffectorNumber_; ++i)
    {
      const TerrainPlane terrainPlane = terrainModel.getLocalTerrainAtPositionInWorldAlongGravity(
        endEffectorPositions_[i]);

      rotationWorldToTerrains_[i] = terrainPlane.getOrientationToTerrain();
      surfaceNormals_[i] = terrainPlane.getSurfaceNormalInWorld();
    }
  }

  void LeggedPrecomputation::updateEndEffectorKinematicsData(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    endEffectorPositions_ = forwardKinematics_.getPosition(state);
    endEffectorLinearVelocities_ = forwardKinematics_.getLinearVelocity(state, input);

    endEffectorEulerAngles_ = forwardKinematics_.getOrientation(state);
    endEffectorAngularVelocities_ = forwardKinematics_.getAngularVelocity(state, input);
  }

  void LeggedPrecomputation::updateEndEffectorKinematicsDerivatives(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    endEffectorPositionDerivaties_ = forwardKinematics_.getPositionLinearApproximation(
      state);
    endEffectorLinearVelocityDerivaties_ = forwardKinematics_.getLinearVelocityLinearApproximation(
      state, input);
      
    endEffectorEulerAngleDerivaties_ = forwardKinematics_.getOrientationLinearApproximation(
      state);
    endEffectorAngularVelocityDerivaties_ = forwardKinematics_.getAngularVelocityLinearApproximation(
      state, input);
  }

  void LeggedPrecomputation::updateApproximatedTorquesData(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    torqueApproximation_ = torqueApproximator_.getValue(state, input);
    torqueApproximationDerivatives_ = torqueApproximator_.getLinearApproximation(state, input);
  }

  void LeggedPrecomputation::updateReferenceEndEffectorVelocities(scalar_t time)
  {
    const vector_t referenceState = referenceManager_.getTargetTrajectories().getDesiredState(time);
    const vector_t referenceInput = referenceManager_.getTargetTrajectories().getDesiredInput(time);
    referenceEndEffectorLinearVelocities_ = forwardKinematics_.getLinearVelocity(
      referenceState, referenceInput);
  }
} // namespace legged_locomotion_mpc
