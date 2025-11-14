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
    return endEffectorPositions_[endEffectorIndex];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorPositionDerivatives(
    size_t endEffectorIndex) const
  {
    return endEffectorPositionDerivaties_[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getEndEffectorVelocity(size_t endEffectorIndex) const
  {
    return endEffectorVelocities_[endEffectorIndex];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorVelocityDerivatives(
    size_t endEffectorIndex) const
  {
    return endEffectorVelocityDerivaties_[endEffectorIndex];
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
    return rotationWorldToTerrains_[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getSurfaceNormal(
    size_t endEffectorIndex) const
  {
    return surfaceNormals_[endEffectorIndex];
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
    for(size_t i = 0; i < endEffectorNumber_; ++i)
    {
      endEffectorPositions_ = forwardKinematics_.getPosition(state);
      endEffectorVelocities_ = forwardKinematics_.getLinearVelocity(state, input);
    }
  }

  void LeggedPrecomputation::updateEndEffectorKinematicsDerivatives(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    for(size_t i = 0; i < endEffectorNumber_; ++i)
    {
      endEffectorPositionDerivaties_ = forwardKinematics_.getPositionLinearApproximation(
        state);
      endEffectorVelocityDerivaties_ = forwardKinematics_.getLinearVelocityLinearApproximation(
        state, input);
    }
  }

  void LeggedPrecomputation::updateApproximatedTorquesData(scalar_t time, 
    const vector_t& state, const vector_t& input)
  {
    torqueApproximation_ = torqueApproximator_.getValue(state, input);
    torqueApproximationDerivatives_ = torqueApproximator_.getLinearApproximation(state, input);
  }
    
} // namespace legged_locomotion_mpc
