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
      torqueApproximator_(torqueApproximator)
  {
    endEffectorPositions_.resize(endEffectorNumber_);
    endEffectorPositionDerivaties_.resize(endEffectorNumber_);

    endEffectorVelocities_.resize(endEffectorNumber_);
    endEffectorVelocityDerivaties_.resize(endEffectorNumber_);

    rotationWorldToTerrains_.resize(endEffectorNumber_);
    surfaceNormals_.resize(endEffectorNumber_);

    torqueApproximation_ = vector_t::Zero(modelInfo_.actuatedDofNum);
    torqueApproximationDerivatives_.f = vector_t::Zero(modelInfo_.actuatedDofNum);
    torqueApproximationDerivatives_.dfdx = matrix_t::Zero(modelInfo_.actuatedDofNum, 
      modelInfo_.stateDim);
    torqueApproximationDerivatives_.dfdu = matrix_t::Zero(modelInfo_.actuatedDofNum, 
      modelInfo_.inputDim);
  }

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

  const vector3_t& LeggedPrecomputation::getEndEffectorPosition(size_t endEffectorIndex)
  {
    return endEffectorPositions_[endEffectorIndex];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorPositionDerivatives(
    size_t endEffectorIndex)
  {
    return endEffectorPositionDerivaties_[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getEndEffectorVelocity(size_t endEffectorIndex)
  {
    return endEffectorVelocities_[endEffectorIndex];
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getEndEffectorVelocityDerivatives(
    size_t endEffectorIndex)
  {
    return endEffectorVelocityDerivaties_[endEffectorIndex];
  }

  const vector_t& LeggedPrecomputation::getApproximatedJointTorques()
  {
    return torqueApproximation_;
  }

  const VectorFunctionLinearApproximation& LeggedPrecomputation::getApproximatedJointTorquesDerivatives()
  {
    return torqueApproximationDerivatives_;
  }

  const matrix3_t& LeggedPrecomputation::getRotationWorldToTerrain(
    size_t endEffectorIndex)
  {
    return rotationWorldToTerrains_[endEffectorIndex];
  }

  const vector3_t& LeggedPrecomputation::getSurfaceNormal(
    size_t endEffectorIndex)
  {
    return surfaceNormals_[endEffectorIndex];
  }

  LeggedPrecomputation::LeggedPrecomputation(const LeggedPrecomputation& other):
    PreComputation(other), 
    endEffectorNumber_(other.endEffectorNumber_),
    modelInfo_(other.modelInfo_), 
    referenceManager_(other.referenceManager_), 
    forwardKinematics_(other.forwardKinematics_),
    torqueApproximator_(other.torqueApproximator_),
    endEffectorPositions_(other.endEffectorPositions_),
    endEffectorPositionDerivaties_(other.endEffectorPositionDerivaties_),
    endEffectorVelocities_(other.endEffectorVelocities_),
    endEffectorVelocityDerivaties_(other.endEffectorVelocityDerivaties_),
    rotationWorldToTerrains_(other.rotationWorldToTerrains_),
    surfaceNormals_(other.surfaceNormals_),
    torqueApproximation_(other.torqueApproximation_),
    torqueApproximationDerivatives_(other.torqueApproximationDerivatives_) {}

  void LeggedPrecomputation::updateContactData(scalar_t time,
    const vector_t& state, const vector_t& input)
  {
    const auto terrainModel = referenceManager_.getTerrainModel();

    for(size_t i = 0; i < endEffectorNumber_; ++i)
    {
      const TerrainPlane terrainPlane = terrainModel->getLocalTerrainAtPositionInWorldAlongGravity(
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
