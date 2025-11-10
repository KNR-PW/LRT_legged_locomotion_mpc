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
      PreComputation(),modelInfo_(std::move(modelInfo)), 
      referenceManager_(referenceManager),forwardKinematics_(forwardKinematics), 
      torqueApproximator_(torqueApproximator) {}

    
} // namespace legged_locomotion_mpc
