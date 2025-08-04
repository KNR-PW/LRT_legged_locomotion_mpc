#include <legged_locomotion_mpc/penalties/TorqueLimitsPenalty.hpp>

namespace legged_locomotion_mpc
{
  namespace penalty
  {

    using namespace ocs2;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    TorqueLimitsPenalty::TorqueLimitsPenalty(
      const floating_base_model::FloatingBaseModelInfo& info,
      const vector_t& torqueLimits,
      RelaxedBarrierPenalty::Config settings,
      legged_locomotion_mpc::PinocchioTorqueApproximationCppAd& torqueApproximator):
        info_(info),
        torqueApproximatorPtr_(torqueApproximator.clone()),
        torqueRelaxedBarrierPenaltyPtr_(new RelaxedBarrierPenalty(settings)),
        torqueLimits_(torqueLimits)
      {
        // checking size of torqueLimits vector (complicated way :/)

        const vector_t sampleState = vector_t::Zero(info.stateDim);
        const vector_t sampleInput = vector_t::Zero(info.inputDim);
        const vector_t sampleTorques = torqueApproximatorPtr_->getValue(sampleState, sampleInput);
        assert(torqueLimits_.rows() == sampleTorques.rows());

      }
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    scalar_t TorqueLimitsPenalty::getValue(scalar_t time, const vector_t& state,
      const vector_t& input, const TargetTrajectories& targetTrajectories,
      const PreComputation& preComp) const
    {
      const vector_t torqueApprox = torqueApproximatorPtr_->getValue(state, input);
      const vector_t upperBoundTorqueOffset = torqueLimits_ - torqueApprox;
      const vector_t lowerBoundTorqueOffset = torqueLimits_ + torqueApprox;

      return upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointTorquePenalty_->getValue(0.0, hi);
      }).sum() + lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointTorquePenalty_->getValue(0.0, hi);
      }).sum();
    }
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    ScalarFunctionQuadraticApproximation TorqueLimitsPenalty::getQuadraticApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const TargetTrajectories& targetTrajectories,
      const PreComputation& preComp) const
    {
      const size_t forceSize = 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;
      const auto torqueApproxDerivative = torqueApproximatorPtr_->getLinearApproximation(state, input);
      const matrix_t dTorquedQ = torqueApproxDerivative.dfdx.block(0, 12, info_.actuatedDofNum, info_.generalizedCoordinatesNum);
      const matrix_t dTorquedF = torqueApproxDerivative.dfdu.block(0, 0, info_.actuatedDofNum, forceSize);
      const vector_t upperBoundTorqueOffset = torqueLimits_ - torqueApproxDerivative.f;
      const vector_t lowerBoundTorqueOffset = torqueLimits_ + torqueApproxDerivative.f;

      ScalarFunctionQuadraticApproximation cost;
      cost.dfdx = vector_t::Zero(info.stateDim);
      cost.dfdu = vector_t::Zero(info.inputDim);
      cost.f = upperBoundTorqueOffset.unaryExpr([&](scalar_t hi) 
      {
        return jointTorquePenalty_->getValue(0.0, hi);
      }).sum() + lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      {
        return jointTorquePenalty_->getValue(0.0, hi);
      }).sum();

      // Penalty derivatives w.r.t. the constraint
      const vector_t penaltyDerivatives = lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      {
        return jointTorquePenalty_->getDerivative(0.0, hi);
      }) - upperBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      { 
        return jointTorquePenalty_->getDerivative(0.0, hi); 
      });

      const vector_t penaltySecondDerivatives = lowerBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      {
          return jointTorquePenalty_->getSecondDerivative(0.0, hi);
      }) + upperBoundTorqueOffset.unaryExpr([&](scalar_t hi)
      {
          return jointTorquePenalty_->getSecondDerivative(0.0, hi);
      });

      // matrix_t hessianStateState, hessianInputInput, hessianInputState;
      // std::tie(hessianStateState, hessianInputInput, hessianInputState) = torqueApproximatorPtr_->getHessians(penaltyDerivatives, state, input);
      matrix_t hessianStateState, hessianInputState;
      std::tie(hessianStateState, hessianInputState) = torqueApproximatorPtr_->getHessians(penaltyDerivatives, state, input);

      cost.dfdx.block(12, 0, info_.actuatedDofNum, 1).noalias() =  dTorquedQ.transpose() * penaltyDerivatives;
      cost.dfdu.block(0, 0, forceSize, 1).noalias() =  dTorquedF.transpose() * penaltyDerivatives;
      cost.dfdxx.noalias() = hessianStateState + dTorquedQ.transpose() * penaltySecondDerivatives.asDiagonal() * dTorquedQ;
      cost.dfduu.noalias() = dTorquedF.transpose() * penaltySecondDerivatives.asDiagonal() * dTorquedF; // torque is linearly dependent on forces, so its hessian is zero
      cost.dfdux.noalias() = hessianInputState + dTorquedF.transpose() * penaltySecondDerivatives.asDiagonal() * dTorquedQ;

      return cost;
    }

  } // namespace penalty
} // namespace legged_locomotion_mpc