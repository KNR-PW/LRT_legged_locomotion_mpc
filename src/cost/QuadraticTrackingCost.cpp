#include <legged_locomotion_mpc/cost/QuadraticTrackingCost.hpp>


namespace legged_locomotion_mpc 
{
  QuadraticTrackingCost::QuadraticTrackingCost(ocs2::vector_t Q, ocs2::vector_t R,
    floating_base_model::FloatingBaseModelInfo& info,
    const LeggedSynchronizedModule &leggedSynchronizedModule,
    const SwitchedModelReferenceManager& referenceManager)
      : StateInputCost(), Q_(std::move(Q)), R_(std::move(R)),
        info_(&info), leggedSynchronizedModulePtr_(&leggedSynchronizedModule),
        referenceManagerPtr_(&referenceManager)
  {
  
  }

  QuadraticTrackingCost::QuadraticTrackingCost *clone() const
  {
    return new QuadraticTrackingCost(*this)
  }

  ocs2::scalar_t QuadraticTrackingCost::getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
    const ocs2::vector_t& input, const ocs2::TargetTrajectories& targetTrajectories,
    const ocs2::PreComputation&) const
  {
    ocs2::vector_t stateDeviation, inputDeviation;
    std::tie(stateDeviation, inputDeviation) = getStateInputDeviation(time, state, input, targetTrajectories);

    ocs2::vector_t newQ, newR;

    if(leggedSynchronizedModulePtr_->newData())
    {
      std::tie(newQ, newR) = leggedSynchronizedModulePtr_->getQuadraticTrackingCoefficients();
      assert(inputDeviation.rows() == newR.rows());
      assert(stateDeviation.rows() == newQ.rows());
      return 0.5 * inputDeviation.dot(newR.asDiagonal() * inputDeviation) + 0.5 * stateDeviation.dot(newQ.asDiagonal() * stateDeviation);
    }
    else
    {
      assert(inputDeviation.rows() == R_.rows());
      assert(stateDeviation.rows() == Q_.rows());
      return 0.5 * inputDeviation.dot(R_.asDiagonal() * inputDeviation) + 0.5 * stateDeviation.dot(Q_.asDiagonal() * stateDeviation);
    }

  }

  ocs2::ScalarFunctionQuadraticApproximation QuadraticTrackingCost::getQuadraticApproximation(
    ocs2::scalar_t time, const ocs2::vector_t& state, const vector_t& input,
    const ocs2::TargetTrajectories& targetTrajectories, const ocs2::PreComputation&) const
  {
    ocs2::vector_t stateDeviation, inputDeviation;
    std::tie(stateDeviation, inputDeviation) = getStateInputDeviation(time, state, input, targetTrajectories);

    ocs2::vector_t newQ, newR;
    ocs2::ScalarFunctionQuadraticApproximation L;

    if(leggedSynchronizedModulePtr_->newData())
    {
      std::tie(newQ, newR) = leggedSynchronizedModulePtr_->getQuadraticTrackingCoefficients();
      assert(inputDeviation.rows() == newR.rows());
      assert(stateDeviation.rows() == newQ.rows());

      L.dfdxx = newQ;
      L.dfduu = newR;
      L.dfdx.noalias() = newQ * stateDeviation;
      L.dfdu.noalias() = newR * inputDeviation;
    }
    else
    {
      assert(inputDeviation.rows() == R_.rows());
      assert(stateDeviation.rows() == Q_.rows());

      L.dfdxx = Q_;
      L.dfduu = R_;
      L.dfdx.noalias() = Q_ * stateDeviation;
      L.dfdu.noalias() = R_ * inputDeviation;
    }

    L.f = 0.5 * stateDeviation.dot(L.dfdx) + 0.5 * inputDeviation.dot(L.dfdu);
    L.dfdux.setZero(input.size(), state.size());
    
    return L;
  }

  std::pair<ocs2::vector_t, ocs2::vector_t> QuadraticTrackingCost::getStateInputDeviation(scalar_t time,
    const ocs2::vector_t &state, const ocs2::vector_t &input, const TargetTrajectories &targetTrajectories) const
  {
    const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
    const vector_t xNominal = targetTrajectories.getDesiredState(time);
    ocs2::vector_t uNominal = targetTrajectories.getDesiredInput(time); // TODO: ONLY VELOCITIES!
    uNominal += weightCompensatingInput(info_, contactFlags);
    return {state - xNominal, input - uNominal};
  }
}