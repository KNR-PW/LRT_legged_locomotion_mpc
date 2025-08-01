#include <legged_locomotion_mpc/cost/QuadraticTrackingCost.hpp>


namespace legged_locomotion_mpc 
{
  namespace cost
  {

    using namespace ocs2;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    QuadraticTrackingCost::QuadraticTrackingCost(vector_t Q, vector_t R,
      floating_base_model::FloatingBaseModelInfo& info,
      const LeggedSynchronizedModule &leggedSynchronizedModule,
      const SwitchedModelReferenceManager& referenceManager)
        : StateInputCost(), Q_(std::move(Q)), R_(std::move(R)),
          info_(&info), leggedSynchronizedModulePtr_(&leggedSynchronizedModule),
          referenceManagerPtr_(&referenceManager)
    {
      quadraticApprox_.dfdux.setZero(input.size(), state.size());
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    QuadraticTrackingCost::QuadraticTrackingCost *clone() const
    {
      return new QuadraticTrackingCost(*this)
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    scalar_t QuadraticTrackingCost::getValue(scalar_t time, const vector_t& state,
      const vector_t& input, const TargetTrajectories& targetTrajectories,
      const PreComputation&) const
    {
      vector_t stateDeviation, inputDeviation;
      std::tie(stateDeviation, inputDeviation) = getStateInputDeviation(time, state, input, targetTrajectories);

      vector_t newQ, newR;

      if(leggedSynchronizedModulePtr_->newCostData())
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

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    ScalarFunctionQuadraticApproximation QuadraticTrackingCost::getQuadraticApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const TargetTrajectories& targetTrajectories, const PreComputation&) const
    {
      vector_t stateDeviation, inputDeviation;
      std::tie(stateDeviation, inputDeviation) = getStateInputDeviation(time, state, input, targetTrajectories);

      vector_t newQ, newR;

      if(leggedSynchronizedModulePtr_->newCostData())
      {
        std::tie(newQ, newR) = leggedSynchronizedModulePtr_->getQuadraticTrackingCoefficients();
        assert(inputDeviation.rows() == newR.rows());
        assert(stateDeviation.rows() == newQ.rows());

        quadraticApprox_.dfdxx.noalias() = Eigen::MatrixXd(newQ.asDiagonal());
        quadraticApprox_.dfduu.noalias() = Eigen::MatrixXd(newR.asDiagonal());
        quadraticApprox_.dfdx.noalias() = newQ.asDiagonal() * stateDeviation;
        quadraticApprox_.dfdu.noalias() = newR.asDiagonal() * inputDeviation;
      }
      else
      {
        assert(inputDeviation.rows() == R_.rows());
        assert(stateDeviation.rows() == Q_.rows());

        quadraticApprox_.dfdxx.noalias() = Eigen::MatrixXd(Q_.asDiagonal());
        quadraticApprox_.dfduu.noalias() = Eigen::MatrixXd(R_.asDiagonal());
        quadraticApprox_.dfdx.noalias() = Q_.asDiagonal() * stateDeviation;
        quadraticApprox_.dfdu.noalias() = R_.asDiagonal() * inputDeviation;
      }

      quadraticApprox_.f = 0.5 * stateDeviation.dot(quadraticApprox_.dfdx) + 0.5 * inputDeviation.dot(quadraticApprox_.dfdu);

      return L;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::pair<vector_t, vector_t> QuadraticTrackingCost::getStateInputDeviation(scalar_t time,
      const vector_t &state, const vector_t &input, const TargetTrajectories &targetTrajectories) const
    {
      const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
      const vector_t xNominal = targetTrajectories.getDesiredState(time);
      vector_t uNominal = targetTrajectories.getDesiredInput(time); // TODO: ONLY VELOCITIES!
      uNominal += weightCompensatingInput(info_, contactFlags);
      return {state - xNominal, input - uNominal};
    }
  } // namespace cost
}  // namespace legged_locomotion_mpc