#include <legged_locomotion_mpc/cost/QuadraticFinalTrackingCost.hpp>


namespace legged_locomotion_mpc 
{
  namespace cost
  {

    using namespace ocs2;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    QuadraticFinalTrackingCost::QuadraticFinalTrackingCost(vector_t Q,
      floating_base_model::FloatingBaseModelInfo& info,
      const LeggedSynchronizedModule &leggedSynchronizedModule,
      const SwitchedModelReferenceManager& referenceManager)
        : StateCost(), Q_(std::move(Q)), info_(&info),
          leggedSynchronizedModulePtr_(&leggedSynchronizedModule),
          referenceManagerPtr_(&referenceManager)

    QuadraticFinalTrackingCost::QuadraticFinalTrackingCost *clone() const
    {
      return QuadraticFinalTrackingCost(*this);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    scalar_t QuadraticFinalTrackingCost::getValue(scalar_t time, const vector_t& state,
      const TargetTrajectories& targetTrajectories,
      const PreComputation&) const
    {
      vector_t stateDeviation;
      stateDeviation = getStateDeviation(time, state, targetTrajectories);

      vector_t newQ;

      if(leggedSynchronizedModulePtr_->newCostData())
      {
        newQ = leggedSynchronizedModulePtr_->getQuadraticFinalTrackingCoefficients();
        assert(stateDeviation.rows() == newQ.rows());
        return 0.5 * stateDeviation.dot(newQ.asDiagonal() * stateDeviation);
      }
      else
      {
        assert(stateDeviation.rows() == Q_.rows());
        return 0.5 * stateDeviation.dot(Q_.asDiagonal() * stateDeviation);
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    ScalarFunctionQuadraticApproximation QuadraticFinalTrackingCost::getQuadraticApproximation(scalar_t time,
      const vector_t& state, const TargetTrajectories& targetTrajectories,
      const PreComputation&) const
    {
      vector_t stateDeviation;
      stateDeviation = getStateInputDeviation(time, state, targetTrajectories);

      vector_t newQ;
      ScalarFunctionQuadraticApproximation L;

      if(leggedSynchronizedModulePtr_->newCostData())
      {
        newQ = leggedSynchronizedModulePtr_->getQuadraticFinalTrackingCoefficients();
        assert(stateDeviation.rows() == newQ.rows());

        L.dfdxx = Eigen::MatrixXd(newQ.asDiagonal());
        L.dfduu.setZero(input.size(), input.size());
        L.dfdx.noalias() = newQ.asDiagonal() * stateDeviation;
        L.dfdu.noalias().setZero(input.size(), 1);
      }
      else
      {
        assert(inputDeviation.rows() == R_.rows());
        assert(stateDeviation.rows() == Q_.rows());

        L.dfdxx = Eigen::MatrixXd(Q_.asDiagonal());
        L.dfduu.setZero(input.size(), input.size());
        L.dfdx.noalias() = Q_.asDiagonal() * stateDeviation;
        L.dfdu.noalias().setZero(input.size(), 1);
      }

      L.f = 0.5 * stateDeviation.dot(L.dfdx);
      L.dfdux.setZero(input.size(), state.size());

      return L;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    vector_t QuadraticFinalTrackingCost::getStateDeviation(scalar_t time,
      const vector_t &state,
      const TargetTrajectories &targetTrajectories) const
    {
      const vector_t xNominal = targetTrajectories.getDesiredState(time);
      return {state - xNominal};
    }
  
  } // namespace cost
}  // namespace legged_locomotion_mpc