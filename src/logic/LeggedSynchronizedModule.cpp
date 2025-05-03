#include <legged_locomotion_mpc/logic/LeggedSynchronizedModule.hpp>

namespace legged_locomotion_mpc 
{
  using namespace ocs2;

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  LeggedSynchronizedModule::LeggedSynchronizedModule(): SolverSynchronizedModule(),
    newFloatingBaseDisturbance_(std::unique_ptr<Eigen::Matrix<ocs2::scalar_t, 6, 1>>(new Eigen::Matrix<ocs2::scalar_t, 6, 1>())) 
  {

  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  void LeggedSynchronizedModule::preSolverRun(scalar_t initTime,
    scalar_t finalTime, const vector_t& initState,
    const ocs2::ReferenceManagerInterface& referenceManager)
  {
    auto lockedFloatingBaseDisturbancePtr = newFloatingBaseDisturbance_.lock();
    activeFloatingBaseDisturbance_= *lockedFloatingBaseDisturbancePtr;
  }

} // namespace legged_locomotion_mpc 

