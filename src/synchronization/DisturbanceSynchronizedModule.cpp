
#include <legged_locomotion_mpc/synchronization/DisturbanceSynchronizedModule.hpp>

namespace legged_locomotion_mpc 
{
  namespace synchronization
  {

    using namespace ocs2;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    DisturbanceSynchronizedModule::DisturbanceSynchronizedModule() 
    {
      disturbanceUpdated_ = false;
      activeDisturbance_ = vector6_t::Zero();
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void DisturbanceSynchronizedModule::preSolverRun(
      ocs2::scalar_t initTime, ocs2::scalar_t finalTime, const ocs2::vector_t& initState,
      const ocs2::ReferenceManagerInterface& referenceManager)
    {
      if(disturbanceUpdated_)
      {
        std::lock_guard<std::mutex> lock(receivedDisturbanceMutex_);
        activeDisturbance_= newDisturbance_;
        activeDisturbanceTime_ = newDisturbanceTime_;
        disturbanceUpdated_ = false;
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void DisturbanceSynchronizedModule::postSolverRun(const ocs2::PrimalSolution& primalSolution) {}

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void DisturbanceSynchronizedModule::updateDistrubance(ocs2::scalar_t time, 
      const vector6_t& disturbance)
    {
      if(newDisturbance_ != disturbance)
      {
        std::lock_guard<std::mutex> lock(receivedDisturbanceMutex_);
        newDisturbance_ = disturbance;
        newDisturbanceTime_ = time;
        disturbanceUpdated_ = true;
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    std::pair<ocs2::scalar_t, vector6_t> DisturbanceSynchronizedModule::getCurrentDisturbance() const
    {
      return {activeDisturbanceTime_, activeDisturbance_};
    }

  } // namespace synchronization
} // namespace legged_locomotion_mpc

