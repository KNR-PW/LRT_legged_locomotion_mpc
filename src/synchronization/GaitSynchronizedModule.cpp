
#include <legged_locomotion_mpc/synchronization/GaitSynchronizedModule.hpp>


namespace legged_locomotion_mpc 
{
  namespace synchronization
  {
    
    using namespace ocs2;
    using namespace locomotion;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    GaitSynchronizedModule::GaitSynchronizedModule(Synchronized<GaitPlanner>& gaitPlanner): 
      gaitPlannerPtr_(&gaitPlanner)
    {
      parametersUpdated_ = false;
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitSynchronizedModule::preSolverRun(scalar_t initTime, scalar_t finalTime, 
      const vector_t& initState, 
      const ReferenceManagerInterface& referenceManager)
    {
      if(parametersUpdated_)
      {
        std::lock_guard<std::mutex> lock(receivedDynamicParametersMutex_);
        gaitPlannerPtr_->lock()->updateDynamicParameters(newParametersTime_, 
          newDynamicParameters_);
        parametersUpdated_ = false;
      }
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitSynchronizedModule::postSolverRun(
      const PrimalSolution& primalSolution) {}
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void GaitSynchronizedModule::updateDynamicParameters(scalar_t time, 
      const locomotion::GaitDynamicParameters& dynamicParams)
    {
      if(newDynamicParameters_ != dynamicParams)
      {
        std::lock_guard<std::mutex> lock(receivedDynamicParametersMutex_);
        newDynamicParameters_= dynamicParams;
        newParametersTime_  = time;
        parametersUpdated_ = true;
      }
    }
  } // namespace synchronization
} // namespace legged_locomotion_mpc 