#ifndef __SYNCHRONIZED_MODULE_LEGGED_LOCOMOTION_MPC__
#define __SYNCHRONIZED_MODULE_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "ocs2_switched_model_interface/dynamics/ComKinoDynamicsParameters.h"

namespace legged_locomotion_mpc 
{
  class LeggedSynchronizedModule: public ocs2::SolverSynchronizedModule 
  {

    public:

      LeggedSynchronizedModule();

      void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                      const ocs2::ReferenceManagerInterface& referenceManager) override;

      void postSolverRun(const ocs2::PrimalSolution& primalSolution) override {};

      // Write-able access to floating base disturbance
      ocs2::Synchronized<Eigen::Matrix<ocs2::scalar_t, 6, 1>>& getFloatingBaseDisturbance() 
      { 
        return newFloatingBaseDisturbance_; 
      }
      const ocs2::Synchronized<Eigen::Matrix<ocs2::scalar_t, 6, 1>>& getFloatingBaseDisturbance() const 
      { 
        return newFloatingBaseDisturbance_; 
      }

      // Read-only access to active floating base disturbance (Not thread safe while MPC is running!)
      const Eigen::Matrix<ocs2::scalar_t, 6, 1>& getActiveFloatingBaseDisturbance() const 
      { 
        return activeFloatingBaseDisturbance_; 
      }

    private:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      // Parameters active in the current MPC optimization

      // External wrench that acts on base expressed in base frame
      Eigen::Matrix<ocs2::scalar_t, 6, 1> activeFloatingBaseDisturbance_;

      // Updated externally, becomes active in next MPC iteration
      
      // External wrench that acts on base expressed in base frame
      ocs2::Synchronized<Eigen::Matrix<ocs2::scalar_t, 6, 1>> newFloatingBaseDisturbance_;

  };

};  // namespace legged_locomotion_mpc 