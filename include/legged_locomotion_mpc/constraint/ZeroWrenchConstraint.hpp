
#ifndef __ZERO_WRENCH_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __ZERO_WRENCH_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>

#include "legged_locomotion_mpc/reference_manager/SwitchedModelReferenceManager.hpp"

namespace legged_locomotion_mpc
{
  class ZeroWrenchConstraint final : public ocs2::StateInputConstraint 
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:

      /** 
       * Constructor
       * @param [in] referenceManager: Switched model ReferenceManager
       * @param [in] contactPointIndex: The 3 DoF contact index
       * @param [in] info: info of FloatingBase model
       * @param [in] stateDim: size of state vector
       * @param [in] inputDim: size of input vector
       */
      ZeroWrenchConstraint(const SwitchedModelReferenceManager &referenceManager,
          size_t contactPointIndex,
          FloatingBaseModelInfo& info,
          size_t stateDim,
          size_t inputDim);

      ~ZeroWrenchConstraint() override = default;

      ZeroWrenchConstraint *clone() const override { return new ZeroWrenchConstraint(*this); }

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override { return 6; }

      vector_t getValue(ocs2::scalar_t time,
        const ocs2::vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

      VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, 
        const ocs2::vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

    private:

      ZeroWrenchConstraint(const ZeroWrenchConstraint &other) = default;

      const SwitchedModelReferenceManager *referenceManagerPtr_;
      const size_t contactPointIndex_;
      const FloatingBaseModelInfo* info_;
      ocs2::VectorFunctionLinearApproximation approx_;

  };

}; // namespace legged_locomotion_mpc

#endif