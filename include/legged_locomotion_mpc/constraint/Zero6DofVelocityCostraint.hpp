#ifndef __ZERO_6_DOF_VELOCITY_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __ZERO_6_DOF_VELOCITY_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <legged_locomotion_mpc/common/Pinocchio6DofEndEffectorKinematicsCppAd.hpp>
#include "legged_locomotion_mpc/reference_manager/SwitchedModelReferenceManager.hpp"

namespace legged_locomotion_mpc
{
    /**
    * Specializes the CppAd version of zero velocity constraint on an end-effector linear velocity.
    */
  class Zero6DofVelocityConstraint final : public ocs2::StateInputConstraint {
    public:

      /**
       * Constructor
       * @param [in] referenceManager : Switched model ReferenceManager
       * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
       * @param [in] contactFeetIndex : The 6 DoF contact index.
       */
      Zero6DofVelocityConstraint(const SwitchedModelReferenceManager &referenceManager,
        const ocs2::Pinocchio6DofEndEffectorKinematicsCppAd &endEffectorKinematics,
        size_t contactFeetIndex);

      ~Zero6DofVelocityConstraint() override = default;

      Zero6DofVelocityConstraint *clone() const override { return new Zero6DofVelocityConstraint(*this); }

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override { return 3; }

      ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t &state,
        const ocs2::vector_t &input,
        const PreComputation &preComp) const override;

        ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time,
        const ocs2::vector_t &state, const vector_t &input,
        const ocs2::PreComputation &preComp) const override;

    private:
      Zero6DofVelocityConstraint(const Zero6DofVelocityConstraint &rhs);

      const SwitchedModelReferenceManager *referenceManagerPtr_;
      std::unique_ptr<Pinocchio6DofEndEffectorKinematicsCppAd> endEffectorKinematicsPtr_;
      const size_t contactFeetIndex_;
    };

};

#endif