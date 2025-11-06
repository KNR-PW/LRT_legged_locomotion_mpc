#ifndef __ZERO_3_DOF_VELOCITY_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __ZERO_3_DOF_VELOCITY_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/reference_manager/SwitchedModelReferenceManager.hpp">

namespace legged_locomotion_mpc
{
    /**
    * Specializes the CppAd version of zero velocity constraint on an end-effector linear velocity.
    */
  class Zero3DofVelocityConstraint final : public ocs2::StateInputConstraint {
    public:

      /**
       * Constructor
       * @param [in] referenceManager : Switched model ReferenceManager
       * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
       * @param [in] contactPointIndex : The 3 DoF contact index.
       */
      Zero3DofVelocityConstraint(const SwitchedModelReferenceManager &referenceManager,
        const ocs2::PinocchioEndEffectorKinematicsCppAd &endEffectorKinematics,
        size_t contactPointIndex);

      ~Zero3DofVelocityConstraint() override = default;

      Zero3DofVelocityConstraint *clone() const override { return new Zero3DofVelocityConstraint(*this); }

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override { return 3; }

      ocs2::vector_t getValue(ocs2::scalar_t time,
        const ocs2::vector_t &state, const ocs2::vector_t &input,
        const PreComputation &preComp) const override;

        ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time,
        const ocs2::vector_t &state, const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

    private:
      Zero3DofVelocityConstraint(const Zero3DofVelocityConstraint &rhs);

      const SwitchedModelReferenceManager *referenceManagerPtr_;
      std::unique_ptr<ocs2::PinocchioEndEffectorKinematicsCppAd> endEffectorKinematicsPtr_;
      const size_t contactPointIndex_;
    };

} // namespace legged_locomotion_mpc

#endif