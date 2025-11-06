#ifndef __NORMAL_VELOCITY_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __NORMAL_VELOCITY_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <legged_locomotion_mpc/common/ModelSettings.hpp>
#include <legged_locomotion_mpc/reference_manager/SwitchedModelReferenceManager.hpp>

namespace legged_locomotion_mpc
{
    /**
    * Specializes the CppAd version of normal to terrain
    * linear velocity constraint on an end-effector linear velocity.
    */
  class NormalVelocityConstraint final : public ocs2::StateInputConstraint {
    public:

      /**
       * Constructor
       * @param [in] referenceManager : Switched model ReferenceManager
       * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
       * @param [in] contactPointIndex : The 3 DoF contact index.
       */
      NormalVelocityConstraint(const SwitchedModelReferenceManager &referenceManager,
        const ocs2::PinocchioEndEffectorKinematicsCppAd &endEffectorKinematics,
        size_t contactPointIndex);

      ~NormalVelocityConstraint() override = default;

      NormalVelocityConstraint *clone() const override { return new NormalVelocityConstraint(*this); }

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override { return 1; }

      ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t &state,
        const ocs2::vector_t &input, const PreComputation &preComp) const override;

        ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time,
        const ocs2::vector_t &state, const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

    private:
      NormalVelocityConstraint(const NormalVelocityConstraint &rhs);

      /**
       * Set surface normal vector for contact
       * @param [in] surfaceNormalInWorld: sufrace normal in world frame
       */
      void setSurfaceNormalInWorld(const vector3_t &surfaceNormalInWorld);

      const SwitchedModelReferenceManager *referenceManagerPtr_;
      std::unique_ptr<ocs2::PinocchioEndEffectorKinematicsCppAd> endEffectorKinematicsPtr_;
      const size_t contactPointIndex_;

      // normal vector to contact 
      vector3_t surfaceNormalInWorld_ = vector3_t(0.0, 0.0, 1.0);
    };

} // namespace legged_locomotion_mpc

#endif