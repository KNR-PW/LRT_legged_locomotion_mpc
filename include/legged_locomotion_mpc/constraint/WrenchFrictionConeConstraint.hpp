
#ifndef __FORCE_WRENCH_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __FORCE_WRENCH_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>

#include "legged_locomotion_mpc/common/Types.hpp"
#include "legged_locomotion_mpc/reference_manager/SwitchedModelReferenceManager.hpp"

using namespace floating_base_model;

namespace legged_locomotion_mpc
{
  /**
   * Implements the linear wrench cone constraint h(t,x,u) >= 0: (https://hal.science/hal-02108449/document)
   */
  class WrenchFrictionConeConstraint final : public ocs2::StateInputConstraint 
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
    
      /**
      * frictionCoefficient: The coefficient of friction.
      * regularization: A positive number to regulize the friction constraint. refer to the FrictionConeConstraint documentation.
      * gripperForce: Gripper force in normal direction.
      * hessianDiagonalShift: The Hessian shift to assure a strictly-convex quadratic constraint approximation.
      */
      struct Config 
      {
        ocs2::scalar_t frictionCoefficient_;
        ocs2::scalar_t rectangleX_;
        ocs2::scalar_t rectangleY_;

        explicit Config(ocs2::scalar_t frictionCoefficientParam = 0.7,
          ocs2::scalar_t rectangleX_,
          ocs2::scalar_t rectangleY_);

      };

      /**
       * Constructor
       * @param [in] referenceManager : Switched model ReferenceManager.
       * @param [in] config : Friction model settings.
       * @param [in] contactPointIndex : The 3 DoF contact index.
       * @param [in] info : The centroidal model information.
       */
      WrenchFrictionConeConstraint(const SwitchedModelReferenceManager &referenceManager,
        Config config,
        size_t contactPointIndex,
        FloatingBaseModelInfo& info);

      ~WrenchFrictionConeConstraint() override = default;

      ForceWrenchFrictionConeConstraint* clone() const override { return new WrenchFrictionConeConstraint(*this); }

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override { return 1; };

      ocs2::vector_t getValue(scalar_t time, const vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

      ocs2::VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
        const ocs2::vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

      /**
       * Set surface normal vector for contact
       * @param [in] surfaceNormalInWorld: sufrace normal in world frame
       */
      void setSurfaceNormalInWorld(const vector3_t &surfaceNormalInWorld);

    private:
      struct LocalForceDerivatives 
      {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        matrix3_t dW_du; // derivative local wrench w.r.t. forces in world frame
      };

      struct ConeLocalDerivatives 
      {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        vector3_t dCone_dw; // derivative w.r.t local wrench
      };

      struct ConeDerivatives 
      {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        vector3_t dCone_du;
      };

      WrenchFrictionConeConstraint(const WrenchFrictionConeConstraint &other) = default;

      ocs2::vector_t coneConstraint(const vector3_t &localForces) const;

      LocalForceDerivatives computeLocalForceDerivatives(const vector3_t &forcesInBodyFrame) const;

      ConeLocalDerivatives computeConeLocalDerivatives(const vector3_t &localForces) const;

      ConeDerivatives computeConeConstraintDerivatives(const ConeLocalDerivatives &coneLocalDerivatives,
        const LocalForceDerivatives &localForceDerivatives) const;

      ocs2::matrix_t frictionConeInputDerivative(size_t inputDim,
        const ConeDerivatives &coneDerivatives) const;

      ocs2::matrix_t frictionConeSecondDerivativeInput(size_t inputDim,
        const ConeDerivatives &coneDerivatives) const;

      ocs2::matrix_t frictionConeSecondDerivativeState(size_t stateDim,
        const ConeDerivatives &coneDerivatives) const;

      const SwitchedModelReferenceManager *referenceManagerPtr_;

      const Config config_;
      const size_t contactPointIndex_;
      const FloatingBaseModelInfo* info_;

      // rotation world to terrain, normal to contact 
      matrix3_t rotationWorldToTerrain_ = matrix3_t::Identity();
  };
}; // namespace legged_locomotion_mpc

#endif