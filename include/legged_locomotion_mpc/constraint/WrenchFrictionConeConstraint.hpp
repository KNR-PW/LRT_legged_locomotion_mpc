
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
      * footLengthX: rectangular foot dimension in X axis w.r.t. foot frame 
      *  (check linked pdf for details)
      * footLengthY: rectangular foot dimension in Y axis w.r.t. foot frame 
      *  (check linked pdf for details)
      */
      struct Config 
      {
        ocs2::scalar_t frictionCoefficient_;
        ocs2::scalar_t footHalfLengthX_;
        ocs2::scalar_t footHalfLengthY_;

        explicit Config(ocs2::scalar_t frictionCoefficientParam = 0.7,
          ocs2::scalar_t footLengthX,
          ocs2::scalar_t footLengthY);

      };

      /**
       * Constructor
       * @param [in] referenceManager : Switched model ReferenceManager.
       * @param [in] config : Friction model settings.
       * @param [in] contactFeetIndex : The 6 DoF contact index.
       * @param [in] info : The centroidal model information.
       */
      WrenchFrictionConeConstraint(const SwitchedModelReferenceManager &referenceManager,
        Config config,
        size_t contactFeetIndex,
        FloatingBaseModelInfo& info);

      ~WrenchFrictionConeConstraint() override = default;

      ForceWrenchFrictionConeConstraint* clone() const override { return new WrenchFrictionConeConstraint(*this); }

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override { return 16; };

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

      WrenchFrictionConeConstraint(const WrenchFrictionConeConstraint &other) = default;

      Eigen::Matrix<ocs2::scalar_t, 16, 6> generateConeConstraintMatrix(const Config& config);

      const SwitchedModelReferenceManager *referenceManagerPtr_;

      const Config config_;
      const size_t contactFeetIndex_;
      const FloatingBaseModelInfo* info_;

      ocs2::VectorFunctionLinearApproximation linearApproximation_;

      Eigen::Matrix<ocs2::scalar_t, 16, 6> coneConstraintMatrix_;

      // rotation world to terrain, normal to contact 
      // for 6D contact is the same as rotation matrix from world to foot 
      // (when foot is in contact, it has to lay flat on terrain, so foot frame of r
      // reference is same as normal to terrain)
      matrix3_t rotationWorldToTerrain_;
  };
}; // namespace legged_locomotion_mpc

#endif