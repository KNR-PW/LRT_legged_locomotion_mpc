/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Modified by Bartłomiej Krajewski (https://github.com/BartlomiejK2), 2025

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __FORCE_FRICTION_CONSTRAINT_LEGGED_LOCOMOTION_MPC__
#define __FORCE_FRICTION_CONSTRAINT_LEGGED_LOCOMOTION_MPC__

#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/AccessHelperFunctions.hpp>

#include "legged_locomotion_mpc/common/Types.hpp"
#include "legged_locomotion_mpc/reference_manager/SwitchedModelReferenceManager.hpp"

using namespace floating_base_model;

namespace legged_locomotion_mpc
{
  /**
   * Implements the constraint h(t,x,u) >= 0
   *
   * frictionCoefficient * (Fz + gripperForce) - sqrt(Fx * Fx + Fy * Fy + regularization) >= 0
   *
   * The gripper force shifts the origin of the friction cone down in z-direction by the amount of gripping force available. This makes it
   * possible to produce tangential forces without applying a regular normal force on that foot, or to "pull" on the foot with magnitude up to
   * the gripping force.
   *
   * The regularization prevents the constraint gradient / hessian to go to infinity when Fx = Fz = 0. It also creates a parabolic safety
   * margin to the friction cone. For example: when Fx = Fy = 0 the constraint zero-crossing will be at Fz = 1/frictionCoefficient *
   * sqrt(regularization) instead of Fz = 0
   *
   */
  class ForceFrictionConeConstraint final : public ocs2::StateInputConstraint 
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
        ocs2::scalar_t regularization_;
        ocs2::scalar_t gripperForce_;
        ocs2::scalar_t hessianDiagonalShift_;


        explicit Config(
          ocs2::scalar_t frictionCoefficientParam = 0.7,
          ocs2::scalar_t regularizationParam = 25.0,
          ocs2::scalar_t gripperForceParam = 0.0,
          ocs2::scalar_t hessianDiagonalShiftParam = 1e-6);

      };

      /**
       * Constructor
       * @param [in] referenceManager : Switched model ReferenceManager.
       * @param [in] config : Friction model settings.
       * @param [in] contactPointIndex : The 3 DoF contact index.
       * @param [in] info : The centroidal model information.
       */
      ForceFrictionConeConstraint(const SwitchedModelReferenceManager &referenceManager,
        Config config,
        size_t contactPointIndex,
        FloatingBaseModelInfo& info);

      ~ForceFrictionConeConstraint() override = default;

      ForceForceFrictionConeConstraint* clone() const override { return new ForceFrictionConeConstraint(*this); }

      bool isActive(ocs2::scalar_t time) const override;

      size_t getNumConstraints(ocs2::scalar_t time) const override { return 1; };

      ocs2::vector_t getValue(scalar_t time, const vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

      ocs2::VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
        const ocs2::vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

      ocs2::VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, 
        const ocs2::vector_t &state,
        const ocs2::vector_t &input,
        const ocs2::PreComputation &preComp) const override;

      /**
       * Set surface normal vector for contact
       * @param [in] surfaceNormalInWorld: sufrace normal in world frame
       */
      void setSurfaceNormalInWorld(const vector3_t &surfaceNormalInWorld);

    private:
      
      ForceFrictionConeConstraint(const ForceFrictionConeConstraint&other) = default;

      ocs2::vector_t coneConstraint(const vector3_t &localForces) const;

      const SwitchedModelReferenceManager *referenceManagerPtr_;

      const Config config_;
      const size_t contactPointIndex_;
      const FloatingBaseModelInfo* info_;

      ocs2::VectorFunctionLinearApproximation linearApproximation_;
      ocs2::VectorFunctionQuadraticApproximation quadraticApproximation_;

      // rotation world to terrain, normal to contact 
      matrix3_t rotationWorldToTerrain_ = matrix3_t::Identity();

  };
}; // namespace legged_locomotion_mpc

#endif