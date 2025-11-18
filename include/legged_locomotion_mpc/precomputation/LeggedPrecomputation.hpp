// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */


#ifndef __LEGGED_PRECOMPUTATION_LEGGED_LOCOMOTION_MPC__
#define __LEGGED_PRECOMPUTATION_LEGGED_LOCOMOTION_MPC__

#include <ocs2_core/PreComputation.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <terrain_model/core/TerrainModel.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/reference_manager/LeggedReferenceManager.hpp>
#include <legged_locomotion_mpc/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.hpp>
#include <legged_locomotion_mpc/torque_approx/PinocchioTorqueApproximationCppAd.hpp>

namespace legged_locomotion_mpc
{
  class LeggedPrecomputation: public ocs2::PreComputation
  {
    public:

      LeggedPrecomputation(floating_base_model::FloatingBaseModelInfo modelInfo,
        const LeggedReferenceManager& referenceManager,
        const PinocchioForwardEndEffectorKinematicsCppAd& forwardKinematics,
        const PinocchioTorqueApproximationCppAd& torqueApproximator);

      ~LeggedPrecomputation() override = default;

      LeggedPrecomputation* clone() const override;

      void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t &x, 
        const ocs2::vector_t &u) override;

      const vector3_t& getEndEffectorPosition(size_t endEffectorIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getEndEffectorPositionDerivatives(
        size_t endEffectorIndex) const;

      const vector3_t& getEndEffectorOrientation(size_t endEffectorIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getEndEffectorOrientationDerivatives(
        size_t endEffectorIndex) const;

      const vector3_t& getEndEffectorLinearVelocity(size_t endEffectorIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getEndEffectorLinearVelocityDerivatives(
        size_t endEffectorIndex) const;

      const vector3_t& getEndEffectorAngularVelocity(size_t endEffectorIndex) const;

      const ocs2::VectorFunctionLinearApproximation& getEndEffectorAngularVelocityDerivatives(
        size_t endEffectorIndex) const;

      const ocs2::vector_t& getApproximatedJointTorques() const;

      const ocs2::VectorFunctionLinearApproximation& getApproximatedJointTorquesDerivatives() const;

      const matrix3_t& getRotationWorldToTerrain(size_t endEffectorIndex) const;

      const vector3_t& getSurfaceNormal(size_t endEffectorIndex) const;

      const vector3_t& getReferenceEndEffectorLinearVelocity(size_t endEffectorIndex) const;

    private:

      LeggedPrecomputation(const LeggedPrecomputation& other);

      void updateContactData(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::vector_t& input);

      void updateEndEffectorKinematicsData(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::vector_t& input);
      
      void updateEndEffectorKinematicsDerivatives(ocs2::scalar_t time, 
        const ocs2::vector_t& state, const ocs2::vector_t& input);

      void updateApproximatedTorquesData(ocs2::scalar_t time, const ocs2::vector_t& state, 
        const ocs2::vector_t& input);

      void updateReferenceEndEffectorVelocities(ocs2::scalar_t time);

      const floating_base_model::FloatingBaseModelInfo modelInfo_;
      const size_t endEffectorNumber_;

      const LeggedReferenceManager& referenceManager_;
      const PinocchioForwardEndEffectorKinematicsCppAd& forwardKinematics_;
      const PinocchioTorqueApproximationCppAd& torqueApproximator_;

      std::vector<vector3_t> endEffectorPositions_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorPositionDerivaties_;

      std::vector<vector3_t> endEffectorEulerAngles_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorEulerAngleDerivaties_;

      std::vector<vector3_t> endEffectorLinearVelocities_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorLinearVelocityDerivaties_;

      std::vector<vector3_t> endEffectorAngularVelocities_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorAngularVelocityDerivaties_;

      std::vector<matrix3_t> rotationWorldToTerrains_;
      std::vector<vector3_t> surfaceNormals_;
      
      // Calculated from target trajectories
      std::vector<vector3_t> referenceEndEffectorLinearVelocities_;

      ocs2::vector_t torqueApproximation_;
      ocs2::VectorFunctionLinearApproximation torqueApproximationDerivatives_;
  };
} // namespace legged_locomotion_mpc

#endif