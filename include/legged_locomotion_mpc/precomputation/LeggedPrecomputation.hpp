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
    LeggedPrecomputation(floating_base_model::FloatingBaseModelInfo modelInfo,
      const LeggedReferenceManager& referenceManager,
      const PinocchioForwardEndEffectorKinematicsCppAd& forwardKinematics,
      const PinocchioTorqueApproximationCppAd& torqueApproximator);

    ~LeggedPrecomputation() override = default;

    LeggedPrecomputation *clone() const override { return new LeggedPrecomputation(*this); }

    void request(ocs2::RequestSet request, scalar_t t, const vector_t &x, const vector_t &u) override;

    const vector3_t& getEndEffectorPosition(size_t endEffectorIndex);

    const ocs2::VectorFunctionLinearApproximation& getEndEffectorPositionDerivatives(
      size_t endEffectorIndex);

    const vector3_t& getEndEffectorVelocity(size_t endEffectorIndex);

    const ocs2::VectorFunctionLinearApproximation& getEndEffectorVelocityDerivatives(
      size_t endEffectorIndex);

    const vector_t& getApproximatedJointTorques();

    const ocs2::VectorFunctionLinearApproximation& getApproximatedJointTorquesDerivatives();

    bool getContactFlag(size_t endEffectorIndex);

    const matrix3_t& getRotationWorldToTerrain(size_t endEffectorIndex);

    const vector3_t& getSurfaceNormal(size_t endEffectorIndex);

    private:

      LeggedPrecomputation(const LeggedPrecomputation& other);

      void updateSwingData(ocs2::scalar_t time);

      void updateEndEffectorKinematics(ocs2::scalar_t time);

      floating_base_model::FloatingBaseModelInfo modelInfo_;

      const LeggedReferenceManager& referenceManager_;
      const PinocchioForwardEndEffectorKinematicsCppAd& forwardKinematics_;
      const PinocchioTorqueApproximationCppAd& torqueApproximator_;

      std::vector<vector3_t> endEffectorPositions_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorPositionDerivaties_;

      std::vector<vector3_t> endEffectorVelocities_;
      std::vector<ocs2::VectorFunctionLinearApproximation> endEffectorVelocitynDerivaties_;

      contact_flags_t contactFlags_;

      std::vector<matrix3_t> rotationWorldToTerrains_;
      std::vector<vector3_t> surfaceNormals_;

      ocs2::vector_t torqueApproximation_;
      ocs2::VectorFunctionLinearApproximation torqueApproximationDerivatives_;

  };
} // namespace legged_locomotion_mpc

#endif