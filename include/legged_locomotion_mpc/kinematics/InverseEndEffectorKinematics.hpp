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


#ifndef __INVERSE_END_EFFECTOR_KINEMATICS_LEGGED_LOCOMOTION_MPC__
#define __INVERSE_END_EFFECTOR_KINEMATICS_LEGGED_LOCOMOTION_MPC__

#include <legged_locomotion_mpc/common/Types.hpp>

#include <lrt_inverse_kinematics/InverseKinematics.hpp>

namespace legged_locomotion_mpc
{
  class InverseEndEffectorKinematics
  {
    public:
      InverseEndEffectorKinematics(lrt_inverse_kinematics::InverseKinematics&& kinematicsSolver);

      ocs2::vector_t getJointPositions(const ocs2::vector_t& actualJointPositions, 
        const vector6_t basePose, 
        const std::vector<vector3_t>& endEffectorPositions);

      ocs2::vector_t getJointVelocities(const ocs2::vector_t& actualJointPositions,
        const vector6_t basePose, const vector6_t baseVelocity,
        const std::vector<vector3_t>& endEffectorVelocities);
    
    private:

      lrt_inverse_kinematics::InverseKinematics kinematicsSolver_;
  };

  lrt_inverse_kinematics::IKModelInfo loadIKModelInfo(const std::string &filename, bool verbose = true);

  lrt_inverse_kinematics::IKSolverInfo loadIKSolverInfo(const std::string &filename, bool verbose = true);

  const std::string loadIKSolverName(const std::string &filename, bool verbose = true);
}; // namespace legged_locomotion_mpc

#endif