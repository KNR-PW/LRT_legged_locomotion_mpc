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

#ifndef __ACCESS_HELPER_FUNCTIONS_LEGGED_LOCOMOTION_MPC__
#define __ACCESS_HELPER_FUNCTIONS_LEGGED_LOCOMOTION_MPC__

#include <Eigen/Dense>

#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>


namespace legged_locomotion_mpc
{
  namespace access_helper_functions
  {
    /**
    * Provides read/write access to the base position.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base position in world frame
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBasePosition(Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base position.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base position in world frame
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBasePosition(
      const Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the base orientation.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base orientation in world frame (euler angles)
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseOrientationZyx(Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base orientation.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base orientation in world frame (euler angles)
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseOrientationZyx(
      const Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the base position and orientation.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base position and orientation in world frame (euler angles)
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 6, 1> getBasePose(Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base position and orientation.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base position and orientation in world frame (euler angles)
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getBasePose(
      const Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the base linear velocity.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base linear velocity in base frame
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseLinearVelocity(Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base linear velocity.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base linear velocity in base frame
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseLinearVelocity(
      const Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);
    
    /**
    * Provides read/write access to the base angular velocity.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base angular velocity in base frame
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseAngularVelocity(Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base angular velocity.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base angular velocity in base frame
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseAngularVelocity(
      const Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the base velocity.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base velocity in base frame
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 6, 1> getBaseVelocity(Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the base velocity.
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with base velocity in base frame
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getBaseVelocity(
      const Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read/write access to the actuated joint positions (angles or displecements).
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with actuated joint positions
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, Eigen::Dynamic, 1> getJointPositions(Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the actuated joint positions (angles or displecements).
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with actuated joint positions
    */
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, Eigen::Dynamic, 1> getJointPositions(
      const Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);
    
    /**
    * Provides read/write access to the actuated joint velocities (angular or linear).
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with actuated joint velocities
    */
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, Eigen::Dynamic, 1> getJointVelocities(Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

    /**
    * Provides read access to the actuated joint velocities (angular or linear).
    * @param [in] robotState: robot state vector defined in Types.hpp
    * @param [in] info: info of FloatingBase model
    * @return block with actuated joint velocities
    */
    template <typename Derived, typename SCALAR_T> 
    const Eigen::Block<const Derived, Eigen::Dynamic, 1> getJointVelocities(
      const Eigen::MatrixBase<Derived>& robotState,
      const floating_base_model::FloatingBaseModelInfoTpl<SCALAR_T>& info);

  }; // namespace access_helper_functions
}; // namespace legged_locomotion

#include "AccessHelperFunctions.hxx"

#endif