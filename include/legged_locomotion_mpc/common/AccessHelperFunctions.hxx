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

namespace legged_locomotion_mpc
{
  namespace access_helper_functions
  {
    using namespace floating_base_model;
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBasePosition(Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<Derived, 3, 1>(robotState.derived(), 6, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBasePosition(
      const Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<const Derived, 3, 1>(robotState.derived(), 6, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseOrientationZyx(Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<Derived, 3, 1>(robotState.derived(), 9, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseOrientationZyx(
      const Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<const Derived, 3, 1>(robotState.derived(), 9, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 6, 1> getBasePose(Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<Derived, 6, 1>(robotState.derived(), 6, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getBasePose(
      const Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<const Derived, 6, 1>(robotState.derived(), 6, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseLinearVelocity(Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<Derived, 3, 1>(robotState.derived(), 0, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseLinearVelocity(
      const Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<const Derived, 3, 1>(robotState.derived(), 0, 0);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 3, 1> getBaseAngularVelocity(Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<Derived, 3, 1>(robotState.derived(), 3, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 3, 1> getBaseAngularVelocity(
      const Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<const Derived, 3, 1>(robotState.derived(), 3, 0);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, 6, 1> getBaseVelocity(Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<Derived, 6, 1>(robotState.derived(), 0, 0);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, 6, 1> getBaseVelocity(
      const Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      return Eigen::Block<const Derived, 6, 1>(robotState.derived(), 0, 0);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, Eigen::Dynamic, 1> getJointPositions(
      Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      const size_t startRow = 12;
      return Eigen::Block<Derived, Eigen::Dynamic, 1>(robotState.derived(), startRow, 0, info.actuatedDofNum, 1);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, Eigen::Dynamic, 1> getJointPositions(
      const Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      const size_t startRow = 12;
      return Eigen::Block<const Derived, Eigen::Dynamic, 1>(robotState.derived(), startRow, 0, info.actuatedDofNum, 1);
    };
    
    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    Eigen::Block<Derived, Eigen::Dynamic, 1> getJointVelocities(Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      const size_t startRow = 12 + info.actuatedDofNum;
      return Eigen::Block<Derived, Eigen::Dynamic, 1>(robotState.derived(), startRow, 0, info.actuatedDofNum, 1);
    };

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    template <typename Derived, typename SCALAR_T>
    const Eigen::Block<const Derived, Eigen::Dynamic, 1> getJointVelocities(
      const Eigen::MatrixBase<Derived>& robotState,
      const FloatingBaseModelInfoTpl<SCALAR_T>& info)
    {
      assert(robotState.rows() == info.stateDim + info.actuatedDofNum);
      assert(robotState.cols() == 1);
      const size_t startRow = 12 + info.actuatedDofNum;
      return Eigen::Block<const Derived, Eigen::Dynamic, 1>(robotState.derived(), startRow, 0, info.actuatedDofNum, 1);
    };
  } // namespace access_helper_functions
} // namespace legged_locomotion