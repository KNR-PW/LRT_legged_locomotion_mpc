// Copyright (c) 2025, Bartłomiej Krajewski
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

#ifndef __PINOCCHIO_FORWARD_COLLISION_KINEMATICS_CPP_AD__
#define __PINOCCHIO_FORWARD_COLLISION_KINEMATICS_CPP_AD__

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>
#include <floating_base_model/FloatingBaseModelPinocchioMapping.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>
#include <legged_locomotion_mpc/collision/PinocchoCollisionInterface.hpp>

namespace legged_locomotion_mpc 
{

  /**
   * This class provides the CppAD implementation of the forward kinematics 
   * for collision links (not their spheres!) other than end effectors, based on pinocchio. 
   * No pre-computation is required. 
   * Position and orientation are expressed with respect to the world inertial frame.
   */
  class PinocchioForwardCollisionKinematicsCppAd
  {
    public:

    /** Constructor
     * @param [in] pinocchioInterface: pinocchio interface.
     * @param [in] info: info of kinematics model
     * @param [in] collisionNames: collision frames
     * @param [in] modelName: name of the generate model library
     * @param [in] modelFolder: folder to save the model library files to
     * @param [in] recompileLibraries: If true, the model library will be newly
     * compiled. If false, an existing library will be loaded if available.
     * @param [in] verbose: print information.
     */
    PinocchioForwardCollisionKinematicsCppAd(
        const ocs2::PinocchioInterface& pinocchioInterface,
        const floating_base_model::FloatingBaseModelInfo info,
        const std::vector<std::string>& collisionNames,
        const std::string& modelName,
        const std::string& modelFolder = "/tmp/ocs2",
        bool recompileLibraries = true, bool verbose = false);

    ~PinocchioForwardCollisionKinematicsCppAd() = default;
    PinocchioForwardCollisionKinematicsCppAd* clone() const;
    PinocchioForwardCollisionKinematicsCppAd& operator =(
        const PinocchioForwardCollisionKinematicsCppAd&) = delete;

    std::vector<vector3_t> getPosition(const ocs2::vector_t& state) const;

    std::vector<vector3_t> getOrientation(const ocs2::vector_t& state) const;

    std::vector<ocs2::VectorFunctionLinearApproximation> getPositionLinearApproximation(
        const ocs2::vector_t& state) const;

    std::vector<ocs2::VectorFunctionLinearApproximation> getOrientationLinearApproximation(
      const ocs2::vector_t& state) const;

    size_t getCollisionNumber() const;

   private:
    PinocchioForwardCollisionKinematicsCppAd(
        const PinocchioForwardCollisionKinematicsCppAd& rhs);

    ocs2::ad_vector_t getPositionCppAd(
      ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const floating_base_model::FloatingBaseModelPinocchioMappingCppAd& mapping,
        const ocs2::ad_vector_t& state);
    
    ocs2::ad_vector_t getOrientationCppAd(
      ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
        const floating_base_model::FloatingBaseModelPinocchioMappingCppAd& mapping,
        const ocs2::ad_vector_t& state);

    std::unique_ptr<ocs2::CppAdInterface> positionCppAdInterfacePtr_;
    std::unique_ptr<ocs2::CppAdInterface> orientationCppAdInterfacePtr_;

    const size_t numCollisions_;
    std::vector<size_t> collisionFrameIndices_;

    const size_t numEndEffectors_;
  };
};  // namespace legged_locomotion_mpc

#endif