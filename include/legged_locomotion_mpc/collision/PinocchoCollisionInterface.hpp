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
   Based on: Farbod Farshidian (https://github.com/leggedrobotics/ocs2)
 */

#ifndef __PINOCCHIO_COLLISION_INTERFACE_LEGGED_LOCOMOTION_MPC__
#define __PINOCCHIO_COLLISION_INTERFACE_LEGGED_LOCOMOTION_MPC__

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_sphere_approximation/SphereApproximation.h>

#include <floating_base_model/FloatingBaseModelInfo.hpp>

#include <legged_locomotion_mpc/common/Types.hpp>

namespace legged_locomotion_mpc
{
  namespace collision
  {
    /**
    * Interface for all collsion objects. 
    * 
    * There are two types of collisions, terrain collision and self collision. 
    * By default all end effectors are included to terrain collision. User can add new 
    * collision objects by link name in constructor.
    * 
    * Every object that user wants to add needs to have primitive collision, 
    * otherwise exception will be thrown.
    * 
    * Currently the following primitive geometries are supported: box, cylinder, and sphere.
    *
    * Reference:
    * [1] A. Voelz and K. Graichen, "Computation of Collision Distance and Gradient using an Automatic Sphere Approximation of the Robot Model
    * with Bounded Error," ISR 2018; 50th International Symposium on Robotics, 2018, pp. 1-8.
    * 
    * @warning 3 DoF end effector collison model i assumed to be sphere with center postion 
    * located in the same positon as end effector frame!
    */
    class PinocchioCollisionInterface
    {
      public:
        /**
         * Constructor
         * @param [in] info: Floating Base model info.
         * @param [in] pinocchioInterface: Pinocchio interface of Floating Base model
         * @param [in] otherCollisionLinks: Other collision links for terrain or
         * self collision
         * @param [in] maxExcesses: vector of maximum allowed distances between 
         * the surfaces of the collision primitives and collision spheres
         * @param [in] shrinkRatio: shrinking ratio for maxExcess to 
         * recursively approximate the circular base of the cylinder when more than one
         * collision sphere is required along the radial direction
         * @param [in] modelFolder: folder to save the model library files to
         * @param [in] recompileLibraries: If true, the model library will be newly
         * compiled. If false, an existing library will be loaded if available.
         * @param [in] verbose: print information.
         */
        PinocchioCollisionInterface(
          floating_base_model::FloatingBaseModelInfo info,
          const ocs2::PinocchioInterface& pinocchioInterface,
          std::vector<std::string> otherCollisionLinks, 
          const std::vector<ocs2::scalar_t>& maxExcesses, ocs2::scalar_t shrinkRatio,
          const std::string& modelFolder = "/tmp/ocs2",
          bool recompileLibraries = true,
          bool verbose = false);
        
        /**
         * Get vector with number of spheres in every geometry in frame
         * Rule of indexes:
         * 1. 3 DoF end effectors
         * 2. 6 DoF end effectors
         * 3. other collision objects, as defined in otherCollisionLinks
         */
        const std::vector<size_t>& getFrameSphereNumbers(size_t collisionIndex) const;

        /**
         * Get vector with radius of every sphere in every geometry in frame
         * Rule of indexes:
         * 1. 3 DoF end effectors
         * 2. 6 DoF end effectors
         * 3. other collision objects, as defined in otherCollisionLinks
         */
        const std::vector<ocs2::scalar_t>& getFrameSphereRadiuses(size_t collisionIndex) const;

        /**
         * Get vector with position of every sphere in every geometry with respect to frame
         * Rule of indexes:
         * 1. 3 DoF end effectors
         * 2. 6 DoF end effectors
         * 3. other collision objects, as defined in otherCollisionLinks
         */
        const std::vector<vector3_t>& getFrameSpherePositions(size_t collisionIndex) const;
        
        /**
         * Get partial derivative w.r.t euler ZYX angles 
         * from rotation matrix and vector multiplication: d(R * v)/de.
         * Used for calculating partial derivatives of sphere positions.
         */
        matrix3_t getRotationTimesVectorGradient(const vector3_t& eulerAnglesZYX, 
          const vector3_t& vector) const;

        const pinocchio::GeometryModel& getGeometryModel() const;

      private:
        
        // Cpp AD version of rotation matrix and vector multiplication: R * v
        ocs2::ad_vector_t getRotationTimesVectorCppAd(
          const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& eulerAnglesAD, 
          const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& vector);
        
        pinocchio::GeometryModel geometryModel_;

        size_t frameNumber_;
        
        // Number of spheres in every object of every frame
        std::vector<std::vector<size_t>> sphereNumbers_;

        // Radius values of spheres of every object of every frame
        std::vector<std::vector<ocs2::scalar_t>> sphereRadiuses_;

        // Positions of sphere centers relative to their parent frame
        std::vector<std::vector<vector3_t>> frameToSpherePositons_;

        /**
         * Helper Cpp AD function for getting partial derivative w.r.t euler ZYX angles
         * from rotation matrix and vector multiplication: d(R * v)/de
         */
        std::unique_ptr<ocs2::CppAdInterface> rotationMatrixVectorAdInterfacePtr_;
    };
  } // namespace collision
} // namespace legged_locomotion_mpc

#endif