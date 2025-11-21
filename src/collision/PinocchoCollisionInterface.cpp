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
   Based on: Farbod Farshidian (https://github.com/leggedrobotics/ocs2)
 */

#include <legged_locomotion_mpc/collision/PinocchoCollisionInterface.hpp>

#include <urdf_parser/urdf_parser.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/geometry.hpp>

namespace legged_locomotion_mpc
{
  namespace collision
  {

    using namespace ocs2;
    using namespace floating_base_model;
    PinocchioCollisionInterface::PinocchioCollisionInterface(
      const FloatingBaseModelInfo info,
      const PinocchioInterface& pinocchioInterface,
      std::vector<std::string> otherCollisionLinks, 
      const std::vector<scalar_t>& maxExcesses, scalar_t shrinkRatio)
    {
      if (!pinocchioInterface.getUrdfModelPtr()) 
      {
        throw std::runtime_error(
          "[PinocchioCollisionInterface]: The PinocchioInterface passed to "
          "PinocchioGeometryInterface(...) "
          "does not contain a urdf model!");
      } 

      const std::unique_ptr<const TiXmlDocument> urdfAsXml(urdf::exportURDF(
        *pinocchioInterface.getUrdfModelPtr()));
      TiXmlPrinter printer;
      urdfAsXml->Accept(&printer);
      const std::stringstream urdfAsStringStream(printer.Str());

      geometryModel_ = pinocchio::GeometryModel();
      pinocchio::urdf::buildGeom(pinocchioInterface.getModel(), urdfAsStringStream, 
      pinocchio::COLLISION, geometryModel_);

      // Get collision frame indexes
      std::vector<size_t> collisionFrames = info.endEffectorFrameIndices;

      for(const std::string& frameName: otherCollisionLinks)
      {
        const size_t frameIndex = pinocchioInterface.getModel().getFrameId(frameName);
        collisionFrames.push_back(frameIndex);
      }

      frameNumber_ = collisionFrames.size();

      for(size_t i = 0; i < collisionFrames.size(); ++i)
      {
        const size_t frameIndex = collisionFrames[i];
        const vector3_t framePosition = pinocchioInterface.getModel().frames[
          frameIndex].placement.translation();

        std::vector<size_t> sphereNumbers;
        std::vector<scalar_t> sphereRadiuses;
        std::vector<vector3_t> spherePositions;

        for (size_t j = 0; j < geometryModel_.geometryObjects.size(); ++j) 
        {
          const pinocchio::GeometryObject& object = geometryModel_.geometryObjects[j];
          const size_t parentFrameIndex = object.parentFrame;
          const vector3_t objectCenterPosition = object.placement.translation();
          const vector3_t frameToCenterPositon = objectCenterPosition - framePosition;
          if (parentFrameIndex == frameIndex) 
          {
            const SphereApproximation sphereApproximation = SphereApproximation(*object.geometry, j, 
              maxExcesses[i], shrinkRatio);

            const size_t sphereNumber = sphereApproximation.getNumSpheres();
            sphereNumbers.push_back(sphereNumber);

            const scalar_t sphereRadius = sphereApproximation.getSphereRadius();
            sphereRadiuses.insert(sphereRadiuses.end(), sphereNumber, sphereRadius);

            const auto& centerToSpherePositons = sphereApproximation.getSphereCentersToObjectCenter();

            for(size_t k = 0; k < sphereNumber; ++k)
            {
              const vector3_t frameToSphere = frameToCenterPositon + centerToSpherePositons[k];
              spherePositions.push_back(std::move(frameToSphere));
            }
          }
        }
        sphereNumbers_.push_back(std::move(sphereNumbers));
        sphereRadiuses_.push_back(std::move(sphereRadiuses));
        frameToSpherePositons_.push_back(std::move(spherePositions));
      }
    }

    const std::vector<size_t>& PinocchioCollisionInterface::getFrameSphereNumbers(
      size_t collisionIndex)
    {
      assert(collisionIndex < frameNumber_);
      return sphereNumbers_[collisionIndex];
    }

    const std::vector<scalar_t>& PinocchioCollisionInterface::getFrameSphereRadiuses(
      size_t collisionIndex)
    {
      assert(collisionIndex < frameNumber_);
      return sphereRadiuses_[collisionIndex];
    }

    const std::vector<vector3_t>& PinocchioCollisionInterface::getFrameSpherePositions(
      size_t collisionIndex)
    {
      assert(collisionIndex < frameNumber_);
      return frameToSpherePositons_[collisionIndex];
    }
  } // namespace collision
} // namespace legged_locomotion_mpc
