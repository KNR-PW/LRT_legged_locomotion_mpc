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

#include <legged_locomotion_mpc/collision/PinocchoCollisionInterface.hpp>

#include <stdexcept>

#include <urdf_parser/urdf_parser.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/geometry.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged_locomotion_mpc
{
  namespace collision
  {

    using namespace ocs2;
    using namespace floating_base_model;
    PinocchioCollisionInterface::PinocchioCollisionInterface(
      const FloatingBaseModelInfo& info,
      const ModelSettings& modelSettings,
      const CollisionSettings& collisionSettings,
      const PinocchioInterface& pinocchioInterface,
      const std::string& modelFolder, bool recompileLibraries, bool verbose)
    {
      if(collisionSettings.maxExcesses.size() != (info.endEffectorFrameIndices.size() + collisionSettings.collisionLinkNames.size()))
      {
        throw std::invalid_argument("[PinocchioCollisionInterface]: "
          "maxExcesses is not the same size as collisionLinks!");
      }

      if(!pinocchioInterface.getUrdfModelPtr()) 
      {
        throw std::invalid_argument(
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

      for(const std::string& frameName: collisionSettings.collisionLinkNames)
      {
        const size_t frameIndex = pinocchioInterface.getModel().getFrameId(frameName);
        if(frameIndex <= 0)
        {
          std::string message = "[PinocchioCollisionInterface]: There is no frame named " + frameName + "!";
          throw std::invalid_argument(message);
        }
        collisionFrames.push_back(frameIndex);
      }

      frameNumber_ = collisionFrames.size();

      for(size_t i = 0; i < frameNumber_; ++i)
      {
        const size_t frameIndex = collisionFrames[i];
        const auto& framePlacement = pinocchioInterface.getModel().frames[
          frameIndex].placement;

        std::vector<size_t> sphereNumbers;
        std::vector<scalar_t> sphereRadiuses;
        std::vector<vector3_t> spherePositions;

        for (size_t j = 0; j < geometryModel_.geometryObjects.size(); ++j) 
        {
          const pinocchio::GeometryObject& object = geometryModel_.geometryObjects[j];
          const size_t parentFrameIndex = object.parentFrame;

          if(parentFrameIndex == frameIndex) 
          {
            const auto& objectCenterPlacement = object.placement;
            const auto frameToCenterPlacement = framePlacement.actInv(objectCenterPlacement);
            const auto& frameToCenterPosition = frameToCenterPlacement.translation();
            const auto& centerToFrameRotation = frameToCenterPlacement.rotation();
            
            const SphereApproximation sphereApproximation = SphereApproximation(*object.geometry, j, 
              collisionSettings.maxExcesses[i], collisionSettings.shrinkRatio);

            const size_t sphereNumber = sphereApproximation.getNumSpheres();
            sphereNumbers.push_back(sphereNumber);

            const scalar_t sphereRadius = sphereApproximation.getSphereRadius();
            sphereRadiuses.insert(sphereRadiuses.end(), sphereNumber, sphereRadius);

            const auto& centerToSpherePositions = sphereApproximation.getSphereCentersToObjectCenter();

            for(size_t k = 0; k < sphereNumber; ++k)
            {
              const vector3_t frameToSphere = frameToCenterPosition + centerToFrameRotation * 
                centerToSpherePositions[k];
              spherePositions.push_back(std::move(frameToSphere));
            }
          }
        }
        sphereNumbers_.push_back(std::move(sphereNumbers));
        sphereRadiuses_.push_back(std::move(sphereRadiuses));
        frameToSpherePositons_.push_back(std::move(spherePositions));
      }

      std::vector<std::string> collisionNames = modelSettings.contactNames3DoF;
      collisionNames.insert(collisionNames.end(), 
        modelSettings.contactNames6DoF.begin(), modelSettings.contactNames6DoF.end());

      collisionNames.insert(collisionNames.end(), 
        collisionSettings.collisionLinkNames.begin(), 
        collisionSettings.collisionLinkNames.end());

      // Add all end effectors by default
      for(size_t i = 0; i < info.endEffectorFrameIndices.size(); ++i)
      {
        terrainAvoidanceIndices_.push_back(i);
      }

      for(const auto& terrainLinkName: collisionSettings.terrainCollisionLinkNames)
      {
        const auto iter = std::find(collisionNames.cbegin(), collisionNames.cend(), terrainLinkName);
        if(iter == collisionNames.cend())
        {
          std::string message = "[PinocchioCollisionInterface]: No " + terrainLinkName + " link in all collison links!";
          throw std::invalid_argument(message);
        }
        const size_t index = std::distance(collisionNames.cbegin(), iter);
        terrainAvoidanceIndices_.push_back(index);
      }

      for(const auto& [firstCollisionName, secondCollisionName]: collisionSettings.selfCollisionPairNames)
      {
        const auto firstIter = std::find(collisionNames.cbegin(), collisionNames.cend(), firstCollisionName);
        const auto secondIter = std::find(collisionNames.cbegin(), collisionNames.cend(), secondCollisionName);

        if(firstIter == collisionNames.cend())
        {
          std::string message = "[PinocchioCollisionInterface]: No " + firstCollisionName + " link in all collison links!";
          throw std::invalid_argument(message);
        }

        if(secondIter == collisionNames.cend())
        {
          std::string message = "[PinocchioCollisionInterface]: No " + secondCollisionName + " link in all collison links!";
          throw std::invalid_argument(message);
        }

        const size_t firstIndex = std::distance(collisionNames.cbegin(), firstIter);
        const size_t secondIndex = std::distance(collisionNames.cbegin(), secondIter);

        if(secondIndex < firstIndex)
        {
          selfCollisionIndices_.emplace_back(secondIndex, firstIndex);
        }
        else
        {
          selfCollisionIndices_.emplace_back(firstIndex, secondIndex);
        }
      }

      auto systemFlowMapFunc = [&](const ocs2::ad_vector_t& x, const ocs2::ad_vector_t& p, 
        ocs2::ad_vector_t& y) 
      {
        const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> eulerAnglesAD = x;
        const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> vector = p;
        y = getRotationTimesVectorCppAd(eulerAnglesAD, vector);
      };
    
      rotationMatrixVectorAdInterfacePtr_.reset(
          new ocs2::CppAdInterface(systemFlowMapFunc, 3, 3, "rotation_times_vector_euler", modelFolder));
    
      if(recompileLibraries) {
        rotationMatrixVectorAdInterfacePtr_->createModels(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
      } else {
        rotationMatrixVectorAdInterfacePtr_->loadModelsIfAvailable(ocs2::CppAdInterface::ApproximationOrder::First, verbose);
      }
    }

    const std::vector<size_t>& PinocchioCollisionInterface::getFrameSphereNumbers(
      size_t collisionIndex) const
    {
      assert(collisionIndex < frameNumber_);
      return sphereNumbers_[collisionIndex];
    }

    const std::vector<scalar_t>& PinocchioCollisionInterface::getFrameSphereRadiuses(
      size_t collisionIndex) const
    {
      assert(collisionIndex < frameNumber_);
      return sphereRadiuses_[collisionIndex];
    }

    const std::vector<vector3_t>& PinocchioCollisionInterface::getFrameSpherePositions(
      size_t collisionIndex) const
    {
      assert(collisionIndex < frameNumber_);
      return frameToSpherePositons_[collisionIndex];
    }

    matrix3_t PinocchioCollisionInterface::getRotationTimesVectorGradient(
      const vector3_t& eulerAnglesZYX, const vector3_t& vector) const
    {
      const ocs2::vector_t x = (ocs2::vector_t(3) << eulerAnglesZYX).finished();
      const ocs2::vector_t p = (ocs2::vector_t(3) << vector).finished();
      const ocs2::matrix_t dynamicsJacobian = rotationMatrixVectorAdInterfacePtr_->getJacobian(x, p);
      const matrix3_t approx = matrix3_t::Map(dynamicsJacobian.data());

      return approx;
    }

    ocs2::ad_vector_t PinocchioCollisionInterface::getRotationTimesVectorCppAd(
      const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& eulerAnglesAD, 
      const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1>& vector)
    {
      const Eigen::Matrix<ocs2::ad_scalar_t, 3, 3> rotationMatrix = getRotationMatrixFromZyxEulerAngles(
        eulerAnglesAD);
      return rotationMatrix * vector;
    }

    const pinocchio::GeometryModel& PinocchioCollisionInterface::getGeometryModel() const
    {
      return geometryModel_;
    }

    const std::vector<size_t>& PinocchioCollisionInterface::getTerrainAvoidanceCollisionIndices() const
    {
      return terrainAvoidanceIndices_;
    }

    const std::vector<std::pair<size_t, size_t>>& PinocchioCollisionInterface::getSelfCollisionIndices() const
    {
      return selfCollisionIndices_;
    }
  } // namespace collision
} // namespace legged_locomotion_mpc
