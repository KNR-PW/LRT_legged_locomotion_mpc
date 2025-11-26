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

#include <legged_locomotion_mpc/soft_constraint/SelfCollisionAvoidanceSoftConstraint.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  
  using namespace ocs2;
  using namespace floating_base_model;
  using namespace collision;
  using namespace terrain_model;

  SelfCollisionAvoidanceSoftConstraint::SelfCollisionAvoidanceSoftConstraint(
    FloatingBaseModelInfo info,
    const PinocchioCollisionInterface& collisionInterface,
    const LeggedReferenceManager& referenceManager,
    const std::vector<std::pair<size_t, size_t>>& collisionIndices,
    std::vector<scalar_t> relaxations,
    RelaxedBarrierPenalty::Config settings):
      threeDofEndEffectorNum_(info.numThreeDofContacts),
      sixDofEndEffectorNum_(info.numSixDofContacts),
      endEffectorNum_(info.numThreeDofContacts + info.numSixDofContacts),
      referenceManager_(referenceManager),
      collisionInterface_(collisionInterface),
      terrainAvoidancePenaltyPtr_(new RelaxedBarrierPenalty(settings)) 
  {
    size_t relaxationIndex = 0;
    for(const auto& collisionPair: collisionIndices)
    {
      // If there is at least one end effector
      if((collisionPair.first < endEffectorNum_) || (collisionPair.second < endEffectorNum_))
      {
        // Two end effectors
        if((collisionPair.first < endEffectorNum_) && (collisionPair.second < endEffectorNum_))
        {
          // Check number of DoFs
          bool isFirst3DoF = collisionPair.first < threeDofEndEffectorNum_;
          bool isSecond3DoF = collisionPair.second < threeDofEndEffectorNum_;
          if(isFirst3DoF && isSecond3DoF)
          {
            endEffector33DoFPairIndices_.push_back(collisionPair);
            endEffector33DoFPairRelaxations_.push_back(relaxations[relaxationIndex]);
          }
          else if(isFirst3DoF)
          {
            endEffector36DoFPairIndices_.push_back(collisionPair);
            endEffector36DoFPairRelaxations_.push_back(relaxations[relaxationIndex]);
          }
          else if(isSecond3DoF)
          {
            size_t temp = collisionPair.second;
            collisionPair.second = collisionPair.first;
            collisionPair.first = temp;
            endEffector36DoFPairIndices_.push_back(collisionPair);
            endEffector36DoFPairRelaxations_.push_back(relaxations[relaxationIndex]);
          }
          else
          {
            ndEffector66DoFPairIndices_.push_back(collisionPair);
            endEffecto636DoFPairRelaxations_.push_back(relaxations[relaxationIndex]);
          }
        }
        // One is end effector, it will be first item in pair
        else
        {
          size_t endEffectorIndex = 0;
          size_t collisionLinkIndex = 0;
          if(collisionPair.first < collisionPair.second)
          {
            endEffectorIndex = collisionPair.first;
            collisionLinkIndex = collisionPair.second;

          }
          else
          {
            endEffectorIndex = collisionPair.second;
            collisionLinkIndex = collisionPair.first;
          }

          bool is3DoF = endEffectorIndex < threeDofEndEffectorNum_;
          if(is3DoF)
          {
            endEffector3DoFLinkIndices_.emplace_back(endEffectorIndex, collisionLinkIndex);
            endEffector3DoFLinkRelaxations_.push_back(relaxations[relaxationIndex]);
          }
          else
          {
            endEffector6DoFLinkIndices_.emplace_back(endEffectorIndex, collisionLinkIndex);
            endEffector6DoFLinkRelaxations_.push_back(relaxations[relaxationIndex]);
          }
        }
      }
      // Two collision links
      else
      {
        collisionLinkPairIndices_.push_back(collisionPair);
        collisionLinkPairRelaxations_.push_back(relaxations[relaxationIndex]);
      }
      relaxationIndex++;
    }
  }

  SelfCollisionAvoidanceSoftConstraint* SelfCollisionAvoidanceSoftConstraint::clone() const
  {
    return new SelfCollisionAvoidanceSoftConstraint(*this);
  }

  scalar_t SelfCollisionAvoidanceSoftConstraint::getValue(
    scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories, 
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    scalar_t cost = 0.0;

    // 3 DoF - 3 DoF
    for(size_t i = 0; i < endEffector33DoFPairIndices_.size(); ++i)
    {
      const auto& endEffectorPair = endEffector33DoFPairIndices_[i];
      const size_t firstIndex = endEffectorPair.first;
      const size_t secondIndex = endEffectorPair.second;

      const vector3_t& firstFramePosition = leggedPrecomputation.getEndEffectorPosition(
        firstIndex);
      const vector3_t& secondFramePosition = leggedPrecomputation.getEndEffectorPosition(
        secondIndex);

      const scalar_t firstRadius = 
        collisionInterface_.getFrameSphereRadiuses(firstIndex)[0];
      const scalar_t  secondRadius = 
        collisionInterface_.getFrameSphereRadiuses(secondIndex)[0];

      const scalar_t distance = (firstFramePosition - secondFramePosition).norm() 
        - firstRadius - secondRadius;

      cost += selfAvoidancePenaltyPtr_->getValue(0.0, distance);
    }

    // 3 DoF - 6 DoF
    for(size_t i = 0; i < endEffector36DoFPairIndices_.size(); ++i)
    {
      const auto& endEffectorPair = endEffector36DoFPairIndices_[i];
      const size_t firstIndex = endEffectorPair.first;
      const size_t secondIndex = endEffectorPair.second;

      const vector3_t& firstFramePosition = leggedPrecomputation.getEndEffectorPosition(
        firstIndex);
      const vector3_t& secondFramePosition = leggedPrecomputation.getEndEffectorPosition(
        secondIndex);

      const vector3_t& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        secondIndex);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);

      const std::vector<vector3_t>& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(secondIndex);

      const scalar_t firstRadius = 
        collisionInterface_.getFrameSphereRadiuses(firstIndex);
      const std::vector<scalar_t>& secondRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(secondIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();

      for(size_t j = 0; j < secondRadiuses.size(); ++j)
      {
        const vector3_t spherePosition = secondFramePosition + rotationMatrix * sphereRelativePositions[j];
        const scalar_t distance = (spherePosition - firstFramePosition).norm() 
          - secondRadiuses[j];
        if(distance < minDistance) minDistance = distance;
      }
      cost += selfAvoidancePenaltyPtr_->getValue(0.0, minDistance - firstRadius);
    }

    // 6 DoF - 6 DoF
    for(size_t i = 0; i < endEffector66DoFPairIndices_.size(); ++i)
    {
      const auto& endEffectorPair = endEffector66DoFPairIndices_[i];
      const size_t firstIndex = endEffectorPair.first;
      const size_t secondIndex = endEffectorPair.second;

      const vector3_t& firstFramePosition = leggedPrecomputation.getEndEffectorPosition(
        firstIndex);

      const vector3_t& firstFrameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        firstIndex);
      const matrix3_t firstRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        firstFrameEulerAngles);

      const vector3_t& secondFramePosition = leggedPrecomputation.getEndEffectorPosition(
        secondIndex);

      const vector3_t& secondFrameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        secondIndex);
      const matrix3_t secondRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        secondFrameEulerAngles);

      const std::vector<vector3_t>& firstSphereRelativePositions = collisionInterface_.getFrameSpherePositions(firstIndex);
      const std::vector<vector3_t>& secondSphereRelativePositions = collisionInterface_.getFrameSpherePositions(secondIndex)

      const std::vector<scalar_t>& firstRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(firstIndex);
      const std::vector<scalar_t>& secondRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(secondIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();

      for(size_t j = 0; j < firstRadiuses.size(); ++j)
      {
        const vector3_t firstSpherePosition = firstFramePosition 
          + firstRotationMatrix * firstSphereRelativePositions[j];
        for(size_t k = 0; k < secondRadiuses.size(); ++k)
        {
          const vector3_t secondSpherePosition = secondFramePosition 
            + secondRotationMatrix * secondSphereRelativePositions[k];
          const scalar_t distance = (firstSpherePosition - secondSpherePosition).norm() 
            - firstRadiuses[j] - secondRadiuses[k];
          if(distance < minDistance) minDistance = distance;
        }
      }
      cost += selfAvoidancePenaltyPtr_->getValue(0.0, minDistance);
    }

    // 3 DoF - Collision Link
    for(size_t i = 0; i < endEffector3DoFLinkIndices_.size(); ++i)
    {
      const auto& endEffectorLinkPair = endEffector3DoFLinkIndices_[i];
      const size_t endEffectorIndex = endEffectorLinkPair.first;
      const size_t collisionIndex = endEffectorLinkPair.second;

      const vector3_t& framePosition = leggedPrecomputation.getEndEffectorPosition(
        endEffectorIndex);
      const vector3_t& collisionFramePosition = leggedPrecomputation.getCollisionLinkPosition(
        collisionIndex);

      const vector3_t& frameEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        endEffectorIndex);
      const matrix3_t frameRotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);

      const vector3_t& collisionEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        collisionIndex);
      const matrix3_t collisionRotationMatrix = getRotationMatrixFromZyxEulerAngles(collisionEulerAngles);

      const std::vector<vector3_t>& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(collisionIndex);

      const scalar_t firstRadius = 
        collisionInterface_.getFrameSphereRadiuses(endEffectorIndex);
      const std::vector<scalar_t>& secondRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(collisionIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();

      for(size_t j = 0; j < secondRadiuses.size(); ++j)
      {
        const vector3_t spherePosition = collisionFramePosition + rotationMatrix * sphereRelativePositions[j];
        const scalar_t distance = (spherePosition - framePosition).norm() 
          - secondRadiuses[j];
        if(distance < minDistance) minDistance = distance;
      }
      cost += selfAvoidancePenaltyPtr_->getValue(0.0, minDistance - firstRadius);
    }

    // 6 DoF - Collision Link
    for(size_t i = 0; i < endEffector6DoFLinkIndices_.size(); ++i)
    {
      const auto& endEffectorLinkPair = endEffector6DoFLinkIndices_[i];
      const size_t frameIndex = endEffectorPair.first;
      const size_t collisionIndex = endEffectorPair.second;

      const vector3_t& framePosition = leggedPrecomputation.getEndEffectorPosition(
        frameIndex);

      const vector3_t& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        frameIndex);
      const matrix3_t frameRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        frameEulerAngles);

      const vector3_t& secondFramePosition = leggedPrecomputation.getCollisionLinkPosition(
        collisionIndex);

      const vector3_t& collisionFrameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        collisionIndex);
      const matrix3_t collisionRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        collisionFrameEulerAngles);

      const std::vector<vector3_t>& frameSphereRelativePositions = collisionInterface_.getFrameSpherePositions(frameIndex);
      const std::vector<vector3_t>& collisionSphereRelativePositions = collisionInterface_.getFrameSpherePositions(collisionIndex)

      const std::vector<scalar_t>& frameRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(frameIndex);
      const std::vector<scalar_t>& collisionRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(collisionIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();

      for(size_t j = 0; j < frameRadiuses.size(); ++j)
      {
        const vector3_t frameSpherePosition = firstFramePosition 
          + frameRotationMatrix * frameSphereRelativePositions[j];
        for(size_t k = 0; k < collisionRadiuses.size(); ++k)
        {
          const vector3_t collisionSpherePosition = collisionFramePosition 
            + collisionRotationMatrix * collisionSphereRelativePositions[k];
          const scalar_t distance = (frameSpherePosition - collisionSpherePosition).norm() 
            - frameRadiuses[j] - collisionRadiuses[k];
          if(distance < minDistance) minDistance = distance;
        }
      }
      cost += selfAvoidancePenaltyPtr_->getValue(0.0, minDistance);
    }

    // Collision Link - Collision Link
    for(size_t i = 0; i < collisionLinkPairIndices_.size(); ++i)
    {
      const auto& collisionPair = collisionLinkPairIndices_[i];
      const size_t firstIndex = collisionPair.first;
      const size_t secondIndex = collisionPair.second;

      const vector3_t& firstFramePosition = leggedPrecomputation.getCollisionLinkPosition(
        firstIndex);

      const vector3_t& firstFrameEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        firstIndex);
      const matrix3_t firstRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        firstFrameEulerAngles);

      const vector3_t& secondFramePosition = leggedPrecomputation.getCollisionLinkPosition(
        secondIndex);

      const vector3_t& secondFrameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        secondIndex);
      const matrix3_t secondRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        secondFrameEulerAngles);

      const std::vector<vector3_t>& firstSphereRelativePositions = collisionInterface_.getFrameSpherePositions(firstIndex);
      const std::vector<vector3_t>& secondSphereRelativePositions = collisionInterface_.getFrameSpherePositions(secondIndex)

      const std::vector<scalar_t>& firstRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(firstIndex);
      const std::vector<scalar_t>& secondRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(secondIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();

      for(size_t j = 0; j < firstRadiuses.size(); ++j)
      {
        const vector3_t firstSpherePosition = firstFramePosition 
          + firstRotationMatrix * firstSphereRelativePositions[j];
        for(size_t k = 0; k < secondRadiuses.size(); ++k)
        {
          const vector3_t secondSpherePosition = secondFramePosition 
            + secondRotationMatrix * secondSphereRelativePositions[k];
          const scalar_t distance = (firstSpherePosition - secondSpherePosition).norm() 
            - firstRadiuses[j] - secondRadiuses[k];
          if(distance < minDistance) minDistance = distance;
        }
      }
      cost += selfAvoidancePenaltyPtr_->getValue(0.0, minDistance);
    }

    return cost;
  }

      
  ScalarFunctionQuadraticApproximation SelfCollisionAvoidanceSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const TargetTrajectories& targetTrajectories,
    const PreComputation& preComp) const
  {
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);
    const SignedDistanceField* sdf = referenceManager_.getTerrainModel()
      .getSignedDistanceField();

    ScalarFunctionQuadraticApproximation cost;
    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(state.size());
    cost.dfdxx = vector_t::Zero(state.size(), state.size());

    // Three Dof End Effectors (one sphere)
    for(size_t i = 0; i < threeDofEndEffectorNum_; ++i)
    {
      const scalar_t radius = collisionInterface_.getFrameSphereRadiuses(i)[0];
      const vector3_t& position = leggedPrecomputation.getEndEffectorPosition(i);
      const scalar_t terrainClearance = leggedPrecomputation.getReferenceEndEffectorTerrainClearance(i);
      const scalar_t relaxation = relaxations_[i];
      const auto [sdfDistance, sdfGradient] = sdf->valueAndDerivative(position);
      const scalar_t distance = sdfDistance - radius - terrainClearance + relaxation;
      cost.f += terrainAvoidancePenaltyPtr_->getValue(0.0, distance);

      const scalar_t penaltyDerivative = terrainAvoidancePenaltyPtr_->getDerivative(
        0.0, distance);

      const scalar_t penaltySecondDerivative = terrainAvoidancePenaltyPtr_->getSecondDerivative(
        0.0, distance);

      const auto& positionDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(i);
      
      const vector_t scaledGrdaient = positionDerivative.dfdx.transpose() * sdfGradient;
      
      cost.dfdx.noalias() += penaltyDerivative * scaledGrdaient;

      // Approximated second derivative (sdf and postion second gradients are omitted)
      cost.dfdxx.noalias() += penaltySecondDerivative * scaledGrdaient * scaledGrdaient.transpose();
    }

    // Six Dof End Effectors (many spheres)
    for(size_t i = threeDofEndEffectorNum_; i < endEffectorNum_; ++i)
    {
      const std::vector<scalar_t>& radiuses = collisionInterface_.getFrameSphereRadiuses(i);
      const std::vector<vector3_t>& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(i);
      const vector3_t& framePosition = leggedPrecomputation.getEndEffectorPosition(i);
      const vector3_t& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(i);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      std::vector<scalar_t> distances(sphereRelativePositions.size());
      for(size_t j = 0; j < sphereRelativePositions.size(); ++j)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[j];
        distances[j] = sdf->value(spherePositionInWorld) - radiuses[j];
      }      

      const scalar_t relaxation = relaxations_[i];
      const auto minDistanceIterator = std::min_element(distances.begin(), distances.end());
      const size_t minDistanceIndex = std::distance(distances.begin(), minDistanceIterator);
      const scalar_t terrainClearance = leggedPrecomputation.getReferenceEndEffectorTerrainClearance(i);
      const scalar_t distance = distances[minDistanceIndex] - terrainClearance + relaxation;
      
      cost.f += terrainAvoidancePenaltyPtr_->getValue(0.0, distance);

      const vector3_t minSpherePosition = framePosition + rotationMatrix * sphereRelativePositions[minDistanceIndex];

      const vector3_t sdfGradient = sdf->derivative(minSpherePosition);

      const scalar_t penaltyDerivative = terrainAvoidancePenaltyPtr_->getDerivative(
        0.0, distance);

      const scalar_t penaltySecondDerivative = terrainAvoidancePenaltyPtr_->getSecondDerivative(
        0.0, distance);

      const auto& positionDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(i);
      
      const vector_t scaledGrdaient = positionDerivative.dfdx.transpose() * sdfGradient;
      
      cost.dfdx.noalias() += penaltyDerivative * scaledGrdaient;

      // Approximated second derivative (sdf and postion second gradients are omitted)
      cost.dfdxx.noalias() += penaltySecondDerivative * scaledGrdaient * scaledGrdaient.transpose();
    }

    // Collison links (one or many spheres)
    for(size_t i = 0; i < collisionLinkIndicies_.size(); ++i)
    {
      const size_t collisionIndex = collisionLinkIndicies_[i];
      const std::vector<scalar_t>& radiuses = collisionInterface_.getFrameSphereRadiuses(
        collisionIndex);
      const std::vector<vector3_t>& sphereRelativePositions = collisionInterface_.getFrameSpherePositions(collisionIndex);
      const vector3_t& framePosition = leggedPrecomputation.getCollisionLinkPosition(i);
      const vector3_t& frameEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(i);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);
      std::vector<scalar_t> distances(sphereRelativePositions.size());
      for(size_t j = 0; j < sphereRelativePositions.size(); ++j)
      {
        const vector3_t spherePositionInWorld = framePosition + rotationMatrix * sphereRelativePositions[j];
        distances[j] = sdf->value(spherePositionInWorld) - radiuses[j];
      }      

      const scalar_t relaxation = relaxations_[i];
      const auto minDistanceIterator = std::min_element(distances.begin(), distances.end());
      const size_t minDistanceIndex = std::distance(distances.begin(), minDistanceIterator);

      const scalar_t distance = distances[minDistanceIndex] + relaxation;
      
      cost.f += terrainAvoidancePenaltyPtr_->getValue(0.0, distance);

      const vector3_t minSpherePosition = framePosition + rotationMatrix * sphereRelativePositions[minDistanceIndex];

      const vector3_t sdfGradient = sdf->derivative(minSpherePosition);

      const scalar_t penaltyDerivative = terrainAvoidancePenaltyPtr_->getDerivative(
        0.0, distance);

      const scalar_t penaltySecondDerivative = terrainAvoidancePenaltyPtr_->getSecondDerivative(
        0.0, distance);

      const auto& positionDerivative = leggedPrecomputation.getCollisionLinkPositionDerivatives(collisionIndex);
      
      const vector_t scaledGrdaient = positionDerivative.dfdx.transpose() * sdfGradient;
      
      cost.dfdx.noalias() += penaltyDerivative * scaledGrdaient;

      // Approximated second derivative (sdf and postion second gradients are omitted)
      cost.dfdxx.noalias() += penaltySecondDerivative * scaledGrdaient * scaledGrdaient.transpose();
    }

    return cost;
  }

  SelfCollisionAvoidanceSoftConstraint::SelfCollisionAvoidanceSoftConstraint(
    const SelfCollisionAvoidanceSoftConstraint& rhs):
      threeDofEndEffectorNum_(rhs.threeDofEndEffectorNum_),
      sixDofEndEffectorNum_(rhs.sixDofEndEffectorNum_),
      endEffectorNum_(rhs.endEffectorNum_),
      endEffectorPairIndices_(rhs.ndEffectorIndices_),
      endEffectorLinkIndices_(rhs.endEffectorLinkIndices_),
      collisionLinkPairIndices_(rhs.collisionLinkPairIndices_),
      referenceManager_(rhs.referenceManager_),
      collisionInterface_(rhs.collisionInterface_),
      relaxations_(rhs.relaxations_),
      terrainAvoidancePenaltyPtr_(rhs.terrainAvoidancePenaltyPtr_->clone()) {}
} // namespace legged_locomotion_mpc