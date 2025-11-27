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
    const std::vector<scalar_t>& relaxations,
    RelaxedBarrierPenalty::Config settings):
      StateCost(),
      threeDofEndEffectorNum_(info.numThreeDofContacts),
      sixDofEndEffectorNum_(info.numSixDofContacts),
      endEffectorNum_(info.numThreeDofContacts + info.numSixDofContacts),
      referenceManager_(referenceManager),
      collisionInterface_(collisionInterface),
      selfAvoidancePenaltyPtr_(new RelaxedBarrierPenalty(settings)) 
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
            const size_t threeEndEffectorIndex = collisionPair.second;
            const size_t sixEndEffectorIndex = collisionPair.first;
            endEffector36DoFPairIndices_.emplace_back(
              threeEndEffectorIndex, sixEndEffectorIndex);
            endEffector36DoFPairRelaxations_.push_back(relaxations[relaxationIndex]);
          }
          else
          {
            endEffector66DoFPairIndices_.push_back(collisionPair);
            endEffector66DoFPairRelaxations_.push_back(relaxations[relaxationIndex]);
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
    for(const auto& endEffectorPair: endEffector33DoFPairIndices_)
    {
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
    for(const auto& endEffectorPair: endEffector36DoFPairIndices_)
    {
      const size_t firstIndex = endEffectorPair.first;
      const size_t secondIndex = endEffectorPair.second;

      const vector3_t& firstFramePosition = leggedPrecomputation.getEndEffectorPosition(
        firstIndex);
      const vector3_t& secondFramePosition = leggedPrecomputation.getEndEffectorPosition(
        secondIndex);

      const vector3_t& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        secondIndex);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);

      const std::vector<vector3_t>& sphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(secondIndex);

      const scalar_t firstRadius = 
        collisionInterface_.getFrameSphereRadiuses(firstIndex)[0];
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
    for(const auto& endEffectorPair: endEffector66DoFPairIndices_)
    {
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

      const std::vector<vector3_t>& firstSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(firstIndex);

      const std::vector<vector3_t>& secondSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(secondIndex);

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
    for(const auto& endEffectorLinkPair: endEffector3DoFLinkIndices_)
    {
      const size_t endEffectorIndex = endEffectorLinkPair.first;
      const size_t collisionIndex = endEffectorLinkPair.second;

      const vector3_t& framePosition = leggedPrecomputation.getEndEffectorPosition(
        endEffectorIndex);

      const vector3_t& collisionPosition = leggedPrecomputation.getCollisionLinkPosition(
        collisionIndex);
      const vector3_t& collisionEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        collisionIndex);
      const matrix3_t collisionRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        collisionEulerAngles);

      const std::vector<vector3_t>& sphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(collisionIndex);

      const scalar_t frameRadius = 
        collisionInterface_.getFrameSphereRadiuses(endEffectorIndex)[0];

      const std::vector<scalar_t>& collisionRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(collisionIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();

      for(size_t j = 0; j < collisionRadiuses.size(); ++j)
      {
        const vector3_t spherePosition = collisionPosition 
          + collisionRotationMatrix * sphereRelativePositions[j];

        const scalar_t distance = (spherePosition - framePosition).norm() 
          - collisionRadiuses[j];
        if(distance < minDistance) minDistance = distance;
      }
      cost += selfAvoidancePenaltyPtr_->getValue(0.0, minDistance - frameRadius);
    }

    // 6 DoF - Collision Link
    for(const auto& endEffectorLinkPair: endEffector6DoFLinkIndices_)
    {
      const size_t frameIndex = endEffectorLinkPair.first;
      const size_t collisionIndex = endEffectorLinkPair.second;

      const vector3_t& framePosition = leggedPrecomputation.getEndEffectorPosition(
        frameIndex);
      const vector3_t& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        frameIndex);
      const matrix3_t frameRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        frameEulerAngles);

      const vector3_t& collisionPosition = leggedPrecomputation.getCollisionLinkPosition(
        collisionIndex);
      const vector3_t& collisionFrameEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        collisionIndex);
      const matrix3_t collisionRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        collisionFrameEulerAngles);

      const std::vector<vector3_t>& frameSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(frameIndex);
      
      const std::vector<vector3_t>& collisionSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(collisionIndex);

      const std::vector<scalar_t>& frameRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(frameIndex);

      const std::vector<scalar_t>& collisionRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(collisionIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();

      for(size_t j = 0; j < frameRadiuses.size(); ++j)
      {
        const vector3_t frameSpherePosition = framePosition 
          + frameRotationMatrix * frameSphereRelativePositions[j];
        for(size_t k = 0; k < collisionRadiuses.size(); ++k)
        {
          const vector3_t collisionSpherePosition = collisionPosition 
            + collisionRotationMatrix * collisionSphereRelativePositions[k];
          const scalar_t distance = (frameSpherePosition - collisionSpherePosition).norm() 
            - frameRadiuses[j] - collisionRadiuses[k];
          if(distance < minDistance) minDistance = distance;
        }
      }
      cost += selfAvoidancePenaltyPtr_->getValue(0.0, minDistance);
    }

    // Collision Link - Collision Link
    for(const auto& collisionPair: collisionLinkPairIndices_)
    {
      const size_t firstIndex = collisionPair.first;
      const size_t secondIndex = collisionPair.second;

      const vector3_t& firstPosition = leggedPrecomputation.getCollisionLinkPosition(
        firstIndex);
      const vector3_t& firstEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        firstIndex);
      const matrix3_t firstRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        firstEulerAngles);

      const vector3_t& secondPosition = leggedPrecomputation.getCollisionLinkPosition(
        secondIndex);
      const vector3_t& secondEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        secondIndex);
      const matrix3_t secondRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        secondEulerAngles);

      const std::vector<vector3_t>& firstSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(firstIndex);

      const std::vector<vector3_t>& secondSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(secondIndex);

      const std::vector<scalar_t>& firstRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(firstIndex);

      const std::vector<scalar_t>& secondRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(secondIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();

      for(size_t j = 0; j < firstRadiuses.size(); ++j)
      {
        const vector3_t firstSpherePosition = firstPosition 
          + firstRotationMatrix * firstSphereRelativePositions[j];
        for(size_t k = 0; k < secondRadiuses.size(); ++k)
        {
          const vector3_t secondSpherePosition = secondPosition 
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
    
    ScalarFunctionQuadraticApproximation cost;
    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(state.size());
    cost.dfdxx = vector_t::Zero(state.size(), state.size());

    // 3 DoF - 3 DoF
    for(const auto& endEffectorPair: endEffector33DoFPairIndices_)
    {
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

      const scalar_t pointDistance = (firstFramePosition - secondFramePosition).norm();

      const scalar_t distance = pointDistance - firstRadius - secondRadius;

      cost.f += selfAvoidancePenaltyPtr_->getValue(0.0, distance);

      const scalar_t penaltyDerivative = selfAvoidancePenaltyPtr_->getDerivative(0.0, distance);

      const auto& firstPositonDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(
        firstIndex);

      const auto& secondPositonDerivative = leggedPrecomputation.getEndEffectorPositionDerivatives(
        secondIndex);

      const vector3_t normal = (firstFramePosition - secondFramePosition).normalized();
      
      const matrix_t positionDifferenceGradient = firstPositonDerivative.dfdx - secondPositonDerivative.dfdx;
      
      const vector_t dDistancedx = positionDifferenceGradient.transpose() * normal;

      cost.dfdx += penaltyDerivative * dDistancedx;
      
      const scalar_t penaltySecondDerivative = selfAvoidancePenaltyPtr_->getSecondDerivative(0.0, 
        distance);

      const matrix3_t distanceSecondGradient = getDistanceSecondGradient(normal, pointDistance);

      cost.dfdxx += penaltySecondDerivative * dDistancedx * dDistancedx.transpose() 
        + penaltyDerivative * positionDifferenceGradient * distanceSecondGradient * 
          positionDifferenceGradient.transpose();
    }

    // 3 DoF - 6 DoF
    for(const auto& endEffectorPair: endEffector36DoFPairIndices_)
    {
      const size_t firstIndex = endEffectorPair.first;
      const size_t secondIndex = endEffectorPair.second;

      const vector3_t& firstFramePosition = leggedPrecomputation.getEndEffectorPosition(
        firstIndex);
      const vector3_t& secondFramePosition = leggedPrecomputation.getEndEffectorPosition(
        secondIndex);

      const vector3_t& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        secondIndex);
      const matrix3_t rotationMatrix = getRotationMatrixFromZyxEulerAngles(frameEulerAngles);

      const std::vector<vector3_t>& sphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(secondIndex);

      const scalar_t firstRadius = 
        collisionInterface_.getFrameSphereRadiuses(firstIndex)[0];
      const std::vector<scalar_t>& secondRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(secondIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      size_t minIndex = 0;

      for(size_t j = 0; j < secondRadiuses.size(); ++j)
      {
        const vector3_t spherePosition = secondFramePosition + rotationMatrix * sphereRelativePositions[j];
        const scalar_t distance = (spherePosition - firstFramePosition).norm() 
          - secondRadiuses[j];
        if(distance < minDistance)
        {
          minDistance = distance;
          minIndex = j;
        } 
      }

      const vector3_t minSpherePosition = secondFramePosition 
        + rotationMatrix * sphereRelativePositions[minIndex];
      const vector3_t pointDifference = firstFramePosition - minSpherePosition;
      const vector3_t normal = pointDifference.normalized();
      const scalar_t pointDistance = pointDifference.norm();
      const scalar_t sphereDistance = minDistance - firstRadius;

      cost.f += selfAvoidancePenaltyPtr_->getValue(0.0, sphereDistance);

      const auto& firstPositionDerivatives = leggedPrecomputation.getEndEffectorPositionDerivatives(
        firstIndex).dfdx;

      matrix_t secondPositionDerivatives = leggedPrecomputation.getEndEffectorPositionDerivatives(
        secondIndex).dfdx;
      const auto& secondOrientationDerivatives = leggedPrecomputation.getEndEffectorOrientationDerivatives(
        secondIndex).dfdx;
      const matrix3_t rotationVectorGradient = collisionInterface_.getRotationTimesVectorGradient(
        frameEulerAngles, sphereRelativePositions[minIndex]);
      secondPositionDerivatives += rotationVectorGradient * secondOrientationDerivatives;

      const scalar_t penaltyDerivative = selfAvoidancePenaltyPtr_->getDerivative(0.0, sphereDistance);

      const matrix_t positionDifferenceGradient = firstPositionDerivatives - secondPositionDerivatives;

      const vector_t dDistancedx = positionDifferenceGradient.transpose() * normal;

      cost.dfdx += penaltyDerivative * dDistancedx;
      
      const scalar_t penaltySecondDerivative = selfAvoidancePenaltyPtr_->getSecondDerivative(0.0, 
        sphereDistance);

      const matrix3_t distanceSecondGradient = getDistanceSecondGradient(normal, pointDistance);

      cost.dfdxx += penaltySecondDerivative * dDistancedx * dDistancedx.transpose() 
        + penaltyDerivative * positionDifferenceGradient * distanceSecondGradient * 
          positionDifferenceGradient.transpose();
    }

    // 6 DoF - 6 DoF
    for(const auto& endEffectorPair: endEffector66DoFPairIndices_)
    {
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

      const std::vector<vector3_t>& firstSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(firstIndex);

      const std::vector<vector3_t>& secondSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(secondIndex);

      const std::vector<scalar_t>& firstRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(firstIndex);

      const std::vector<scalar_t>& secondRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(secondIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      size_t minFirstIndex = 0;
      size_t minSecondIndex = 0;

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
          if(distance < minDistance)
          {
            minDistance = distance;
            minFirstIndex = j;
            minSecondIndex = k;
          }
        }
      }
      
      const vector3_t minFirstSpherePosition = firstFramePosition 
        + firstRotationMatrix * firstSphereRelativePositions[minFirstIndex];

      const vector3_t minSecondSpherePosition = secondFramePosition 
        + secondRotationMatrix * secondSphereRelativePositions[minSecondIndex];

      const vector3_t pointDifference = minFirstSpherePosition - minSecondSpherePosition;
      const vector3_t normal = pointDifference.normalized();
      const scalar_t pointDistance = pointDifference.norm();
      const scalar_t sphereDistance = minDistance;

      cost.f += selfAvoidancePenaltyPtr_->getValue(0.0, sphereDistance);

      matrix_t firstPositionDerivatives = leggedPrecomputation.getEndEffectorPositionDerivatives(
        firstIndex).dfdx;
      const auto& firstOrientationDerivatives = leggedPrecomputation.getEndEffectorOrientationDerivatives(
        firstIndex).dfdx;
      const matrix3_t firstRotationVectorGradient = collisionInterface_.getRotationTimesVectorGradient(
        firstFrameEulerAngles, firstSphereRelativePositions[minFirstIndex]);
      firstPositionDerivatives += firstRotationVectorGradient * firstOrientationDerivatives;

      matrix_t secondPositionDerivatives = leggedPrecomputation.getEndEffectorPositionDerivatives(
        secondIndex).dfdx;
      const auto& secondOrientationDerivatives = leggedPrecomputation.getEndEffectorOrientationDerivatives(
        secondIndex).dfdx;
      const matrix3_t secondRotationVectorGradient = collisionInterface_.getRotationTimesVectorGradient(
        secondFrameEulerAngles, secondSphereRelativePositions[minSecondIndex]);
      secondPositionDerivatives += secondRotationVectorGradient * secondOrientationDerivatives;

      const scalar_t penaltyDerivative = selfAvoidancePenaltyPtr_->getDerivative(0.0, sphereDistance);

      const matrix_t positionDifferenceGradient = firstPositionDerivatives - secondPositionDerivatives;

      const vector_t dDistancedx = positionDifferenceGradient.transpose() * normal;

      cost.dfdx += penaltyDerivative * dDistancedx;
      
      const scalar_t penaltySecondDerivative = selfAvoidancePenaltyPtr_->getSecondDerivative(0.0, 
        sphereDistance);

      const matrix3_t distanceSecondGradient = getDistanceSecondGradient(normal, pointDistance);

      cost.dfdxx += penaltySecondDerivative * dDistancedx * dDistancedx.transpose() 
        + penaltyDerivative * positionDifferenceGradient * distanceSecondGradient * 
          positionDifferenceGradient.transpose();
    }

    // 3 DoF - Collision Link
    for(const auto& endEffectorLinkPair: endEffector3DoFLinkIndices_)
    {
      const size_t endEffectorIndex = endEffectorLinkPair.first;
      const size_t collisionIndex = endEffectorLinkPair.second;

      const vector3_t& framePosition = leggedPrecomputation.getEndEffectorPosition(
        endEffectorIndex);

      const vector3_t& collisionPosition = leggedPrecomputation.getCollisionLinkPosition(
        collisionIndex);
      const vector3_t& collisionEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        collisionIndex);
      const matrix3_t collisionRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        collisionEulerAngles);

      const std::vector<vector3_t>& sphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(collisionIndex);

      const scalar_t frameRadius = 
        collisionInterface_.getFrameSphereRadiuses(endEffectorIndex)[0];

      const std::vector<scalar_t>& collisionRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(collisionIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      size_t minIndex = 0;

      for(size_t j = 0; j < collisionRadiuses.size(); ++j)
      {
        const vector3_t spherePosition = collisionPosition 
          + collisionRotationMatrix * sphereRelativePositions[j];

        const scalar_t distance = (spherePosition - framePosition).norm() 
          - collisionRadiuses[j];
        if(distance < minDistance)
        {
          minDistance = distance;
          minIndex = j;
        }
      }

      const vector3_t minSpherePosition = collisionPosition  
        + collisionRotationMatrix * sphereRelativePositions[minIndex];
      const vector3_t pointDifference = framePosition - minSpherePosition;
      const vector3_t normal = pointDifference.normalized();
      const scalar_t pointDistance = pointDifference.norm();
      const scalar_t sphereDistance = minDistance - frameRadius;

      cost.f += selfAvoidancePenaltyPtr_->getValue(0.0, sphereDistance);

      const auto& framePositionDerivatives = leggedPrecomputation.getEndEffectorPositionDerivatives(
        endEffectorIndex ).dfdx;

      matrix_t collisionPositionDerivatives = leggedPrecomputation.getCollisionLinkPositionDerivatives(
        collisionIndex).dfdx;
      const auto& collisionOrientationDerivatives = leggedPrecomputation.getCollisionLinkOrientationDerivatives(
        collisionIndex).dfdx;
      const matrix3_t rotationVectorGradient = collisionInterface_.getRotationTimesVectorGradient(
        collisionEulerAngles, sphereRelativePositions[minIndex]);
      collisionPositionDerivatives += rotationVectorGradient * collisionOrientationDerivatives;

      const scalar_t penaltyDerivative = selfAvoidancePenaltyPtr_->getDerivative(0.0, sphereDistance);

      const matrix_t positionDifferenceGradient = framePositionDerivatives - secondPositionDerivatives;

      const vector_t dDistancedx = positionDifferenceGradient.transpose() * normal;

      cost.dfdx += penaltyDerivative * dDistancedx;
      
      const scalar_t penaltySecondDerivative = selfAvoidancePenaltyPtr_->getSecondDerivative(0.0, 
        sphereDistance);

      const matrix3_t distanceSecondGradient = getDistanceSecondGradient(normal, pointDistance);

      cost.dfdxx += penaltySecondDerivative * dDistancedx * dDistancedx.transpose() 
        + penaltyDerivative * positionDifferenceGradient * distanceSecondGradient * 
          positionDifferenceGradient.transpose();
    }

    // 6 DoF - Collision Link
    for(const auto& endEffectorLinkPair: endEffector6DoFLinkIndices_)
    {
      const size_t frameIndex = endEffectorPair.first;
      const size_t collisionIndex = endEffectorPair.second;

      const vector3_t& framePosition = leggedPrecomputation.getEndEffectorPosition(
        frameIndex);
      const vector3_t& frameEulerAngles = leggedPrecomputation.getEndEffectorOrientation(
        frameIndex);
      const matrix3_t frameRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        frameEulerAngles);

      const vector3_t& collisionPosition = leggedPrecomputation.getCollisionLinkPosition(
        collisionIndex);
      const vector3_t& collisionFrameEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        collisionIndex);
      const matrix3_t collisionRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        collisionFrameEulerAngles);

      const std::vector<vector3_t>& frameSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(frameIndex);
      
      const std::vector<vector3_t>& collisionSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(collisionIndex)

      const std::vector<scalar_t>& frameRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(frameIndex);

      const std::vector<scalar_t>& collisionRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(collisionIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      size_t minFrameIndex = 0;
      size_t minCollisionIndex = 0;

      for(size_t j = 0; j < frameRadiuses.size(); ++j)
      {
        const vector3_t frameSpherePosition = framePosition 
          + frameRotationMatrix * frameSphereRelativePositions[j];
        for(size_t k = 0; k < collisionRadiuses.size(); ++k)
        {
          const vector3_t collisionSpherePosition = collisionPosition 
            + collisionRotationMatrix * collisionSphereRelativePositions[k];
          const scalar_t distance = (frameSpherePosition - collisionSpherePosition).norm() 
            - frameRadiuses[j] - collisionRadiuses[k];
          if(distance < minDistance)
          {
            minDistance = distance;
            minFrameIndex = j;
            minCollisionIndex = k;
          }
        }
      }
      const vector3_t minFrameSpherePosition = framePosition 
        + frameRotationMatrix * frameSphereRelativePositions[minFrameIndex];

      const vector3_t minCollisionSpherePosition = collisionFramePosition 
        + collisionRotationMatrix * collisionSphereRelativePositions[minCollisionIndex];

      const vector3_t pointDifference = minFrameSpherePosition - minCollisionSpherePosition;
      const vector3_t normal = pointDifference.normalized();
      const scalar_t pointDistance = pointDifference.norm();
      const scalar_t sphereDistance = minDistance;

      cost.f += selfAvoidancePenaltyPtr_->getValue(0.0, sphereDistance);

      matrix_t framePositionDerivatives = leggedPrecomputation.getEndEffectorPositionDerivatives(
        frameIndex).dfdx;
      const auto& frameOrientationDerivatives = leggedPrecomputation.getEndEffectorOrientationDerivatives(
        frameIndex).dfdx;
      const matrix3_t frameRotationVectorGradient = collisionInterface_.getRotationTimesVectorGradient(
        frameEulerAngles, frameSphereRelativePositions[minFrameIndex]);
      framePositionDerivatives += frameRotationVectorGradient * frameOrientationDerivatives;

      matrix_t collisionPositionDerivatives = leggedPrecomputation.getEndEffectorPositionDerivatives(
        collisionIndex).dfdx;
      const auto& collisionOrientationDerivatives = leggedPrecomputation.getEndEffectorOrientationDerivatives(
        collisionIndex).dfdx;
      const matrix3_t collisionRotationVectorGradient = collisionInterface_.getRotationTimesVectorGradient(
        collisionEulerAngles, collisionSphereRelativePositions[minCollisionIndex]);
      collisionPositionDerivatives += collisionRotationVectorGradient * collisionOrientationDerivatives;

      const scalar_t penaltyDerivative = selfAvoidancePenaltyPtr_->getDerivative(0.0, sphereDistance);

      const matrix_t positionDifferenceGradient = framePositionDerivatives - collisionPositionDerivatives;

      const vector_t dDistancedx = positionDifferenceGradient.transpose() * normal;

      cost.dfdx += penaltyDerivative * dDistancedx;
      
      const scalar_t penaltySecondDerivative = selfAvoidancePenaltyPtr_->getSecondDerivative(0.0, 
        sphereDistance);

      const matrix3_t distanceSecondGradient = getDistanceSecondGradient(normal, pointDistance);

      cost.dfdxx += penaltySecondDerivative * dDistancedx * dDistancedx.transpose() 
        + penaltyDerivative * positionDifferenceGradient * distanceSecondGradient * 
          positionDifferenceGradient.transpose();
    }

    // Collision Link - Collision Link
    for(const auto& collisionPair: collisionLinkPairIndices_.)
    {
      const size_t firstIndex = collisionPair.first;
      const size_t secondIndex = collisionPair.second;

      const vector3_t& firstPosition = leggedPrecomputation.getCollisionLinkPosition(
        firstIndex);
      const vector3_t& firstEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        firstIndex);
      const matrix3_t firstRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        firstEulerAngles);

      const vector3_t& secondPosition = leggedPrecomputation.getCollisionLinkPosition(
        secondIndex);
      const vector3_t& secondEulerAngles = leggedPrecomputation.getCollisionLinkOrientation(
        secondIndex);
      const matrix3_t secondRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        secondEulerAngles);

      const std::vector<vector3_t>& firstSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(firstIndex);

      const std::vector<vector3_t>& secondSphereRelativePositions = 
        collisionInterface_.getFrameSpherePositions(secondIndex)

      const std::vector<scalar_t>& firstRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(firstIndex);

      const std::vector<scalar_t>& secondRadiuses = 
        collisionInterface_.getFrameSphereRadiuses(secondIndex);

      scalar_t minDistance = std::numeric_limits<scalar_t>::max();
      size_t minFirstIndex = 0;
      size_t minSecondIndex = 0;

      for(size_t j = 0; j < firstRadiuses.size(); ++j)
      {
        const vector3_t firstSpherePosition = firstPosition 
          + firstRotationMatrix * firstSphereRelativePositions[j];
        for(size_t k = 0; k < secondRadiuses.size(); ++k)
        {
          const vector3_t secondSpherePosition = secondPosition 
            + secondRotationMatrix * secondSphereRelativePositions[k];
          const scalar_t distance = (firstSpherePosition - secondSpherePosition).norm() 
            - firstRadiuses[j] - secondRadiuses[k];
          if(distance < minDistance)
          {
            minDistance = distance;
            minFirstIndex = j; 
            minSecondIndex = k;
          }
        }
      }
      
      const vector3_t minFirstSpherePosition = firstPosition 
        + firstRotationMatrix * firstSphereRelativePositions[minFirstIndex];

      const vector3_t minSecondSpherePosition = secondPosition 
        + secondRotationMatrix * secondSphereRelativePositions[minSecondIndex];

      const vector3_t pointDifference = minFirstSpherePosition - minSecondSpherePosition;
      const vector3_t normal = pointDifference.normalized();
      const scalar_t pointDistance = pointDifference.norm();
      const scalar_t sphereDistance = minDistance;

      cost.f += selfAvoidancePenaltyPtr_->getValue(0.0, sphereDistance);

      matrix_t firstPositionDerivatives = leggedPrecomputation.getCollisionLinkPositionDerivatives(
        firstIndex).dfdx;
      const auto& firstOrientationDerivatives = leggedPrecomputation.getCollisionLinkOrientationDerivatives(
        firstIndex).dfdx;
      const matrix3_t firstRotationVectorGradient = collisionInterface_.getRotationTimesVectorGradient(
        firstFrameEulerAngles, firstSphereRelativePositions[minFirstIndex]);
      firstPositionDerivatives += firstRotationVectorGradient * firstOrientationDerivatives;

      mmatrix_t secondPositionDerivatives = leggedPrecomputation.getCollisionLinkPositionDerivatives(
        secondIndex).dfdx;
      const auto& secondOrientationDerivatives = leggedPrecomputation.getCollisionLinkOrientationDerivatives(
        secondIndex).dfdx;
      const matrix3_t secondRotationVectorGradient = collisionInterface_.getRotationTimesVectorGradient(
        secondFrameEulerAngles, secondSphereRelativePositions[minSecondIndex]);
      secondPositionDerivatives += secondRotationVectorGradient * secondOrientationDerivatives;

      const scalar_t penaltyDerivative = selfAvoidancePenaltyPtr_->getDerivative(0.0, sphereDistance);

      const matrix_t positionDifferenceGradient = firstPositionDerivatives - secondPositionDerivatives;

      const vector_t dDistancedx = positionDifferenceGradient.transpose() * normal;

      cost.dfdx += penaltyDerivative * dDistancedx;
      
      const scalar_t penaltySecondDerivative = selfAvoidancePenaltyPtr_->getSecondDerivative(0.0, 
        sphereDistance);

      const matrix3_t distanceSecondGradient = getDistanceSecondGradient(normal, pointDistance);

      cost.dfdxx += penaltySecondDerivative * dDistancedx * dDistancedx.transpose() 
        + penaltyDerivative * positionDifferenceGradient * distanceSecondGradient * 
          positionDifferenceGradient.transpose();
    }
    return cost;
  }

  matrix3_t SelfCollisionAvoidanceSoftConstraint::getDistanceSecondGradient(
    const vector3_t& normal, scalar_t distance) const
  {
    matrix3_t temp = - normal * normal.transpose() / distance;
    temp.diagonal().array() += distance * distance;
    return temp;
  }

  SelfCollisionAvoidanceSoftConstraint::SelfCollisionAvoidanceSoftConstraint(
    const SelfCollisionAvoidanceSoftConstraint& rhs):
      StateCost(),
      threeDofEndEffectorNum_(rhs.threeDofEndEffectorNum_),
      sixDofEndEffectorNum_(rhs.sixDofEndEffectorNum_),
      endEffectorNum_(rhs.endEffectorNum_),
      endEffector33DoFPairIndices_(rhs.endEffector33DoFPairIndices_),
      endEffector33DoFPairRelaxations_(rhs.endEffector33DoFPairRelaxations_),
      endEffector36DoFPairIndices_(rhs.endEffector36DoFPairIndices_),
      endEffector36DoFPairRelaxations_(rhs.endEffector36DoFPairRelaxations_),
      endEffector66DoFPairIndices_(rhs.endEffector66DoFPairIndices_),
      endEffector66DoFPairRelaxations_(rhs.endEffector66DoFPairRelaxations_),
      endEffector3DoFLinkIndices_(rhs.endEffector3DoFLinkIndices_),
      endEffector3DoFLinkRelaxations_(rhs.endEffector3DoFLinkRelaxations_),
      endEffector6DoFLinkIndices_(rhs.endEffector6DoFLinkIndices_),
      endEffector6DoFLinkRelaxations_(rhs.endEffector6DoFLinkRelaxations_),
      collisionLinkPairIndices_(rhs.collisionLinkPairIndices_),
      collisionLinkPairRelaxations_(rhs.collisionLinkPairRelaxations_),
      referenceManager_(rhs.referenceManager_),
      collisionInterface_(rhs.collisionInterface_),
      relaxations_(rhs.relaxations_),
      selfAvoidancePenaltyPtr_(rhs.selfAvoidancePenaltyPtr_->clone()) {}
} // namespace legged_locomotion_mpc