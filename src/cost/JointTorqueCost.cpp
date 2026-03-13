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

#include <legged_locomotion_mpc/cost/JointTorqueCost.hpp>

#include <stdexcept>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{
  namespace cost
  {
    using namespace ocs2;
    using namespace floating_base_model;

    JointTorqueCost::JointTorqueCost(FloatingBaseModelInfo info, 
      JointTorqueCost::JointTorqueWeights jointWeights):
        info_(std::move(info)), jointWeights_(std::move(jointWeights))
    {
      if(jointWeights_.weights.size() != info_.actuatedDofNum)
      {
        throw std::invalid_argument("[JointTorqueCost]: Wrong size for joint torque weights!");
      }
    }

    JointTorqueCost* JointTorqueCost::clone() const
    {
      return new JointTorqueCost(*this);
    }
        
    scalar_t JointTorqueCost::getValue(scalar_t time, const vector_t& state, 
      const vector_t& input, const TargetTrajectories& targetTrajectories, 
      const PreComputation& preComp) const
    {
      const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

      const auto& torqueApprox = leggedPrecomputation.getApproximatedJointTorques();

      const scalar_t cost = 0.5 * torqueApprox.transpose() 
        * jointWeights_.weights.asDiagonal() * torqueApprox;

      return cost;
    }
        
    ScalarFunctionQuadraticApproximation JointTorqueCost::getQuadraticApproximation(
      scalar_t time, const vector_t& state, const vector_t& input,
      const TargetTrajectories& targetTrajectories,
      const PreComputation& preComp) const
    {
      const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

      const auto& torqueApproxDerivative = leggedPrecomputation.getApproximatedJointTorquesDerivatives();

      const vector_t scaledTorque = jointWeights_.weights.asDiagonal() 
        * torqueApproxDerivative.f;

      const size_t forceSize = 3 * info_.numThreeDofContacts + 6 * info_.numSixDofContacts;

      const matrix_t dTorquedQ = torqueApproxDerivative.dfdx.block(0, 6, 
        info_.actuatedDofNum, info_.generalizedCoordinatesNum);
      const matrix_t dTorquedF = torqueApproxDerivative.dfdu.block(0, 0, 
        info_.actuatedDofNum, forceSize);
    
      ScalarFunctionQuadraticApproximation cost;

      cost.f = 0.5 * torqueApproxDerivative.f.transpose() * scaledTorque;

      cost.dfdx = vector_t::Zero(info_.stateDim);
      cost.dfdu = vector_t::Zero(info_.inputDim);
      cost.dfdxx = matrix_t::Zero(info_.stateDim, info_.stateDim);
      cost.dfdux = matrix_t::Zero(info_.inputDim, info_.stateDim);
      cost.dfduu = matrix_t::Zero(info_.inputDim, info_.inputDim);


      cost.dfdx.block(6, 0, info_.generalizedCoordinatesNum, 1).noalias() = 
        dTorquedQ.transpose() * scaledTorque;
      cost.dfdu.block(0, 0, forceSize, 1).noalias() =  dTorquedF.transpose() * scaledTorque;

      const matrix_t scaledDToqueQ = jointWeights_.weights.asDiagonal() * dTorquedQ;

      cost.dfdxx.block(6, 6, info_.generalizedCoordinatesNum, info_.generalizedCoordinatesNum).noalias() = 
        dTorquedQ.transpose() * scaledDToqueQ;
      cost.dfduu.block(0, 0, forceSize, forceSize).noalias() = 
        dTorquedF.transpose() * jointWeights_.weights.asDiagonal() * dTorquedF;
      cost.dfdux.block(0, 6, forceSize, info_.generalizedCoordinatesNum).noalias() = 
        dTorquedF.transpose() * scaledDToqueQ;

      return cost;
    }

    JointTorqueCost::JointTorqueCost(const JointTorqueCost& rhs):
      StateInputCost(), info_(rhs.info_), jointWeights_(rhs.jointWeights_) {}

    JointTorqueCost::JointTorqueWeights loadJointTorqueWeights(const std::string& filename,
      const FloatingBaseModelInfo& info, const pinocchio::Model& robotModel,
      const std::string& fieldName, bool verbose)
    {
      boost::property_tree::ptree pt;
      read_info(filename, pt);

      if(verbose) 
      {
        std::cerr << "\n #### Legged Locomotion MPC Joint Torque Weights:";
        std::cerr << "\n #### =============================================================================\n";
      }
      
      JointTorqueCost::JointTorqueWeights jointWeights;

      std::vector<std::string> jointNames = robotModel.names;

      // Remove "universe" and "root_joint" joints from joint names
      jointNames.erase(std::remove(jointNames.begin(), jointNames.end(), "universe"), jointNames.end());
      jointNames.erase(std::remove(jointNames.begin(), jointNames.end(), "root_joint"), jointNames.end()); 

      if(jointNames.size() != info.actuatedDofNum)
      {
        throw std::invalid_argument("[JointTorqueCost]: Wrong size of joint names vector!");
      }

      jointWeights.weights = Eigen::VectorXd::Zero(info.actuatedDofNum);

      for(size_t i = 0; i < info.actuatedDofNum; ++i)
      {
        const std::string& jointName = jointNames[i];
        
        ocs2::loadData::loadPtreeValue(pt, jointWeights.weights[i], fieldName + "." + jointName + ".torqueWeight", verbose);
        if(jointWeights.weights[i] < 0.0)
        {
          std::string message = "[JointTorqueCost]: Joint torque weights element for joint " + jointName + " smaller than 0!";
          throw std::invalid_argument(message);
        }
      }

      if(verbose) 
      {
        std::cerr << " #### =============================================================================" <<
        std::endl;
      }

      return jointWeights;
    }
  }
}