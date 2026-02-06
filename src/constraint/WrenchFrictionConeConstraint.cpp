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

#include <legged_locomotion_mpc/constraint/WrenchFrictionConeConstraint.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <legged_locomotion_mpc/precomputation/LeggedPrecomputation.hpp>

namespace legged_locomotion_mpc
{

  using namespace ocs2;
  using namespace floating_base_model;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WrenchFrictionConeConstraint::Config::Config(
    scalar_t footLengthX,
    scalar_t footLengthY,
    scalar_t frictionCoefficientParam):
      footHalfLengthX(footLengthX * 0.5),
      footHalfLengthY(footLengthY * 0.5),
      frictionCoefficient(frictionCoefficientParam)
  {
    assert(frictionCoefficient > 0.0);
    assert(footHalfLengthX > 0.0);
    assert(footHalfLengthY > 0.0);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WrenchFrictionConeConstraint::WrenchFrictionConeConstraint(
    const LeggedReferenceManager &referenceManager,
    Config config,
    FloatingBaseModelInfo info,
    size_t endEffectorIndex): 
      StateInputConstraint(ConstraintOrder::Linear),
      referenceManager_(referenceManager),
      config_(std::move(config)),
      info_(std::move(info)) ,
      endEffectorIndex_(endEffectorIndex)
  {
    coneConstraintMatrix_ = generateConeConstraintMatrix(config_);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  WrenchFrictionConeConstraint* WrenchFrictionConeConstraint::clone() const
  { 
    return new WrenchFrictionConeConstraint(*this); 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  size_t WrenchFrictionConeConstraint::getNumConstraints(scalar_t time) const 
  { 
    return 16; 
  };

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool WrenchFrictionConeConstraint::isActive(scalar_t time) const 
  {
    return referenceManager_.getContactFlags(time)[endEffectorIndex_];
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  vector_t WrenchFrictionConeConstraint::getValue(scalar_t time,
    const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const 
  {
    const auto wrenchInWorldFrame = access_helper_functions::getContactWrenches(input, 
      endEffectorIndex_, info_);

    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    // Get rotation matrix to foot frame because the axes are consistent with the leg lengths
    const vector3_t eulerAngles = leggedPrecomputation.getEndEffectorOrientation(
      endEffectorIndex_);
    const matrix3_t rotationMatrixToTerrain = getRotationMatrixFromZyxEulerAngles(
      eulerAngles).transpose();
    
    vector6_t localWrench;
    localWrench.block<3, 1>(0, 0) = rotationMatrixToTerrain * wrenchInWorldFrame.block<3, 1>(0, 0);
    localWrench.block<3, 1>(3, 0) = rotationMatrixToTerrain * wrenchInWorldFrame.block<3, 1>(3, 0);

    const vector_t coneConstraint = coneConstraintMatrix_ * localWrench;

    return coneConstraint;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  VectorFunctionLinearApproximation WrenchFrictionConeConstraint::getLinearApproximation(
    scalar_t time,
    const vector_t &state,
    const vector_t &input,
    const PreComputation &preComp) const 
  {
    const auto wrenchInWorldFrame = access_helper_functions::getContactWrenches(input, 
      endEffectorIndex_, info_);
    
    const auto& leggedPrecomputation = cast<LeggedPrecomputation>(preComp);

    // Get rotation matrix to foot frame because the axes are consistent with the leg lengths
    const vector3_t eulerAngles = leggedPrecomputation.getEndEffectorOrientation(
      endEffectorIndex_);
    const matrix3_t rotationMatrixToTerrain = getRotationMatrixFromZyxEulerAngles(
      eulerAngles).transpose();

    vector6_t localWrench;
    localWrench.block<3, 1>(0, 0) = rotationMatrixToTerrain * wrenchInWorldFrame.block<3, 1>(0, 0);
    localWrench.block<3, 1>(3, 0) = rotationMatrixToTerrain * wrenchInWorldFrame.block<3, 1>(3, 0);

    const vector_t coneConstraint = coneConstraintMatrix_ * localWrench;

    VectorFunctionLinearApproximation linearApproximation;

    linearApproximation.f = coneConstraint;
    linearApproximation.dfdx = matrix_t::Zero(16, info_.stateDim);
    linearApproximation.dfdu = matrix_t::Zero(16, info_.inputDim);

    matrix6_t rotation6x6 = matrix6_t::Zero();
    rotation6x6.block<3, 3>(0, 0) = rotationMatrixToTerrain;
    rotation6x6.block<3, 3>(3, 3) = rotationMatrixToTerrain;

    const size_t startIndex = (3 * info_.numThreeDofContacts + 6 * (endEffectorIndex_ - info_.numThreeDofContacts));

    linearApproximation.dfdu.block<16, 6>(0, startIndex) = coneConstraintMatrix_ * rotation6x6;
    
    return linearApproximation;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::Matrix<scalar_t, 16, 6> WrenchFrictionConeConstraint::generateConeConstraintMatrix(
    const Config& config)
  {
    Eigen::Matrix<scalar_t, 16, 6> coneConstraintMatrix;

    // u * f_z + f_x >= 0
    coneConstraintMatrix(0, 0) = 1.0;
    coneConstraintMatrix(0, 2) = config.frictionCoefficient;

    // u * f_z - f_x >= 0
    coneConstraintMatrix(1, 0) = -1.0;
    coneConstraintMatrix(1, 2) = config.frictionCoefficient;

    // u * f_z + f_y >= 0
    coneConstraintMatrix(2, 1) = 1.0;
    coneConstraintMatrix(2, 2) = config.frictionCoefficient;

    // u * f_z - f_y >= 0
    coneConstraintMatrix(3, 1) = -1.0;
    coneConstraintMatrix(3, 2) = config.frictionCoefficient;

    // Y * f_z + tau_x >= 0
    coneConstraintMatrix(4, 2) = config.footHalfLengthY;
    coneConstraintMatrix(4, 3) = config.frictionCoefficient;

    // Y * f_z - tau_x >= 0
    coneConstraintMatrix(5, 2) = config.footHalfLengthY;
    coneConstraintMatrix(5, 3) = -config.frictionCoefficient;

    // X * f_z + tau_y >= 0
    coneConstraintMatrix(6, 2) = config.footHalfLengthX;
    coneConstraintMatrix(7, 4) = config.frictionCoefficient;

    // X * f_z - tau_y >= 0
    coneConstraintMatrix(7, 2) = config.footHalfLengthX;
    coneConstraintMatrix(7, 4) = -config.frictionCoefficient;

    // Y * f_x + X * f_y + u * (X + Y) * f_z - u * tau_x - u * tau_y + tau_z >= 0
    coneConstraintMatrix(8, 0) = config.footHalfLengthY;
    coneConstraintMatrix(8, 1) = config.footHalfLengthX;
    coneConstraintMatrix(8, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(8, 3) = -config.frictionCoefficient;
    coneConstraintMatrix(8, 4) = -config.frictionCoefficient;
    coneConstraintMatrix(8, 5) = 1;

    // Y * f_x - X * f_y + u * (X + Y) * f_z - u * tau_x + u * tau_y + tau_z >= 0
    coneConstraintMatrix(9, 0) = config.footHalfLengthY;
    coneConstraintMatrix(9, 1) = -config.footHalfLengthX;
    coneConstraintMatrix(9, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(9, 3) = -config.frictionCoefficient;
    coneConstraintMatrix(9, 4) = config.frictionCoefficient;
    coneConstraintMatrix(9, 5) = 1;

    // -Y * f_x + X * f_y + u * (X + Y) * f_z - u * tau_x - u * tau_y + tau_z >= 0
    coneConstraintMatrix(10, 0) = -config.footHalfLengthY;
    coneConstraintMatrix(10, 1) = config.footHalfLengthX;
    coneConstraintMatrix(10, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(10, 3) = config.frictionCoefficient;
    coneConstraintMatrix(10, 4) = -config.frictionCoefficient;
    coneConstraintMatrix(10, 5) = 1;

    // -Y * f_x - X * f_y + u * (X + Y) * f_z + u * tau_x + u * tau_y + tau_z >= 0
    coneConstraintMatrix(11, 0) = -config.footHalfLengthY;
    coneConstraintMatrix(11, 1) = -config.footHalfLengthX;
    coneConstraintMatrix(11, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(11, 3) = config.frictionCoefficient;
    coneConstraintMatrix(11, 4) = config.frictionCoefficient;
    coneConstraintMatrix(11, 5) = 1;

    // -Y * f_x - X * f_y + u * (X + Y) * f_z - u * tau_x - u * tau_y - tau_z >= 0
    coneConstraintMatrix(12, 0) = -config.footHalfLengthY;
    coneConstraintMatrix(12, 1) = -config.footHalfLengthX;
    coneConstraintMatrix(12, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(12, 3) = -config.frictionCoefficient;
    coneConstraintMatrix(12, 4) = -config.frictionCoefficient;
    coneConstraintMatrix(12, 5) = -1;

    // -Y * f_x + X * f_y + u * (X + Y) * f_z - u * tau_x + u * tau_y - tau_z >= 0
    coneConstraintMatrix(13, 0) = -config.footHalfLengthY;
    coneConstraintMatrix(13, 1) = config.footHalfLengthX;
    coneConstraintMatrix(13, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(13, 3) = -config.frictionCoefficient;
    coneConstraintMatrix(13, 4) = config.frictionCoefficient;
    coneConstraintMatrix(13, 5) = -1;

    // Y * f_x - X * f_y + u * (X + Y) * f_z + u * tau_x - u * tau_y - tau_z >= 0
    coneConstraintMatrix(14, 0) = config.footHalfLengthY;
    coneConstraintMatrix(14, 1) = -config.footHalfLengthX;
    coneConstraintMatrix(14, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(14, 3) = config.frictionCoefficient;
    coneConstraintMatrix(14, 4) = -config.frictionCoefficient;
    coneConstraintMatrix(14, 5) = -1;

    // Y * f_x + X * f_y + u * (X + Y) * f_z + u * tau_x + u * tau_y - tau_z >= 0
    coneConstraintMatrix(15, 0) = config.footHalfLengthY;
    coneConstraintMatrix(15, 1) = config.footHalfLengthX;
    coneConstraintMatrix(15, 2) = (config.footHalfLengthY + config.footHalfLengthX) * config.frictionCoefficient;
    coneConstraintMatrix(15, 3) = config.frictionCoefficient;
    coneConstraintMatrix(15, 4) = config.frictionCoefficient;
    coneConstraintMatrix(15, 5) = -1;

    return coneConstraintMatrix;
  }

  WrenchFrictionConeConstraint::Config loadWrenchFrictionConeConfig(
    const std::string &filename, const std::string &fieldName,bool verbose)
  {
    scalar_t frictionCoefficient = -1.0;
    scalar_t footHalfLengthX = -1.0;
    scalar_t footHalfLengthY = -1.0;

    boost::property_tree::ptree pt;
    read_info(filename, pt);

    if (verbose) 
    {
      std::cerr << "\n #### 3D Friction Cone Constraint Config:";
      std::cerr << "\n #### =============================================================================\n";
    }

    ocs2::loadData::loadPtreeValue(pt, frictionCoefficient, fieldName + ".frictionCoefficient", verbose);

    if(frictionCoefficient < 0)
    {
      throw std::invalid_argument("[WrenchFrictionConeConstraint]: Fricition coefficient smaller than 0!");
    }

    ocs2::loadData::loadPtreeValue(pt, footHalfLengthX, fieldName + ".footLengthX", verbose);
    footHalfLengthX *= 0.5;

    if(footHalfLengthX < 0)
    {
      throw std::invalid_argument("[WrenchFrictionConeConstraint]: Foot length in direction X smaller than 0!");
    }
    
    ocs2::loadData::loadPtreeValue(pt, footHalfLengthY, fieldName + ".footLengthY", verbose);
    footHalfLengthY *= 0.5;

    if(footHalfLengthY < 0)
    {
      throw std::invalid_argument("[WrenchFrictionConeConstraint]: Foot length in direction Y smaller than 0!");
    }

    WrenchFrictionConeConstraint::Config config = WrenchFrictionConeConstraint::Config(
      footHalfLengthX, footHalfLengthY, frictionCoefficient);
      
    return config;
  }

} // namespace legged_locomotion_mpc