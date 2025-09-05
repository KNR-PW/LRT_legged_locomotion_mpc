
#include <legged_locomotion_mpc/kinematics/InverseEndEffectorKinematics.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace legged_locomotion_mpc
{
  using namespace ocs2;
  using namespace lrt_inverse_kinematics;
  InverseEndEffectorKinematics::InverseEndEffectorKinematics(
    InverseKinematics&& kinematicsSolver): kinematicsSolver_(std::move(kinematicsSolver)) {}

  ocs2::vector_t InverseEndEffectorKinematics::getJointPositions(
    const ocs2::vector_t& actualJointPositions,
    const vector6_t basePose,  
    const std::vector<vector3_t>& endEffectorPositions)
  {
    const size_t numEndEffectors = kinematicsSolver_.getModelInternalInfo().numEndEffectors_;

    std::vector<vector3_t> relativeEndEffectorPositions(numEndEffectors);
    const static std::vector<pinocchio::SE3> endEffectorTransforms;

    const vector3_t baseOrientationZyx = basePose.block<3, 1>(3, 0);
    const matrix3_t worldToBaseRotationMatrix = getRotationMatrixFromZyxEulerAngles(
      baseOrientationZyx).transpose();

    for(size_t i = 0; i < numEndEffectors; ++i)
    {
      relativeEndEffectorPositions[i].noalias() = worldToBaseRotationMatrix * 
        (endEffectorPositions[i] - basePose.block<3, 1>(0, 0));
    }

    vector_t newJointPositions;

    const ReturnStruct returnData = kinematicsSolver_.calculateJointPositions(
      actualJointPositions, relativeEndEffectorPositions, 
      endEffectorTransforms, newJointPositions);
    
    if(returnData.success_) return newJointPositions;
    else
    {
      switch (returnData.flag_)
        {
          case ReturnFlag::IN_PROGRESS:
            return newJointPositions;
            break;

          case ReturnFlag::SMALL_STEP_SIZE:
            return newJointPositions;
            break;

          case ReturnFlag::POSITION_OUT_OF_BOUNDS:
            throw std::runtime_error("[InverseEndEffectorKinematics]: Error: " 
              + returnData.toString());
            break;

          default:
            return newJointPositions;
            break;
      }
    }
  }

  ocs2::vector_t InverseEndEffectorKinematics::getJointVelocities(
    const ocs2::vector_t& actualJointPositions, const vector6_t basePose, 
    const vector6_t baseVelocity, const std::vector<vector3_t>& endEffectorVelocities)
  {
    const size_t numEndEffectors = kinematicsSolver_.getModelInternalInfo().numEndEffectors_;

    vector_t relativeEndEffectorVelocities(3 * numEndEffectors);

    const vector3_t baseOrientationZyx = basePose.block<3, 1>(3, 0);
    const matrix3_t worldToBaseRotationMatrix = getRotationMatrixFromZyxEulerAngles(
      baseOrientationZyx).transpose();
    
    for(size_t i = 0; i < numEndEffectors; ++i)
    {
      relativeEndEffectorVelocities.block<3, 1>(3 * i, 0).noalias() = worldToBaseRotationMatrix * 
        (endEffectorVelocities[i]) - baseVelocity.block<3, 1>(0, 0);
    }

    const matrix_t jacobian = kinematicsSolver_.getJacobian(actualJointPositions);

    const vector_t jointVelocities = jacobian.ldlt().solve(relativeEndEffectorVelocities);

    if(kinematicsSolver_.checkVelocityBounds(jointVelocities)) return jointVelocities;
    else throw("[InverseEndEffectorKinematics]: Error: VELOCITY_OUT_OF_BOUNDS");
  }

  IKModelInfo loadIKModelInfo(const std::string &filename, bool verbose)
  {
    IKModelInfo settings{};

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    const std::string prefix{"model_settings.inverse_kinematics_model_settings."};
    const std::string threeDofContactsPrefix{"threeDofEndEffectorNames"};
    const std::string sixDofContactsPrefix{"sixDofEndEffectorNames"};

    if (verbose) 
    {
      std::cerr << "\n #### Inverse Kinematics Model Settings:" << std::endl;
      std::cerr << "#### IMPORTANT: Six DOF end effectors not supported for IK!" << std::endl;
      std::cerr << " #### ==================================================" << std::endl;
    }

    loadData::loadPtreeValue(pt, settings.baseLinkName_, 
      prefix + "baseLinkName", verbose);
    loadData::loadStdVector(filename, threeDofContactsPrefix, 
      settings.threeDofEndEffectorNames_);
    loadData::loadStdVector(filename, sixDofContactsPrefix, 
      settings.sixDofEndEffectorNames_);

    settings.threeDofEndEffectorNames_.insert(settings.threeDofEndEffectorNames_.end(), 
      settings.sixDofEndEffectorNames_.begin(), settings.sixDofEndEffectorNames_.end());

    settings.sixDofEndEffectorNames_.clear();
    if (verbose) 
    {
      std::cerr << " #### ==================================================" << std::endl;
    }

    return settings;
  }

  IKSolverInfo loadIKSolverInfo(const std::string &filename, bool verbose)
  {
    IKSolverInfo settings;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    const std::string prefix{"model_settings.inverse_kinematics_solver_settings."};

    if (verbose) 
    {
      std::cerr << "\n #### Inverse Kinematics Model Settings:" << std::endl;
      std::cerr << "#### IMPORTANT: Six DOF end effectors not supported for IK!" << std::endl;
      std::cerr << " #### ==================================================" << std::endl;
    }

    loadData::loadPtreeValue(pt, settings.maxIterations_, 
      prefix + "maxIterations", verbose);
    loadData::loadPtreeValue(pt, settings.tolerance_, 
      prefix + "tolerance", verbose);
    loadData::loadPtreeValue(pt, settings.minimumStepSize_, 
      prefix + "minimumStepSize", verbose);
    loadData::loadPtreeValue(pt, settings.dampingCoefficient_, 
      prefix + "dampingCoefficient", verbose);
    loadData::loadPtreeValue(pt, settings.stepCoefficient_, 
      prefix + "stepCoefficient", verbose);

    if (verbose) 
    {
      std::cerr << " #### ==================================================" << std::endl;
    }

    return settings;
  }

  const std::string loadIKSolverName(const std::string &filename, bool verbose)
  {

    std::string solverName;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    const std::string prefix{"model_settings.inverse_kinematics_solver_name."};

    if (verbose) 
    {
      std::cerr << "\n #### Inverse Kinematics Model Settings:" << std::endl;
      std::cerr << "#### IMPORTANT: Six DOF end effectors not supported for IK!" << std::endl;
      std::cerr << " #### ==================================================" << std::endl;
    }

    loadData::loadPtreeValue(pt, solverName, 
      prefix + "", verbose);

    if (verbose) 
    {
      std::cerr << " #### ==================================================" << std::endl;
    }

    return solverName;
  }
} // namespace legged_locomotion_mpc
