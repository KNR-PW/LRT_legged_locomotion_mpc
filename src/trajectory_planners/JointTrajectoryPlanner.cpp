
#include <legged_locomotion_mpc/trajectory_planners/JointTrajectoryPlanner.hpp>
#include <legged_locomotion_mpc/common/AccessHelperFunctions.hpp>

#include <floating_base_model/AccessHelperFunctions.hpp>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace legged_locomotion_mpc
{
  namespace planners
  {
    using namespace ocs2;
    using namespace floating_base_model;
    using namespace multi_end_effector_kinematics;
    using namespace legged_locomotion_mpc::locomotion;

    JointTrajectoryPlanner::JointTrajectoryPlanner(
      FloatingBaseModelInfo modelInfo,
      MultiEndEffectorKinematics&& kinematicsSolver): 
        modelInfo_(std::move(modelInfo)),
        kinematicsSolver_(std::move(kinematicsSolver)),
        jointPositionUpperLimits_(std::move(kinematicsSolver_.getPinocchioModel().lowerPositionLimit)),
        jointPositionLowerLimits_(std::move(kinematicsSolver_.getPinocchioModel().upperPositionLimit)),
        jointVelocityLimits_(std::move(kinematicsSolver_.getPinocchioModel().velocityLimit))
      {
        if(jointPositionUpperLimits_.size() != modelInfo_.actuatedDofNum)
        {
          std::stringstream stringStream;
          stringStream << "[JointTrajectoryPlanner]: Size of joint upper limits is " <<  jointPositionUpperLimits_.size() 
            << ", should be " << modelInfo_.actuatedDofNum << "!";
          throw std::invalid_argument(stringStream.str());
        }

        if(jointPositionLowerLimits_.size() != modelInfo_.actuatedDofNum)
        {
          std::stringstream stringStream;
          stringStream << "[JointTrajectoryPlanner]: Size of joint lower limits is " <<  jointPositionLowerLimits_.size() 
            << ", should be " << modelInfo_.actuatedDofNum << "!";
          throw std::invalid_argument(stringStream.str());
        }

        if(jointVelocityLimits_.size() != modelInfo_.actuatedDofNum)
        {
          std::stringstream stringStream;
          stringStream << "[JointTrajectoryPlanner]: Size of joint velocity limits is " << jointVelocityLimits_.size() 
            << ", should be " << modelInfo_.actuatedDofNum << "!";
          throw std::invalid_argument(stringStream.str());
        }
      }
    
    void JointTrajectoryPlanner::updateTrajectory(const state_vector_t& currentState,
      TargetTrajectories& targetTrajectories, 
      const position_trajectories& endEffectorPositionTrajectories,
      const velocity_trajectories& endEffectorVelocityTrajectories)
    {
      assert(endEffectorPositionTrajectories.size() == endEffectorVelocityTrajectories.size());
      assert(endEffectorPositionTrajectories.size() == targetTrajectories.timeTrajectory.size());
      
      const size_t trajectorySize = targetTrajectories.timeTrajectory.size();
      auto& currentOptimalState = targetTrajectories.stateTrajectory[0];
      auto& currentOptimalInput = targetTrajectories.inputTrajectory[0];

      floating_base_model::access_helper_functions::getJointPositions(currentOptimalState, modelInfo_) 
        = legged_locomotion_mpc::access_helper_functions::getJointPositions(currentState, modelInfo_);
      
      floating_base_model::access_helper_functions::getJointVelocities(currentOptimalInput, modelInfo_) 
        = legged_locomotion_mpc::access_helper_functions::getJointVelocities(currentState, modelInfo_);
      
      for(size_t i = 1; i < trajectorySize; ++i)
      {
        const scalar_t currentTime = targetTrajectories.timeTrajectory[i];
        const scalar_t previousTime = targetTrajectories.timeTrajectory[i - 1];

        const scalar_t deltaTime = currentTime - previousTime;

        const std::vector<vector3_t>& newEndEffectorPositions = 
          endEffectorPositionTrajectories[i];
        const std::vector<vector3_t>& newEndEffectorVelocities = 
          endEffectorVelocityTrajectories[i];

        const auto& previousState = targetTrajectories.stateTrajectory[i - 1];
        const auto& previousInput = targetTrajectories.inputTrajectory[i - 1];

        const vector_t previousJointPositions = 
          floating_base_model::access_helper_functions::getJointPositions(previousState, 
            modelInfo_);

        const vector_t previousJointVelocities = 
          floating_base_model::access_helper_functions::getJointVelocities(previousInput, 
            modelInfo_);

        // Extrapolate using previous joint velocity
        const vector_t extrapolatedJointPositions = previousJointPositions + 
            previousJointVelocities * deltaTime;
            
        vector_t& newState = targetTrajectories.stateTrajectory[i];
        vector_t& newInput = targetTrajectories.inputTrajectory[i];

        const vector6_t& newBasePose = 
          floating_base_model::access_helper_functions::getBasePose(newState, modelInfo_);
        
        const vector6_t& newBaseVelocity = 
          floating_base_model::access_helper_functions::getBaseVelocity(newState, modelInfo_);

        auto newJointPositions = 
          floating_base_model::access_helper_functions::getJointPositions(newState, modelInfo_);

        auto newJointVelocities = 
          floating_base_model::access_helper_functions::getJointVelocities(newInput, modelInfo_);
        
        newJointPositions = computeJointPositions(extrapolatedJointPositions, newBasePose, 
          newEndEffectorPositions);
          
        newJointVelocities = computeJointVelocities(newJointPositions, newBasePose,
          newBaseVelocity, newEndEffectorVelocities);
      }
    }

    vector_t JointTrajectoryPlanner::computeJointPositions(
      const vector_t& actualJointPositions,
      const vector6_t& basePose,  
      const std::vector<vector3_t>& endEffectorPositions)
    {
      const size_t numEndEffectors = kinematicsSolver_.getModelInternalSettings().numEndEffectors;

      std::vector<vector3_t> relativeEndEffectorPositions(numEndEffectors);

      const vector3_t baseOrientationZyx = basePose.block<3, 1>(3, 0);
      const matrix3_t worldToBaseRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        baseOrientationZyx).transpose();

      for(size_t i = 0; i < numEndEffectors; ++i)
      {
        relativeEndEffectorPositions[i].noalias() = worldToBaseRotationMatrix * 
          (endEffectorPositions[i] - basePose.block<3, 1>(0, 0));
      }

      vector_t newJointPositions;

      const ReturnStatus returnStatus = kinematicsSolver_.calculateJointPositions(
        actualJointPositions, relativeEndEffectorPositions, newJointPositions);
      
      if(returnStatus.success) return newJointPositions;
      else
      {
        switch (returnStatus.flag)
        {
          case TaskReturnFlag::IN_PROGRESS:
            return newJointPositions;
            break;

          case TaskReturnFlag::SMALL_STEP_SIZE:
            return newJointPositions;
            break;

          case TaskReturnFlag::NEW_POSITION_OUT_OF_BOUNDS:
            for(size_t i = 0; i < modelInfo_.actuatedDofNum; ++i)
            {
              if(newJointPositions[i] > jointPositionUpperLimits_[i])
              {
                newJointPositions[i] = jointPositionUpperLimits_[i];
              }
              else if(newJointPositions[i] < jointPositionLowerLimits_[i])
              {
                newJointPositions[i] = jointPositionLowerLimits_[i];
              }
            }
            return newJointPositions;
            break;

          default:
            return newJointPositions;
            break;
        }
      }
    }

    vector_t JointTrajectoryPlanner::computeJointVelocities(
      const vector_t& actualJointPositions, const vector6_t& basePose, 
      const vector6_t& baseVelocity, const std::vector<vector3_t>& endEffectorVelocities)
    {
      const size_t numEndEffectors = kinematicsSolver_.getModelInternalSettings().numEndEffectors;

      std::vector<vector3_t> relativeEndEffectorVelocities(numEndEffectors);

      const vector3_t baseOrientationZyx = basePose.block<3, 1>(3, 0);
      const matrix3_t worldToBaseRotationMatrix = getRotationMatrixFromZyxEulerAngles(
        baseOrientationZyx).transpose();

      std::vector<vector3_t> relativeEndEffectorPositions(numEndEffectors);

      kinematicsSolver_.calculateEndEffectorPoses(actualJointPositions, 
        relativeEndEffectorPositions);

      const vector3_t& baseLinearVelocity = baseVelocity.block<3, 1>(0, 0);
      const vector3_t& baseAngularVelocity = baseVelocity.block<3, 1>(3, 0);
      
      for(size_t i = 0; i < numEndEffectors; ++i)
      {
        relativeEndEffectorVelocities[i].noalias() = worldToBaseRotationMatrix * 
          (endEffectorVelocities[i]) - baseLinearVelocity - 
          baseAngularVelocity.cross(relativeEndEffectorPositions[i]);
      }

      vector_t jointVelocities;

      const ReturnStatus returnStatus = kinematicsSolver_.calculateJointVelocities(
        actualJointPositions, relativeEndEffectorVelocities, jointVelocities);
      
      if(!returnStatus.success)
      {
        for(size_t i = 0; i < modelInfo_.actuatedDofNum; ++i)
        {
          if(jointVelocities[i] > jointVelocityLimits_[i])
          {
            jointVelocities[i] = jointVelocityLimits_[i];
          }
          else if(jointVelocities[i] < -jointVelocityLimits_[i])
          {
            jointVelocities[i] = -jointVelocityLimits_[i];
          }
        }
      }
      return jointVelocities;
    }

    KinematicsModelSettings loadKinematicsModelSettings(
      const std::string& filename, bool verbose)
    {
      KinematicsModelSettings settings{};

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      const std::string prefix{"model_settings.inverse_kinematics_model_settings."};
      const std::string threeDofContactsPrefix{"threeDofEndEffectorNames"};
      const std::string sixDofContactsPrefix{"sixDofEndEffectorNames"};

      if(verbose) 
      {
        std::cerr << "\n #### Inverse Kinematics Model Settings:" << std::endl;
        std::cerr << "#### IMPORTANT: Six DOF end effectors not supported, added to three DOF!" << std::endl;
        std::cerr << " #### ==================================================" << std::endl;
      }

      loadData::loadPtreeValue(pt, settings.baseLinkName, 
        prefix + "baseLinkName", verbose);
      loadData::loadStdVector(filename, threeDofContactsPrefix, 
        settings.threeDofEndEffectorNames);
      loadData::loadStdVector(filename, sixDofContactsPrefix, 
        settings.sixDofEndEffectorNames);

      settings.threeDofEndEffectorNames.insert(settings.threeDofEndEffectorNames.end(), 
        settings.sixDofEndEffectorNames.begin(), settings.sixDofEndEffectorNames.end());

      settings.sixDofEndEffectorNames.clear();
      if(verbose) 
      {
        std::cerr << " #### ==================================================" << std::endl;
      }

      return settings;
    }

    InverseSolverSettings loadInverseSolverSettings(
      const std::string& filename, bool verbose)
    {
      InverseSolverSettings settings;

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      const std::string prefix{"model_settings.inverse_kinematics_solver_settings."};

      if(verbose) 
      {
        std::cerr << "\n #### Inverse Kinematics Solver Settings:" << std::endl;
        std::cerr << " #### ==================================================" << std::endl;
      }

      loadData::loadPtreeValue(pt, settings.maxIterations, 
        prefix + "maxIterations", verbose);
      loadData::loadPtreeValue(pt, settings.tolerance, 
        prefix + "tolerance", verbose);
      loadData::loadPtreeValue(pt, settings.minimumStepSize, 
        prefix + "minimumStepSize", verbose);
      loadData::loadPtreeValue(pt, settings.dampingCoefficient, 
        prefix + "dampingCoefficient", verbose);
      loadData::loadPtreeValue(pt, settings.stepCoefficient, 
        prefix + "stepCoefficient", verbose);

      if(verbose) 
      {
        std::cerr << " #### ==================================================" << std::endl;
      }

      return settings;
    }

    std::string loadInverseSolverName(const std::string& filename, bool verbose)
    {

      std::string solverName;

      boost::property_tree::ptree pt;
      boost::property_tree::read_info(filename, pt);

      const std::string prefix{"model_settings.inverse_kinematics_solver_name."};

      if(verbose) 
      {
        std::cerr << "\n #### Inverse Kinematics Solver Name:" << std::endl;
        std::cerr << " #### ==================================================" << std::endl;
      }

      loadData::loadPtreeValue(pt, solverName, 
        prefix + "", verbose);

      if(verbose) 
      {
        std::cerr << " #### ==================================================" << std::endl;
      }

      return solverName;
    }
  } // namespace planners
} // namespace legged_locomotion_mpc
