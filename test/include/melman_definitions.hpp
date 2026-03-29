#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <legged_locomotion_mpc/path_management/package_path.h>

namespace legged_locomotion_mpc 
{

  enum Melman: size_t {
    STATE_DIM = 6 + 6 + 18,  // base body velocity, generalized coordinates
    INPUT_DIM = 2 * 6 + 18,  // end effector forces, actuated joint velocities
    ACTUATED_DOF_NUM = 12,
    GENERALIZED_COORDINATES_NUM = 6 + 12, 
    NUM_THREE_DOF_CONTACTS = 2, // FOR TEST
    NUM_SIX_DOF_CONTACTS = 2,   // FOR TEST
    ROBOT_STATE_DIM = 6 + 6 + 12 + 12,  // base body velocity, generalized coordinates and velocities
  };

  // Contact frames
  static const std::vector<std::string> meldog3DofContactNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};
  static const std::vector<std::string> meldog6DofContactNames;
  
  // Example collision names
  static const std::vector<std::string> meldogCollisions = {"LFLL_link", "RFLL_link", "LRLL_link", "RRLL_link"};

  // Hip and or upper leg frame names
  static const std::vector<std::string> meldogHipNames = {"RFUL_link", "RRUL_link", "LFUL_link", "LRUL_link"};

  // Two urdfs, one with additional link before base link (will be removed by factory function)
  static const std::string meldogWithBaseLinkUrdfFile = legged_locomotion_mpc::package_path::getPath() + "/test/test_models/meldog_base_link.urdf";
  static const std::string meldogWithoutBaseLinkUrdfFile = legged_locomotion_mpc::package_path::getPath() + "/test/test_models/meldog_no_base_link.urdf";
  static const std::string meldogConfigFolder = legged_locomotion_mpc::package_path::getPath() + "/test/config/";
  
  // Base link name
  static const std::string baseLink = "trunk_link";

  // End effector parent joint frame names
  static const std::vector<std::string> meldogEndEffectorJointNames = {"LFK_joint", "LRK_joint", "RFK_joint", "RRK_joint"};

}  // namespace legged_locomotion_mpc