#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <legged_locomotion_mpc/path_management/package_path.h>

namespace legged_locomotion_mpc 
{

  enum Meldog : size_t {
    STATE_DIM = 6 + 6 + 12,  // base body velocity, generalized coordinates
    INPUT_DIM = 2 * 3 + 2 * 6 + 12,  // end effector forces, actuated joint velocities
    ACTUATED_DOF_NUM = 12,
    GENERALIZED_COORDINATES_NUM = 6 + 12, 
    NUM_THREE_DOF_CONTACTS = 2, // FOR TEST
    NUM_SIX_DOF_CONTACTS = 2,   // FOR TEST
    ROBOT_STATE_DIM = 6 + 6 + 12 + 12,  // base body velocity, generalized coordinates and velocities
  };

  // Contact frames
  static const std::vector<std::string> meldog3DofContactNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};
  static const std::vector<std::string> meldog6DofContactNames;

  // Two urdfs, one with additional link before base link (will be removed by factory function)
  static const std::string meldogWithBaseLinkUrdfFile = legged_locomotion_mpc::package_path::getPath() + "/test/test_models/meldog_base_link.urdf";
  static const std::string meldogWithoutBaseLinkUrdfFile = legged_locomotion_mpc::package_path::getPath() + "/test/test_models/meldog_no_base_link.urdf";
  
  // Base link name
  static const std::string baseLink = "trunk_link";

  // End effector parent joint frame names
  static const std::vector<std::string> meldogEndEffectorJointNames = {"LFK_joint", "LRK_joint", "RFK_joint", "RRK_joint"};

  inline ocs2::vector_t getAccessTestRobotState() 
  {
    ocs2::vector_t x = ocs2::vector_t::Zero(Meldog::ROBOT_STATE_DIM);
    
    // Base Velocitiy: [linear, angular]
    x(0) = 0.0;  // vB_x
    x(1) = 1.0;  // vB_y
    x(2) = 2.0;  // vB_z
    x(3) = 3.0;  // wB_x
    x(4) = 4.0;  // wB_y
    x(5) = 5.0;  // wB_z
  
    // Base Pose: [position, orientation]
    x(6)  = 6.0;   // p_base_x
    x(7)  = 7.0;   // p_base_y
    x(8)  = 8.0;   // p_base_z
    x(9)  = 9.0;   // theta_base_z
    x(10) = 10.0;  // theta_base_y
    x(11) = 11.0;  // theta_base_x
  
    // Leg Joint Positions: 
    x(12) = 12.0;   
    x(13) = 13.0;
    x(14) = 14.0;
    x(15) = 15.0;
    x(16) = 16.0;
    x(17) = 17.0;
    x(18) = 18.0;
    x(19) = 19.0;
    x(20) = 20.0;    
    x(21) = 21.0;
    x(22) = 22.0;
    x(23) = 23.0;

    // Leg Joint Velocities: 
    x(24) = 24.0;
    x(25) = 25.0;
    x(26) = 26.0;
    x(27) = 27.0;
    x(28) = 28.0;
    x(29) = 29.0;
    x(30) = 30.0;
    x(31) = 31.0;    
    x(32) = 32.0;
    x(33) = 33.0;
    x(34) = 34.0;
    x(35) = 35.0;   
   
    return x;
  };
}  // namespace legged_locomotion_mpc