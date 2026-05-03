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

#ifndef __MELDOG_DEFINITIONS_LOCOMOTION_MPC_ROS2__
#define __MELDOG_DEFINITIONS_LOCOMOTION_MPC_ROS2__

#include <cstddef>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <legged_locomotion_mpc_ros2/path_management/package_path.h>

namespace legged_locomotion_mpc_ros2
{
  namespace meldog_definitions
  {
    enum Meldog: size_t 
    {
      STATE_DIM = 6 + 6 + 12,  // base body velocity, generalized coordinates
      INPUT_DIM = 4 * 3 + 12,  // end effector forces, actuated joint velocities
      ACTUATED_DOF_NUM = 12,
      GENERALIZED_COORDINATES_NUM = 6 + 12, 
      NUM_THREE_DOF_CONTACTS = 4, 
      NUM_SIX_DOF_CONTACTS = 0,   
    };

    // Contact frames
    static const std::vector<std::string> threeDofContactNames = {"RFF_link", "RRF_link", "LFF_link", "LRF_link"};
    static const std::vector<std::string> sixDofContactNames;

    // URDF
    static const std::string urdfFile = legged_locomotion_mpc_ros2::package_path::getPath() + "/include/legged_locomotion_mpc_ros2/demos/meldog_demo/meldog.urdf";

    // Config directory
    static const std::string configDirectory = legged_locomotion_mpc_ros2::package_path::getPath() + "/include/legged_locomotion_mpc_ros2/demos/meldog_demo/config/";

    // Base link name
    static const std::string baseLink = "trunk_link";
    
  } // namespace meldog_definitions
} // namespace legged_locomotion_mpc

#endif