cmake_minimum_required(VERSION 3.16)
project(legged_locomotion_mpc LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Resolve for the package path at compile time.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h" @ONLY
)


##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPENDS
    ocs2_ddp
    ocs2_mpc
    ocs2_sqp
    ocs2_ipm
    floating_base_model
    terrain_model
    multi_end_effector_kinematics
)

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# add_executable(${PROJECT_NAME}
#         # src/common/Utils.cpp
#         # src/common/ModelSettings.cpp
#         # src/kinematics/PinocchioForwardEndEffectorKinematicsCppAd.cpp
#         # src/kinematics/ForwardEndEffectorKinematicsInfo.cpp
#         # src/dynamics/LeggedDynamicsAD.cpp
        
#         # src/constraint/ZeroForceConstraint.cpp
#         # src/constraint/ZeroWrenchConstraint.cpp
#         # src/constraint/ForceFrictionConeConstraint.cpp
#         # src/constraint/WrenchFrictionConeConstraint.cpp
#         src/main.cpp
# )

add_library(${PROJECT_NAME}
  src/common/Utils.cpp
  src/common/ModelSettings.cpp
  src/common/ModelHelperFunctions.cpp

  src/locomotion/ModeSequenceTemplate.cpp
  src/locomotion/GaitPlanner.cpp
  src/locomotion/ModeDynamicSequenceTemplate.cpp
  src/locomotion/GaitDynamicPhaseController.cpp
  src/locomotion/QuinticSplineSwing.cpp
  src/locomotion/SwingSpline3d.cpp
  src/locomotion/SingleLegLogic.cpp
  src/locomotion/FootPhase.cpp
  src/locomotion/SwingTrajectoryPlanner.cpp

  src/trajectory_planners/BaseTrajectoryPlanner.cpp
  src/trajectory_planners/JointTrajectoryPlanner.cpp

  src/synchronization/GaitSynchronizedModule.cpp
  src/synchronization/DisturbanceSynchronizedModule.cpp

  src/dynamics/LeggedDynamicsAD.cpp

  # src/constraint/ForceFrictionConeConstraint.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPENDS})
target_compile_options(${PROJECT_NAME} PUBLIC ${FLAGS})

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS src include test
            CT_HEADER_DIRS include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)


#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# install(
#         DIRECTORY include/
#         DESTINATION include/${PROJECT_NAME}
# )

# install(
#         TARGETS ${PROJECT_NAME}
#         EXPORT export_${PROJECT_NAME}
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION bin
# )
# install(TARGETS ${PROJECT_NAME}
#     DESTINATION lib/${PROJECT_NAME})

# install(
#         DIRECTORY config
#         DESTINATION share/${PROJECT_NAME}/
# )

# ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  # # find_package(ament_lint_auto REQUIRED)
  # # the following line skips the linter which checks for copyrights
  # # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # # the following line skips cpplint (only works in a git repo)
  # # comment the line when this package is in a git repo and when
  # # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_common
          test/common/AccessHelperFunctionsTest.cpp
          
  )
  ament_target_dependencies(${PROJECT_NAME}_test_common ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_common PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_common ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_common PUBLIC ${FLAGS})

  ament_add_gtest(${PROJECT_NAME}_test_locomotion
          test/locomotion/GaitCommonTest.cpp
          test/locomotion/ModeDynamicSequenceTemplateTest.cpp
          test/locomotion/GaitPlannerTest.cpp
          test/locomotion/GaitDynamicPhaseControllerTest.cpp
          test/locomotion/FootPhaseTest.cpp
          test/locomotion/SingleLegLogicTest.cpp
          
  )
  ament_target_dependencies(${PROJECT_NAME}_test_locomotion ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test_locomotion PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test_locomotion ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test_locomotion PUBLIC ${FLAGS})

  find_package(ament_cmake_google_benchmark REQUIRED)
  ament_add_google_benchmark(${PROJECT_NAME}_benchmark benchmark/GaitDynamicPhaseControllerBenchmark.cpp)
  target_include_directories(${PROJECT_NAME}_benchmark PRIVATE benchmark/include)
  target_link_libraries(${PROJECT_NAME}_benchmark ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_benchmark PUBLIC ${FLAGS})
endif()

ament_package()
