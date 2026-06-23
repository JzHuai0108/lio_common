cmake_minimum_required(VERSION 3.16)
project(lio_common VERSION 0.1.0 LANGUAGES CXX)

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

add_library(lio_common src/common.cpp)
set_target_properties(lio_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(lio_common::lio_common ALIAS lio_common)

target_compile_features(lio_common PUBLIC cxx_std_17)
target_include_directories(lio_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(lio_common PUBLIC Eigen3::Eigen)

install(TARGETS lio_common
  EXPORT lio_commonTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(EXPORT lio_commonTargets
  FILE lio_commonTargets.cmake
  NAMESPACE lio_common::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lio_common
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/lio_commonConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/lio_commonConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lio_common
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/lio_commonConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/lio_commonConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/lio_commonConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/lio_common
)
