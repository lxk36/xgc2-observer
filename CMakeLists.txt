cmake_minimum_required(VERSION 3.16)
project(xgc2_observer VERSION 0.3.3 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(XGC2_OBSERVER_BUILD_TESTING "Build xgc2_observer tests" ON)

find_package(Eigen3 REQUIRED)

add_library(xgc2_observer INTERFACE)
add_library(xgc2_observer::observer ALIAS xgc2_observer)

set_target_properties(xgc2_observer PROPERTIES
  EXPORT_NAME observer
)

target_compile_features(xgc2_observer INTERFACE cxx_std_17)
target_include_directories(xgc2_observer
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(xgc2_observer
  INTERFACE
    Eigen3::Eigen
)

install(TARGETS xgc2_observer
  EXPORT xgc2_observerTargets
)

install(DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES README.md
  DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/libxgc2-observer-dev
)

install(DIRECTORY matlab/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/doc/libxgc2-observer-dev/matlab
)

configure_package_config_file(
  cmake/xgc2_observerConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/xgc2_observerConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xgc2_observer
  PATH_VARS CMAKE_INSTALL_INCLUDEDIR
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/xgc2_observerConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/xgc2_observerConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/xgc2_observerConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xgc2_observer
)

install(EXPORT xgc2_observerTargets
  NAMESPACE xgc2_observer::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/xgc2_observer
)

if(XGC2_OBSERVER_BUILD_TESTING)
  include(CTest)
  if(BUILD_TESTING)
    add_executable(observer_header_test
      test/observer_header_test.cpp
    )
    target_link_libraries(observer_header_test
      PRIVATE
        xgc2_observer::observer
    )
    add_test(NAME observer_header_test COMMAND observer_header_test)
  endif()
endif()
