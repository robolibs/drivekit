cmake_minimum_required(VERSION 3.15)

set(project_name navcon)
project(${project_name} VERSION 0.1.6 LANGUAGES CXX)

add_compile_options(-Wall -Wextra -Wpedantic)
cmake_policy(SET CMP0074 NEW)
cmake_policy(SET CMP0135 NEW)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(params
    -Wno-reorder
    -Wno-narrowing
    -Wno-array-bounds
    -Wno-unused-variable
    -Wno-unused-parameter
    -Wno-stringop-overflow
)

string(TOUPPER ${project_name} project_name_upper)
option(${project_name_upper}_BUILD_EXAMPLES "Build examples" OFF)
option(${project_name_upper}_ENABLE_TESTS "Enable tests" OFF)
option(${project_name_upper}_ENABLE_LQR "Enable LQR controller" OFF)
option(${project_name_upper}_ENABLE_MPC "Enable MPC controller" OFF)
include(FetchContent)

# --------------------------------------------------------------------------------------------------
set(ext_deps)

FetchContent_Declare(concord GIT_REPOSITORY https://github.com/robolibs/concord.git GIT_TAG 2.0.2)
FetchContent_MakeAvailable(concord)
list(APPEND ext_deps concord::concord)

FetchContent_Declare(bonsai GIT_REPOSITORY https://github.com/robolibs/bonsai.git GIT_TAG develop)
FetchContent_MakeAvailable(bonsai)
list(APPEND ext_deps bonsai::bonsai)

# Optional dependencies via pkg-config (for LQR, MPC, etc.)
if(${project_name_upper}_ENABLE_LQR OR ${project_name_upper}_ENABLE_MPC)
    if(${project_name_upper}_ENABLE_LQR)
        message(STATUS "LQR controller requested - checking dependencies...")
    endif()
    if(${project_name_upper}_ENABLE_MPC)
        message(STATUS "MPC controller requested - checking dependencies...")
    endif()
    find_package(PkgConfig)
    # Check for Eigen3 (required for LQR and MPC)
    find_package(Eigen3 QUIET)
    if(Eigen3_FOUND)
        message(STATUS "Found Eigen3: ${EIGEN3_VERSION}")
        list(APPEND ext_deps Eigen3::Eigen)
    else()
        message(WARNING "Eigen3 not found - LQR/MPC cannot be enabled")
    endif()

    set(MPC_PACKAGES ipopt cppad)
    foreach(pkg IN LISTS MPC_PACKAGES)
        string(TOUPPER ${pkg} pkg_upper)
        pkg_check_modules(${pkg_upper} ${pkg})
        if(${pkg_upper}_FOUND)
            message(STATUS "Found ${pkg}: ${${pkg_upper}_LIBRARIES}")
            message(STATUS "  Include dirs: ${${pkg_upper}_INCLUDE_DIRS}")
            message(STATUS "  Library dirs: ${${pkg_upper}_LIBRARY_DIRS}")
            include_directories(${${pkg_upper}_INCLUDE_DIRS})
            link_directories(${${pkg_upper}_LIBRARY_DIRS})
            list(APPEND ext_deps ${${pkg_upper}_LIBRARIES})
        else()
            message(WARNING "${pkg} not found - MPC cannot be enabled")
        endif()
    endforeach()

    # Enable LQR if Eigen3 is found
    if(${project_name_upper}_ENABLE_LQR AND Eigen3_FOUND)
        add_definitions(-DHAS_LQR)
        message(STATUS "✓ LQR controller enabled (Eigen3)")
    elseif(${project_name_upper}_ENABLE_LQR)
        message(WARNING "✗ LQR controller disabled - missing Eigen3")
    endif()

    # Enable MPC if all dependencies found
    if(${project_name_upper}_ENABLE_MPC AND Eigen3_FOUND AND IPOPT_FOUND AND CPPAD_FOUND)
        add_definitions(-DHAS_MPC)
        message(STATUS "✓ MPC controller enabled (Eigen3 + ipopt + cppad)")
    elseif(${project_name_upper}_ENABLE_MPC)
        message(WARNING "✗ MPC controller disabled - missing dependencies")
    endif()
else()
    message(STATUS "Advanced controllers disabled (use -D${project_name_upper}_ENABLE_LQR=ON or -D${project_name_upper}_ENABLE_MPC=ON to enable)")
endif()

# --------------------------------------------------------------------------------------------------
# Build library
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)

# Remove optional controller sources if their features are disabled
if(NOT ${project_name_upper}_ENABLE_LQR)
    list(FILTER SOURCES EXCLUDE REGEX ".*/tracking/path/lqr\\.cpp$")
endif()

if(NOT ${project_name_upper}_ENABLE_MPC)
    list(FILTER SOURCES EXCLUDE REGEX ".*/tracking/path/mpc\\.cpp$")
endif()

add_library(${project_name} ${SOURCES})
add_library(${project_name}::${project_name} ALIAS ${project_name})

target_include_directories(${project_name} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${project_name} PRIVATE ${ext_deps})
target_link_libraries(${project_name} PUBLIC ${ext_deps})
target_compile_options(${project_name} PRIVATE ${params})
if(HAS_RERUN OR ${project_name_upper}_BUILD_EXAMPLES)
    find_package(rerun_sdk REQUIRED)
    list(APPEND ext_deps rerun_sdk)
    target_compile_definitions(${project_name} PUBLIC HAS_RERUN)
    target_link_libraries(${project_name} PUBLIC rerun_sdk)
endif()

include_directories(include)

# --------------------------------------------------------------------------------------------------
include(GNUInstallDirs)

# Install headers (excluding optional features that are disabled)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    PATTERN "lqr.hpp" EXCLUDE
    PATTERN "mpc.hpp" EXCLUDE
)

# Install optional headers only if enabled
if(${project_name_upper}_ENABLE_LQR AND Eigen3_FOUND)
    install(FILES include/navcon/tracking/path/lqr.hpp 
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/navcon/tracking/path)
endif()

if(${project_name_upper}_ENABLE_MPC AND Eigen3_FOUND AND IPOPT_FOUND AND CPPAD_FOUND)
    install(FILES include/navcon/tracking/path/mpc.hpp 
            DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/navcon/tracking/path)
endif()

# Install library and export targets
install(TARGETS ${project_name} 
    EXPORT ${project_name}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

#Install the export file so that `find_package(${project_name})` works
install(EXPORT ${project_name}Targets
  FILE ${project_name}Targets.cmake
  NAMESPACE ${project_name}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${project_name}
)

#Also export it for in‐source FetchContent usage
export(EXPORT ${project_name}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${project_name}Targets.cmake"
  NAMESPACE ${project_name}::
)


# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_BUILD_EXAMPLES)
  find_package(spdlog REQUIRED)
  list(APPEND ext_deps spdlog::spdlog)
  
  file(GLOB exe CONFIGURE_DEPENDS examples/*.cpp)

  # ----------------------------------------------
  set(exec_names)
  foreach(src_file IN LISTS exe)
    get_filename_component(exec_name "${src_file}" NAME_WE)
    add_executable(${exec_name} "${src_file}")
    target_compile_definitions(${exec_name} PRIVATE HAS_RERUN)
    target_compile_options(${exec_name} PRIVATE ${params})
    target_link_libraries(${exec_name} ${project_name}::${project_name} ${ext_deps})
    install(TARGETS ${exec_name} DESTINATION bin)
    list(APPEND exec_names ${exec_name})
  endforeach()
  # ----------------------------------------------
  foreach(exec IN LISTS exec_names)
    file(REMOVE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    file(WRITE "${CMAKE_CURRENT_LIST_DIR}/.execs")
    get_filename_component(exec_name "${exec}" NAME)
    file(APPEND "${CMAKE_CURRENT_LIST_DIR}/.execs" "${CMAKE_CURRENT_LIST_DIR}/build/${exec_name}\n")
  endforeach()
  # ----------------------------------------------
  install(TARGETS
    ${exec_names}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()


# --------------------------------------------------------------------------------------------------
if(${project_name_upper}_ENABLE_TESTS)
  enable_testing()
  add_definitions(-DENABLE_DOCTEST_IN_LIBRARY)
  include(FetchContent)
  FetchContent_Declare(DocTest GIT_REPOSITORY "https://github.com/onqtam/doctest")
  FetchContent_MakeAvailable(DocTest)

  file(GLOB test_src CONFIGURE_DEPENDS test/*.cpp)

  foreach(src_file IN LISTS test_src)
    get_filename_component(test_name "${src_file}" NAME_WE)
    add_executable(${test_name} "${src_file}")
    target_compile_options(${test_name} PRIVATE ${params})
    target_link_libraries(${test_name} ${project_name}::${project_name} doctest_with_main)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
