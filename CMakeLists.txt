cmake_minimum_required(VERSION 3.22)
project(drone_sim VERSION 2.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# --------------------------------------------------------------------------
# Options
# --------------------------------------------------------------------------
option(DRONE_SIM_ASAN    "Enable AddressSanitizer"        OFF)
option(DRONE_SIM_PROFILE "Enable profiling flags"          OFF)
option(DRONE_SIM_TESTS   "Build unit test suite"           OFF)
# OFF by default on purpose: -march=native / AVX2 bakes in the *build*
# machine's CPU and makes redistributable binaries crash (SIGILL) on
# older/different CPUs. Only turn this on for a private, single-machine build.
option(DRONE_SIM_NATIVE  "Optimise for the build machine's CPU (NOT portable)" OFF)
# Reference-determinism build: disables fast-math so trajectories are
# reproducible across machines/architectures (slower; for benchmark runs).
option(DRONE_SIM_DETERMINISTIC "Strict FP for reproducible physics" OFF)

set(GODOTCPP_GIT_TAG "godot-4.3-stable" CACHE STRING
    "godot-cpp branch/tag to fetch (match your Godot editor version)")

# --------------------------------------------------------------------------
# godot-cpp: prefer a vendored copy, otherwise fetch it automatically.
# No git submodule required -> a plain `git clone` (or a zip download) works.
# --------------------------------------------------------------------------
set(GODOT_CPP_LOCAL "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/godot-cpp")
if(EXISTS "${GODOT_CPP_LOCAL}/CMakeLists.txt")
  message(STATUS "[drone_sim] Using vendored godot-cpp at ${GODOT_CPP_LOCAL}")
  add_subdirectory("${GODOT_CPP_LOCAL}" godot-cpp)
else()
  message(STATUS "[drone_sim] Fetching godot-cpp (${GODOTCPP_GIT_TAG})...")
  include(FetchContent)
  FetchContent_Declare(
    godot-cpp
    GIT_REPOSITORY https://github.com/godotengine/godot-cpp.git
    GIT_TAG        ${GODOTCPP_GIT_TAG}
    GIT_SHALLOW    ON
  )
  FetchContent_MakeAvailable(godot-cpp)
endif()

# --------------------------------------------------------------------------
# Sources (aero/control/sensors are header-only)
# --------------------------------------------------------------------------
file(GLOB_RECURSE DRONE_SIM_SOURCES CONFIGURE_DEPENDS "src/*.cpp")

add_library(drone_sim SHARED ${DRONE_SIM_SOURCES})

target_include_directories(drone_sim PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_link_libraries(drone_sim PRIVATE godot-cpp)

# --------------------------------------------------------------------------
# Compile flags. -ffast-math / /fp:fast are safe (FP semantics, not ISA).
# CPU-specific instruction selection is gated behind DRONE_SIM_NATIVE.
# --------------------------------------------------------------------------
target_compile_options(drone_sim PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:
    -Wall -Wextra -Wpedantic $<$<NOT:$<BOOL:${DRONE_SIM_DETERMINISTIC}>>:-ffast-math>
    $<$<CONFIG:Release>:-O3 -DNDEBUG>
    $<$<CONFIG:Debug>:-O0 -g3>
  >
  $<$<CXX_COMPILER_ID:MSVC>:
    /W4 $<IF:$<BOOL:${DRONE_SIM_DETERMINISTIC}>,/fp:strict,/fp:fast> /EHsc
    $<$<CONFIG:Release>:/O2 /DNDEBUG>
    $<$<CONFIG:Debug>:/Od /Zi>
  >
)

if(DRONE_SIM_DETERMINISTIC)
  message(STATUS "[drone_sim] Reference-determinism build (fast-math disabled).")
  target_compile_options(drone_sim PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-ffp-contract=off -fno-fast-math>)
  target_compile_definitions(drone_sim PRIVATE DRONE_SIM_DETERMINISTIC=1)
endif()

if(DRONE_SIM_NATIVE)
  message(WARNING "[drone_sim] DRONE_SIM_NATIVE=ON -> binary is NOT portable.")
  target_compile_options(drone_sim PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-march=native>
    $<$<CXX_COMPILER_ID:MSVC>:/arch:AVX2>)
endif()

if(DRONE_SIM_ASAN)
  target_compile_options(drone_sim PRIVATE -fsanitize=address,undefined)
  target_link_options(drone_sim PRIVATE -fsanitize=address,undefined)
endif()

if(DRONE_SIM_PROFILE)
  target_compile_options(drone_sim PRIVATE -pg -fno-omit-frame-pointer)
endif()

# --------------------------------------------------------------------------
# Output naming: (lib)drone_sim.<platform>.<cfg>.<arch>.<ext>
# Detect the real target arch instead of hardcoding x86_64, so Apple Silicon
# and ARM Linux builds are labelled correctly for the .gdextension file.
# --------------------------------------------------------------------------
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _proc)
if(_proc MATCHES "arm64|aarch64")
  set(ARCH_TAG "arm64")
elseif(_proc MATCHES "x86_64|amd64|x64")
  set(ARCH_TAG "x86_64")
else()
  set(ARCH_TAG "${_proc}")
endif()

if(WIN32)
  set(PLATFORM_TAG "windows")
  set_target_properties(drone_sim PROPERTIES SUFFIX ".dll" PREFIX "")
elseif(APPLE)
  set(PLATFORM_TAG "macos")
  set_target_properties(drone_sim PROPERTIES SUFFIX ".dylib" PREFIX "lib")
  # Universal binary when no single arch was forced.
  if(NOT CMAKE_OSX_ARCHITECTURES)
    set(ARCH_TAG "universal")
    set_target_properties(drone_sim PROPERTIES OSX_ARCHITECTURES "x86_64;arm64")
  endif()
else()
  set(PLATFORM_TAG "linux")
  set_target_properties(drone_sim PROPERTIES SUFFIX ".so" PREFIX "lib")
endif()

string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TAG)
if(NOT BUILD_TAG)
  set(BUILD_TAG "release")
endif()

set_target_properties(drone_sim PROPERTIES
  OUTPUT_NAME "drone_sim.${PLATFORM_TAG}.${BUILD_TAG}.${ARCH_TAG}"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/gdextension/bin"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/gdextension/bin"
)

message(STATUS "[drone_sim] Target: ${PLATFORM_TAG}.${BUILD_TAG}.${ARCH_TAG}")

if(DRONE_SIM_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
