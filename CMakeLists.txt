# ── TARGETER ──────────────────────────────────────────────────────────────────
# C++23 skeletal animation data model and retargeting library. Provides:
#   - Transform, Skeleton, Clip: the shared in-memory animation data model
#   - BoneMap + RetargetClip: rest-aligned world-delta retargeting
#   - KimodoSkeletons: SMPL-X / SOMA30 / G1Skel34 capture-source rig specs
#   - RootMotion, AutoTag: animation normalization and tagging utilities
#
# Consumed via FetchContent; consumers link `targeter::targeter` and
# `#include <targeter/Targeter.h>`. Layout:
#   API/targeter/   — public headers (umbrella + components)
#   src/            — implementations; src/MathOps.h is a private helper
#   tests/          — CTest suite
#
# Dependency-free leaf. The MSVC runtime (static /MT vs dynamic /MD) is
# deliberately NOT set here — it inherits CMAKE_MSVC_RUNTIME_LIBRARY from
# the consuming project, so the same source serves SKSE plugins and Qt apps.

cmake_minimum_required(VERSION 3.28)

project(TARGETER
    VERSION 0.1.0
    DESCRIPTION "C++23 skeletal animation data model and retargeting library"
    LANGUAGES CXX)

if(PROJECT_IS_TOP_LEVEL)
    set(CMAKE_CXX_STANDARD 23)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

option(TARGETER_BUILD_TESTS "Build TARGETER tests" ${PROJECT_IS_TOP_LEVEL})

# ── targeter ──────────────────────────────────────────────────────────────────
file(GLOB_RECURSE TARGETER_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE TARGETER_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/API/targeter/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h")

add_library(targeter STATIC ${TARGETER_SOURCES} ${TARGETER_HEADERS})
add_library(targeter::targeter ALIAS targeter)

# PUBLIC = API/targeter/ (the public surface); PRIVATE = src/ (MathOps.h and
# any other internal headers — not installed, not visible to consumers).
target_include_directories(targeter
    PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/API>
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_features(targeter PUBLIC cxx_std_23)

if(MSVC)
    target_compile_options(targeter PRIVATE /W4 /wd4100 /utf-8)
endif()

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${TARGETER_SOURCES} ${TARGETER_HEADERS})

# ── Tests ─────────────────────────────────────────────────────────────────────
if(TARGETER_BUILD_TESTS)
    enable_testing()
    add_executable(targeter-test tests/targeter_test.cpp)
    target_link_libraries(targeter-test PRIVATE targeter::targeter)
    if(MSVC)
        target_compile_options(targeter-test PRIVATE /utf-8)
    endif()
    add_test(NAME targeter-test COMMAND targeter-test)
endif()
