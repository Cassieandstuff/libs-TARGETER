# TARGETER

C++23 skeletal animation data model and retargeting library.

## What it provides

| Header | Contents |
|---|---|
| `targeter/Transform.h` | `Transform` — translation, XYZW quaternion, scale |
| `targeter/Skeleton.h` | `Skeleton`, `SkeletonBone`, `Compose`, `ComputeModelSpace` |
| `targeter/Clip.h` | `Clip`, `FrameRange`, `RangeMap`, `AnnotationTrack`, `FloatTrack`, `NormalizeRanges` |
| `targeter/BoneMap.h` | `BoneMap` — target→source bone mapping for retargeting |
| `targeter/Retarget.h` | `RetargetClip` — rest-aligned world-delta retargeting |
| `targeter/RootMotion.h` | `GroundRoot` — remove lateral drift and vertical offset |
| `targeter/AutoTag.h` | `MeasureClip`, `AutoTag` — CB-compatible motion tagging |
| `targeter/KimodoSkeletons.h` | `smplx22`, `soma30`, `g1skel34` rig specs (Apache-2.0, metres) |

Include only `<targeter/Targeter.h>`.

## Conventions

- **Axis**: Z up, character faces +Y, Skyrim game units.
- **Quaternion**: XYZW (`w` at `[3]`). Havok QsTransform layout: scale → rotate → translate.
- **Dependency-free**: no external libraries. Math is in `cpp/MathOps.h` (private).
- **MSVC runtime**: not set — inherits from the consuming project.
- **Parent-before-child**: `Skeleton::bones` must have parents at lower indices.

## Consuming (FetchContent)

```cmake
include(FetchContent)
FetchContent_Declare(TARGETER
    GIT_REPOSITORY https://github.com/your-org/TARGETER.git
    GIT_TAG        main)
FetchContent_MakeAvailable(TARGETER)

target_link_libraries(MyTarget PRIVATE targeter::targeter)
```

Local override during development:

```bash
cmake -DFETCHCONTENT_SOURCE_DIR_TARGETER=D:/Projects/Common/internal/TARGETER ..
```

## Building standalone (tests)

```bash
cmake -B build -DTARGETER_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build
```

## License

GPL-3.0-only. The copyright holder may license under other terms.

`include/targeter/KimodoSkeletons.h` contains data vendored from
[localai-org/kimodo.cpp](https://github.com/localai-org/kimodo.cpp) (Apache-2.0).
