# TARGETER — working notes

C++23 skeletal animation data model and retargeting library. Dependency-free leaf.
Initially built by porting the `dcc-anim` + `dcc-retarget` modules from Dance Card Creator
and the `KimodoSkeletons` from SCT's Skeleton Config Creator.

## Scope

TARGETER provides three layers:

- **Data model** (`Transform`, `Skeleton`, `Clip`): the shared in-memory representation for
  animation data, used by Dance Card Creator, the Scene Editor, and anything else that works
  with skeletal clips. This is the type set POSER will build editing operations on top of.
- **Retargeting** (`BoneMap`, `RetargetClip`): rest-aligned world-delta retargeting from an
  arbitrary source rig onto a target. The algorithm was proved in SCT's Skeleton Config Creator
  and re-implemented here over the data model.
- **Source rigs** (`KimodoSkeletons`): SMPL-X / SOMA30 / G1Skel34 capture-source rig specs
  vendored from localai-org/kimodo.cpp (Apache-2.0). These are the canonical "what kimodo gives
  you" definitions; SCT's Skeleton Config Creator and any future capture pipeline use them.
- **Utilities** (`RootMotion`, `AutoTag`): normalization and tagging operations that work on
  Skeleton + Clip. These are CB-compatible: `AutoTag` ports CB's `hky-tool autotag` thresholds
  so tags authored here match what CB seeds from compiled clips.

## Public surface

- Consumers link `targeter::targeter` and `#include <targeter/Targeter.h>` — nothing else.
  Component headers under `include/targeter/` exist to organize the umbrella, not as direct
  entry points.
- Everything is in `namespace targeter`, except `KimodoSkeletons.h` which uses
  `namespace targeter::kimskel`.
- No external dependency. Math lives in `cpp/Math.h` (private to the library, never installed):
  quaternion and vector operations over `std::array<float, 3/4>`. No glm, no Eigen.
- Never set `CMAKE_MSVC_RUNTIME_LIBRARY`. Inherits from the consumer (static-CRT SKSE plugins
  and dynamic-CRT Qt apps both consume it).

## Conventions

### Math representation

- `std::array<float, 3>` for positions and vectors (x, y, z).
- `std::array<float, 4>` for quaternions (x, y, z, w — Havok convention). The `w` component is
  always last, index [3]. Never deviate; mixing xyzw and wxyz is a silent correctness bug.
- Scales are `std::array<float, 3>` (uniform scale = all three equal to the same value). The
  retargeter assumes scale = 1 throughout; callers that bake scale before feeding a clip in are
  fine.
- `cpp/Math.h` provides: `Vec3`, `Quat`, and all the helpers (`QMul`, `QRotate`, `QInverse`,
  `QNormalize`, `QFromTo`, `QAngleAxis`, `Dot`, `Cross`, `Normalize`, `YawOf`). Keep all math
  using those types; do not promote `std::array` to glm or Eigen anywhere in the library.

### Code style (matches NIFFER/ESPER)

- `#pragma once` on every header.
- Unnamed `namespace { }` for translation-unit-private helpers.
- `[[nodiscard]]` on every function that returns a value the caller must use.
- `std::expected<T, std::string>` for fallible operations (`RetargetClip`); no output-parameter
  + bool pairs, no exceptions.
- `operator==` defaulted on all data types; no hand-written comparators.
- Types are `PascalCase`, functions `PascalCase`, member variables and locals `camelCase`,
  constants `kCamelCase`. Private `cpp/` helpers follow the same rules.
- No comments that restate what the code does; only comments that explain WHY (a constraint,
  a subtle invariant, a cited source). Public headers carry a one-line doc comment per exported
  symbol.

### Axis convention

Z up, character faces +Y, Skyrim game units (unless a specific function says otherwise, e.g.
`KimodoSkeletons` offsets are in metres as the upstream defines them). The retargeter works
entirely in this frame; `BoneMap::sourceLeftHip` / `sourceRightHip` establish facing so the
source's left points to -X before retargeting begins.

### Parent-before-child invariant

`Skeleton::bones` must always have parents at lower indices than their children.
`ComputeModelSpace` and `RetargetClip` both assume this; violating it produces wrong results
silently. Loaders that read skeletons are responsible for sorting.

## Adding a new feature

- New data-model types go in a new header under `include/targeter/` and a new `.cpp` under
  `cpp/`. Add the `#include` to `Targeter.h` (the umbrella). Add a test to `tests/targeter_test.cpp`.
- New utility functions (operations on Skeleton/Clip) follow the `RootMotion` and `AutoTag`
  pattern: a header with `[[nodiscard]]` free functions, an impl, added to the umbrella, tested.
- Never add a dependency. If the math you need isn't in `cpp/Math.h`, add it there. If you need
  a YAML reader, put it in the consumer (DCC has ryml; SCT has its own YAML layer).

## Consumers

- **Dance Card Creator** (`dcc-anim` + `dcc-retarget`): replace both modules with a
  FetchContent pull of TARGETER. The type names are unchanged; only the namespace changes
  (`dcc::anim::` → `targeter::`, `dcc::retarget::` → `targeter::`).
- **Skyrim Content Tools / Scene Editor**: the Skeleton Config Creator's `KimodoSkeletons.h`
  and its bespoke retarget path both move to TARGETER.
- **SKEngine**: no dependency yet, but TARGETER's Skeleton/Clip types are the intended
  foundation for the POSER layer that SKEngine's animation tooling will share.

## Bug log — sync is mandatory

`docs/bugs.md`: symptom, what's ruled out, root cause, exact `file:line`, status
(`OPEN` / `INVESTIGATING` / `FIXED`). Add an entry when you find a bug, flip it to `FIXED`
(with the commit) when you fix one, in the same change as the code.
