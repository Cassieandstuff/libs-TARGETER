# TARGETER bug log

Each entry: symptom, what was ruled out, root cause at `file:line`, status (`OPEN` / `INVESTIGATING` / `FIXED`), and the
test that pins it (`tests/targeter_test.cpp`). Measurements on real clips come from BK's `[.measure]` retarget test
(UEFN Mannequin clips onto Skyrim's human skeleton, `retarget-sources/ue5-manny.yaml`).

## R1 — Twist bones hold their rest: the wrist and upper arm candy-wrap — OPEN

- **Symptom:** a hand that pronates, or an upper arm that rolls, twists the skin at one joint. Skyrim's
  `UpperarmTwist1/2` and `ForearmTwist1/2` stay at rest under their parent.
- **Ruled out:** the maps. No source has twist bones that line up with Skyrim's, so mapping them can't fix it.
- **Root cause:** `src/Retarget.cpp` gave an unmapped bone its rest pose under its parent, with no way to share a
  neighbour's twist.
- **Test:** `TestTwistBonesShareTheTwist`.

## R2 — Planted feet slide — OPEN

- **Symptom:** feet that stand still in the source slide on the target. SCT measured 8.88 u/s on Skyrim, down to
  5.33 u/s with leg IK (`0523893c6`).
- **Ruled out:** the hips' path. It is the source's, scaled by one hip-height ratio.
- **Root cause:** one scale can't match both legs. The target's thigh and calf split their length differently from the
  source's (Skyrim's legs are 0.742 of hip height against 0.710), so the same joint angles put the foot elsewhere.
  `DetectFootPlants` found the plants, but nothing pinned them.
- **Test:** `TestFootLockPinsPlants`.
- **Note:** `RetargetOptions::lockFeet` is off in the library, but BK's Retarget window turns it on by default. Re-baking a
  clip there changes the thighs, calves and feet of its planted frames, compared with a bake made before.

## R3 — The root pitches, rolls and sways — OPEN

- **Symptom:** with a source root, the target root took its full rotation. Without one, the root took the hips'
  heading, so a walk's pelvis sway (8–14° against COM) turned the root, and with it the motion data, back and forth.
- **Root cause:** `src/Retarget.cpp`, the per-frame root branch. It used the full world delta of the source root, or the
  hips' heading frame by frame.
- **Test:** `TestRootIsYawOnlyAndSteady`.

## R4 — Helper bones below the feet set the ground — OPEN

- **Symptom:** Skyrim's `x_NPC` helpers sit at Z = -2. The ground became -2, so the root sank 2 units and the scale
  grew by 2%.
- **Root cause:** `src/Retarget.cpp`. The ground was the lowest joint of the whole skeleton on each side.
- **Test:** `TestGroundIgnoresHelpers`.

## R5 — A coarse source spine kinks the target's — OPEN

- **Symptom:** cmu-daz drives `NPC Spine [Spn0]` and `NPC Spine2 [Spn2]` but not `NPC Spine1 [Spn1]`. The whole bend
  landed on one joint (SCT's chain-mapping de-kink, `8877d14fc`).
- **Root cause:** an unmapped bone between two mapped ones held its rest.
- **Test:** `TestBetweenBonesShareTheBend`.

## R6 — The default aim picks the wrong child; aims on the hips vanish silently — OPEN

- **Symptom:** the first mapped child by index decided the aim. A chest listing a clavicle before the neck aimed at
  the clavicle (SCT ED-10), and a hand aimed at its thumb. Since `af24ae5`, an aim on the root or hips is ignored without
  a report. BK's own test fixture still sets one.
- **Root cause:** `src/Retarget.cpp`, the aim-child search.
- **Test:** `TestDefaultAimFollowsTheChain`.

## R7 — Rest roll differences survive the alignment — OPEN

- **Symptom:** where the two rigs roll a bone differently at rest (a palm, a clavicle), the result keeps that roll and
  looks "blocky" (DCC DESIGN 3.3c). Only a hand-set retarget pose fixed it.
- **Root cause:** the rest alignment is a shortest-arc swing onto the aim child, which leaves roll as it was.
- **Test:** `TestRestRollFollowsTheOtherChildren`.

## R8 — Quaternion tracks change sign between frames — OPEN

- **Symptom:** a root heading taken through `atan2` jumps from +180° to -180°, and its quaternion changes sign from one
  key to the next. Interpolating between the two keys goes the long way round.
- **Root cause:** `src/Retarget.cpp`, the no-root heading. No track was kept in one hemisphere.
- **Test:** `TestTracksStayInOneHemisphere`.

## R9 — Marker mocap has its own transfer, and PFERD never aligns a rest — OPEN

- **Symptom:** `RetargetMocap` copied the transfer with its own root, scale and alignment rules, and got none of the
  fixes above. `PferdHorseToSkyrim` never set `alignRest`, so every horse bone kept the target's rest wherever the rest
  frame's bone pointed.
- **Root cause:** `src/Mocap.cpp` `RetargetMocap`; `src/PferdHorse.cpp` `pair()`.
- **Test:** `TestMocapAlignsRest` (and `TestRetargetMocapSwing` must keep passing).
