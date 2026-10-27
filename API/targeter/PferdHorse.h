#pragma once
// PFERD horse — the marker layout of the Poses for Equine Research Dataset (Li et al., Scientific Data 2024; Qualisys
// export, 132 anatomical markers, centimetres, Z up) as a MocapLayout, and its map onto Skyrim's horse skeleton
// (meshes/actors/horse/character assets/skeleton.hkx). Marker names are PFERD's labels without the subject prefix
// ("ID2_"), which RetargetMocap detects per take. Only the labelling is held here: no captured data.

#include <targeter/Mocap.h>

namespace targeter {

/// PFERD's markers as a source skeleton: trunk, pelvis, five back segments, chest, neck, head, jaw, ears, tail, and
/// six bones down each leg (scapula/femur to hoof).
[[nodiscard]] const MocapLayout& PferdHorseLayout();

/// PferdHorseLayout's bones onto Skyrim's horse skeleton; travel and heading on `NPC Root [Root]`, height on
/// `Horse_COM`.
[[nodiscard]] const MocapMap& PferdHorseToSkyrim();

} // namespace targeter
