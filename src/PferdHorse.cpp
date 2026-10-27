#include <targeter/PferdHorse.h>

#include <string>

namespace targeter {

namespace {

MocapLayout BuildLayout() {
    MocapLayout l;
    l.name      = "PFERD horse";
    l.unitScale = 0.7f;   // centimetres -> game units (70 per metre)

    auto mid = [&](std::string name, std::string a, std::string b) {
        l.virtuals.push_back({std::move(name), {{std::move(a), 0.5f}, {std::move(b), 0.5f}}});
    };
    l.virtuals.push_back({"PelvisCentre",
                          {{"L_Pelvis_1_25", 0.25f}, {"R_Pelvis_1_25", 0.25f}, {"L_Pelvis_2_26", 0.25f}, {"R_Pelvis_2_26", 0.25f}}});
    mid("Ischia", "L_Pelvis_2_26", "R_Pelvis_2_26");
    l.virtuals.push_back({"Withers", {{"C_Back_1_10", 1.f / 3.f}, {"L_ShoulderBlade_20", 1.f / 3.f}, {"R_ShoulderBlade_20", 1.f / 3.f}}});
    mid("NeckLow", "L_Neck_3_9", "R_Neck_3_9");
    mid("NeckMid", "L_Neck_2_8", "R_Neck_2_8");
    mid("NeckHigh", "L_Neck_1_7", "R_Neck_1_7");
    mid("Temples", "L_Temple_3", "R_Temple_3");
    mid("Muzzle", "L_Nostril_Out_36", "R_Nostril_Out_36");
    mid("Jaw", "L_Jaw_5", "R_Jaw_5");
    mid("Chin", "L_Chin_6", "R_Chin_6");
    // A hoof's toe and outside markers: its back marker drops out of whole takes.
    for (const char* leg : {"LF", "RF", "LH", "RH"}) {
        const std::string s = leg;
        const char*       n = s[1] == 'F' ? "_71" : "_74";
        const char*       o = s[1] == 'F' ? "_73" : "_76";
        mid(s + "_Hoof", s + "_Hoof_Front" + n, s + "_Hoof_Side" + o);
    }

    auto bone = [&](std::string name, std::string parent, std::string origin, std::string aim, std::string roll,
                    std::string hinge = {}) {
        l.bones.push_back({std::move(name), std::move(parent), std::move(origin), std::move(aim), std::move(roll), std::move(hinge)});
    };
    bone("trunk", "", "PelvisCentre", "Withers", "R_Barrel_6_58");
    bone("pelvis", "trunk", "PelvisCentre", "Ischia", "R_Pelvis_1_25");
    bone("spine1", "trunk", "C_Back_5_14", "C_Back_4_13", "R_Barrel_7_59");
    bone("spine2", "spine1", "C_Back_4_13", "C_Back_3_12", "R_Barrel_6_58");
    bone("spine3", "spine2", "C_Back_3_12", "C_Back_2_11", "R_Barrel_5_57");
    bone("spine4", "spine3", "C_Back_2_11", "C_Back_1_10", "R_Barrel_4_56");
    bone("chest", "spine4", "Withers", "C_Chestpoint_19", "R_Barrel_1_53");
    bone("neck1", "chest", "NeckLow", "NeckMid", "R_Neck_3_9");
    bone("neck2", "neck1", "NeckMid", "NeckHigh", "R_Neck_2_8");
    bone("neck3", "neck2", "NeckHigh", "C_Poll_1", "R_Neck_1_7");
    bone("head", "neck3", "Temples", "Muzzle", "R_Temple_3");
    bone("jaw", "head", "Jaw", "Chin", "R_Jaw_5");
    bone("l_ear", "head", "L_Ear_Base_32", "L_Ear_Top_33", "L_Ear_Side_34");
    bone("r_ear", "head", "R_Ear_Base_32", "R_Ear_Top_33", "R_Ear_Side_34");
    bone("tail1", "pelvis", "C_Tail_1_16", "C_Tail_2_17", "R_Pelvis_2_26");
    bone("tail2", "tail1", "C_Tail_2_17", "C_Tail_3_18", "R_Pelvis_2_26");
    // Legs: a bent joint's plane (hinge) gives the twist; the near-straight forearm, cannon and hock segments take it
    // from a marker off the bone instead, where a hinge would be degenerate.
    for (const char side : {'L', 'R'}) {
        const std::string s(1, side), x = s + "_", f = s + "F_", h = s + "H_", lo(1, char(side + 32));
        bone(lo + "_scapula", "chest", x + "ShoulderBlade_20", x + "ShoulderJoint_21", "", x + "ElbowJoint_22");
        bone(lo + "_humerus", lo + "_scapula", x + "ShoulderJoint_21", x + "ElbowJoint_22", "", x + "CarpalJoint_23");
        bone(lo + "_radius", lo + "_humerus", x + "ElbowJoint_22", x + "CarpalJoint_23", f + "Front_1_47");
        bone(lo + "_metacarpus", lo + "_radius", x + "CarpalJoint_23", f + "FetlockJoint_24", f + "Front_2_48");
        bone(lo + "_fore_pastern", lo + "_metacarpus", f + "FetlockJoint_24", s + "F_Hoof", "", x + "CarpalJoint_23");
        bone(lo + "_fore_hoof", lo + "_fore_pastern", f + "Hoof_Side_73", f + "Hoof_Front_71", f + "FetlockJoint_24");
        bone(lo + "_femur", "pelvis", x + "HipJoint_27", x + "StifleJoint_28", "", x + "HockJoint_30");
        bone(lo + "_tibia", lo + "_femur", x + "StifleJoint_28", x + "HockJoint_30", "", x + "HipJoint_27");
        bone(lo + "_cannon", lo + "_tibia", x + "HockJoint_30", h + "FetlockJoint_31", x + "PointOfHock_29");
        bone(lo + "_hind_pastern", lo + "_cannon", h + "FetlockJoint_31", s + "H_Hoof", "", x + "HockJoint_30");
        bone(lo + "_hind_hoof", lo + "_hind_pastern", h + "Hoof_Side_76", h + "Hoof_Front_74", h + "FetlockJoint_31");
    }
    return l;
}

MocapMap BuildMap() {
    MocapMap m;
    m.name         = "PFERD horse -> Skyrim horse";
    m.root         = "NPC Root [Root]";
    m.anchor       = "Horse_COM";
    m.anchorSource = "trunk";
    auto pair = [&](std::string source, std::string target, std::string aim = {}) {
        m.pairs.push_back({std::move(source), std::move(target), std::move(aim)});
    };
    pair("trunk", "Horse_COM");
    pair("pelvis", "HorsePelvis", "HorseTail1");
    pair("spine1", "HorseSpine1", "HorseSpine2");
    pair("spine2", "HorseSpine2", "HorseSpine3");
    pair("spine3", "HorseSpine3", "HorseSpine4");
    pair("spine4", "HorseSpine4", "HorseRibcage");
    pair("chest", "HorseRibcage");
    pair("neck1", "HorseNeck1", "HorseNeck2");
    pair("neck1", "HorseNeck2", "HorseNeck3");
    pair("neck2", "HorseNeck3", "HorseNeck4");
    pair("neck3", "HorseNeck4", "HorseScull");
    pair("head", "HorseScull");
    pair("jaw", "HorseJaw");
    pair("l_ear", "HorseLEar");
    pair("r_ear", "HorseREar");
    pair("tail1", "HorseTail1", "HorseTail2");
    pair("tail2", "HorseTail2", "HorseTail3");
    pair("tail2", "HorseTail3", "HorseTail4");
    // Skyrim names the left foreleg `HorseL*` and the right one `HorseFrontRLeg*`.
    const struct {
        const char* side;
        const char* fore;
    } legs[] = {{"l", "HorseL"}, {"r", "HorseFrontRLeg"}};
    for (const auto& leg : legs) {
        const std::string s = leg.side, fore = leg.fore, hind = s == "l" ? "HorseL" : "HorseR";
        pair(s + "_scapula", fore + "Scapula", fore + "Humerus");
        pair(s + "_humerus", fore + "Humerus", fore + "Radius");
        pair(s + "_radius", fore + "Radius", fore + "Metacarpus");
        pair(s + "_metacarpus", fore + "Metacarpus", fore + "PhalangesManus");
        pair(s + "_fore_pastern", fore + "PhalangesManus", fore + "FrontHoof");
        pair(s + "_fore_hoof", fore + "FrontHoof");
        pair(s + "_femur", hind + "Femur", hind + "Tibia");
        pair(s + "_tibia", hind + "Tibia", hind + "LargeCannon");
        pair(s + "_cannon", hind + "LargeCannon", hind + "PhalanxPrima");
        pair(s + "_hind_pastern", hind + "PhalanxPrima", hind + "RearHoof");
        pair(s + "_hind_hoof", hind + "RearHoof");
    }
    return m;
}

} // namespace

const MocapLayout& PferdHorseLayout() {
    static const MocapLayout kLayout = BuildLayout();
    return kLayout;
}

const MocapMap& PferdHorseToSkyrim() {
    static const MocapMap kMap = BuildMap();
    return kMap;
}

} // namespace targeter
