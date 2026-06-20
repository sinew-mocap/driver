// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// Hard-coded hardware-ID → joint mapping (previously loaded from hwid_table.ini).
// To add a new kit: extend g_builtin below and recompile.
#include "hwid_table.h"
#include <string.h>

static const struct { const char *hwid; const char *joint; } g_builtin[] = {
    /* Hips */
    { "603f5e", "Hips" },
    { "abd9b2", "Hips" },
    { "55ea58", "Hips" },
    { "c9e904", "Hips" },
    /* LeftUpperLeg */
    { "588b57", "LeftUpperLeg" },
    { "93f405", "LeftUpperLeg" },
    { "aef2b2", "LeftUpperLeg" },
    { "5e5360", "LeftUpperLeg" },
    /* RightUpperLeg */
    { "598857", "RightUpperLeg" },
    { "1e04f8", "RightUpperLeg" },
    { "57985c", "RightUpperLeg" },
    { "afcdaf", "RightUpperLeg" },
    /* LeftLowerLeg */
    { "ad0fae", "LeftLowerLeg" },
    { "ac07fd", "LeftLowerLeg" },
    { "5b4e63", "LeftLowerLeg" },
    { "583d57", "LeftLowerLeg" },
    /* RightLowerLeg */
    { "593d56", "RightLowerLeg" },
    { "ad5dac", "RightLowerLeg" },
    { "8bfff9", "RightLowerLeg" },
    { "5d4a67", "RightLowerLeg" },
    /* LeftFoot */
    { "5daf5d", "LeftFoot" },
    { "574756", "LeftFoot" },
    { "ad69ad", "LeftFoot" },
    { "111004", "LeftFoot" },
    /* RightFoot */
    { "59d856", "RightFoot" },
    { "65c759", "RightFoot" },
    { "ad96ae", "RightFoot" },
    { "3b09fa", "RightFoot" },
    /* Chest */
    { "42ed05", "Chest" },
    { "5aad55", "Chest" },
    { "628260", "Chest" },
    { "ad04b1", "Chest" },
    /* Head */
    { "a969b5", "Head" },
    { "607c5e", "Head" },
    { "556054", "Head" },
    { "a0f0fd", "Head" },
    /* LeftUpperArm */
    { "54eb58", "LeftUpperArm" },
    { "5e1b64", "LeftUpperArm" },
    { "3ff0fe", "LeftUpperArm" },
    { "aac8b3", "LeftUpperArm" },
    /* RightUpperArm */
    { "6df101", "RightUpperArm" },
    { "61675b", "RightUpperArm" },
    { "adebb5", "RightUpperArm" },
    { "579055", "RightUpperArm" },
    /* LeftLowerArm */
    { "350c04", "LeftLowerArm" },
    { "639e59", "LeftLowerArm" },
    { "ad81ab", "LeftLowerArm" },
    { "589957", "LeftLowerArm" },
    /* RightLowerArm */
    { "5ee156", "RightLowerArm" },
    { "adc1ac", "RightLowerArm" },
    { "573257", "RightLowerArm" },
    { "de0dfa", "RightLowerArm" },
    /* LeftHand */
    { "5e965b", "LeftHand" },
    { "3f0d0a", "LeftHand" },
    { "561357", "LeftHand" },
    { "ad39ad", "LeftHand" },
    /* RightHand */
    { "584c57", "RightHand" },
    { "aeb9ac", "RightHand" },
    { "8a0c07", "RightHand" },
    { "5c8059", "RightHand" },
};

#define BUILTIN_COUNT ((int)(sizeof(g_builtin)/sizeof(g_builtin[0])))

static SinewHwidEntry g_table[BUILTIN_COUNT];
const SinewHwidEntry *SINEW_HWID_TABLE = g_table;
int SINEW_HWID_TABLE_SIZE = 0;

void sinew_hwid_init(void) {
    if (SINEW_HWID_TABLE_SIZE > 0)
        return;
    for (int i = 0; i < BUILTIN_COUNT; i++) {
        g_table[i].hwid  = g_builtin[i].hwid;
        g_table[i].joint = g_builtin[i].joint;
    }
    SINEW_HWID_TABLE_SIZE = BUILTIN_COUNT;
}

// Kept for API compat — always returns BUILTIN_COUNT (ignores path).
int sinew_hwid_load(const char *p_path) {
    (void)p_path;
    sinew_hwid_init();
    return SINEW_HWID_TABLE_SIZE;
}

const char *sinew_joint_for_hwid(const char *p_hwid) {
    if (!p_hwid) return NULL;
    sinew_hwid_init();
    for (int i = 0; i < SINEW_HWID_TABLE_SIZE; i++) {
        if (strcmp(g_table[i].hwid, p_hwid) == 0)
            return g_table[i].joint;
    }
    return NULL;
}
