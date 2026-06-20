// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
//
// TRIAD algorithm for sensor→world attitude determination.
// Given two non-collinear reference vectors known in both sensor and world frames,
// construct orthonormal triads in each frame and recover the rotation matrix R
// that maps sensor→world.  Convert to quaternion via Shepperd's method.
#include "triad_calib.h"
#include <math.h>

static float vec3_norm(const float v[3]) {
    return sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static int vec3_normalize(const float v[3], float out[3]) {
    float n = vec3_norm(v);
    if (n < 1e-6f) return 0;
    out[0] = v[0] / n;
    out[1] = v[1] / n;
    out[2] = v[2] / n;
    return 1;
}

static void vec3_cross(const float a[3], const float b[3], float out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}

int triad_compute(const float a_sensor[3], const float m_sensor[3],
                  const float Ag_world[3], const float Bg_world[3],
                  Quat *q_out) {
    // ── World frame triad ─────────────────────────────────────────────────────
    float t1[3], t2[3], t3[3];

    // t1 = normalize(Ag_world) — gravity reference
    if (!vec3_normalize(Ag_world, t1)) return 0;

    // t2 = normalize(cross(Ag_world, Bg_world)) — perpendicular reference
    float cross_wb[3];
    vec3_cross(Ag_world, Bg_world, cross_wb);
    if (!vec3_normalize(cross_wb, t2)) return 0;

    // t3 = cross(t1, t2)
    vec3_cross(t1, t2, t3);

    // ── Sensor frame triad ────────────────────────────────────────────────────
    float s1[3], s2[3], s3[3];

    // s1 = normalize(a_sensor)
    if (!vec3_normalize(a_sensor, s1)) return 0;

    // s2 = normalize(cross(a_sensor, m_sensor))
    float cross_am[3];
    vec3_cross(a_sensor, m_sensor, cross_am);
    if (!vec3_normalize(cross_am, s2)) return 0;

    // s3 = cross(s1, s2)
    vec3_cross(s1, s2, s3);

    // ── Build rotation matrix R = T · Sᵀ ─────────────────────────────────────
    // R maps sensor→world: R · s_i = t_i
    // R[i][j] = t1[i]*s1[j] + t2[i]*s2[j] + t3[i]*s3[j]
    float R[3][3];
    for (int i = 0; i < 3; i++) {
        R[i][0] = t1[i]*s1[0] + t2[i]*s2[0] + t3[i]*s3[0];
        R[i][1] = t1[i]*s1[1] + t2[i]*s2[1] + t3[i]*s3[1];
        R[i][2] = t1[i]*s1[2] + t2[i]*s2[2] + t3[i]*s3[2];
    }

    // ── Convert R to quaternion via Shepperd's method ─────────────────────────
    // Uses same convention as q_align_phase in sinew_serial_host.c
    // temp[0]=x, temp[1]=y, temp[2]=z, temp[3]=w  (Godot/Shepperd convention)
    float trace = R[0][0] + R[1][1] + R[2][2];
    float temp[4];
    if (trace > 0.f) {
        float sv = sqrtf(trace + 1.f);
        temp[3] = sv * 0.5f;
        sv = 0.5f / sv;
        temp[0] = (R[2][1] - R[1][2]) * sv;
        temp[1] = (R[0][2] - R[2][0]) * sv;
        temp[2] = (R[1][0] - R[0][1]) * sv;
    } else {
        int i = (R[0][0] < R[1][1]) ? ((R[1][1] < R[2][2]) ? 2 : 1) : ((R[0][0] < R[2][2]) ? 2 : 0);
        int j = (i + 1) % 3, k = (i + 2) % 3;
        float diag = R[i][i] - R[j][j] - R[k][k] + 1.f;
        if (diag < 0.f) diag = 0.f;
        float sv = sqrtf(diag);
        temp[i] = sv * 0.5f;
        sv = 0.5f / sv;
        temp[3] = (R[k][j] - R[j][k]) * sv;
        temp[j] = (R[j][i] + R[i][j]) * sv;
        temp[k] = (R[k][i] + R[i][k]) * sv;
    }

    // Return as (w, x, y, z)
    float nn = sqrtf(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] + temp[3]*temp[3]);
    if (nn < 1e-9f) return 0;
    q_out->w = temp[3] / nn;
    q_out->x = temp[0] / nn;
    q_out->y = temp[1] / nn;
    q_out->z = temp[2] / nn;
    return 1;
}
