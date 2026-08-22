// SPDX-License-Identifier: MIT
// Copyright (c) 2026-present K. S. Ernest (iFire) Lee
#include "mag_calib.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NJOINTS 15

// ── per-joint state ──────────────────────────────────────────────────────────

typedef struct {
    // Result
    float b[3];        // hard iron offset
    float H[3][3];     // soft iron matrix (identity until solved)
    float Bg[3];       // earth field estimate in global frame
    int   valid;       // 1 when calibration has converged (n >= MIN_SAMPLES)

    // Bootstrap accumulator for B_global and A_global (200 diverse-orientation frames)
    float Bg_acc[3];
    int   Bg_n;
    Quat  Bg_q_prev;   // quaternion at last accepted bootstrap sample (rotation gate)
    float Ag_acc[3];   // gravity direction accumulator in global frame
    float Ag[3];       // gravity direction in global frame (normalized)
    int   Ag_set;      // 1 when Ag has been computed

    // Online least-squares accumulators
    // x = [Braw_x, Braw_y, Braw_z, 1]  (4D)
    // y[i] = (R_cleanᵀ · Bg)[i]         (3 scalars)
    // XtX[4][4] = Σ x·xᵀ
    // Xty[3][4] = Σ y[i]·x
    double XtX[4][4];
    double Xty[3][4];
    int    n_total;    // total samples pushed
    int    n_since_solve;

    const char *joint_name;
} JointMagCal;

static JointMagCal g_cal[NJOINTS];
static int         g_inited = 0;

// ── helpers ──────────────────────────────────────────────────────────────────

static void ensure_init(void) {
    if (g_inited) return;
    for (int i = 0; i < NJOINTS; i++) {
        memset(&g_cal[i], 0, sizeof(g_cal[i]));
        // H starts as identity
        g_cal[i].H[0][0] = g_cal[i].H[1][1] = g_cal[i].H[2][2] = 1.f;
    }
    g_inited = 1;
}

void mag_calib_set_joint_name(int ji, const char *name) {
    ensure_init();
    if (ji >= 0 && ji < NJOINTS) g_cal[ji].joint_name = name;
}

// Rotate vector v by quaternion q: v' = q * [0,v] * q†  (standard sandwich)
static void quat_rot_vec(Quat q, const float v[3], float out[3]) {
    float w = q.w, x = q.x, y = q.y, z = q.z;
    float vx = v[0], vy = v[1], vz = v[2];
    // t = 2 * cross(q.xyz, v)
    float tx = 2.f*(y*vz - z*vy);
    float ty = 2.f*(z*vx - x*vz);
    float tz = 2.f*(x*vy - y*vx);
    out[0] = vx + w*tx + y*tz - z*ty;
    out[1] = vy + w*ty + z*tx - x*tz;
    out[2] = vz + w*tz + x*ty - y*tx;
}

// Transpose-rotate: apply q† (conjugate) to v  →  R_cleanᵀ · v
static void quat_invrot_vec(Quat q, const float v[3], float out[3]) {
    Quat qi = {q.w, -q.x, -q.y, -q.z};
    quat_rot_vec(qi, v, out);
}

// Solve 4×4 linear system Ax=b via Gaussian elimination with partial pivoting.
// Returns 1 on success, 0 on singular.
static int solve4(double A[4][4], double b4[4], double x[4]) {
    double M[4][5];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) M[i][j] = A[i][j];
        M[i][4] = b4[i];
    }
    for (int col = 0; col < 4; col++) {
        // partial pivot
        int pivot = col;
        double best = fabs(M[col][col]);
        for (int row = col+1; row < 4; row++) {
            if (fabs(M[row][col]) > best) { best = fabs(M[row][col]); pivot = row; }
        }
        if (best < 1e-12) return 0;
        if (pivot != col) {
            for (int j = 0; j < 5; j++) {
                double tmp = M[col][j]; M[col][j] = M[pivot][j]; M[pivot][j] = tmp;
            }
        }
        double inv = 1.0 / M[col][col];
        for (int row = col+1; row < 4; row++) {
            double f = M[row][col] * inv;
            for (int j = col; j < 5; j++) M[row][j] -= f * M[col][j];
        }
    }
    for (int i = 3; i >= 0; i--) {
        double s = M[i][4];
        for (int j = i+1; j < 4; j++) s -= M[i][j] * x[j];
        x[i] = s / M[i][i];
    }
    return 1;
}

// Re-solve H, b from accumulated XtX / Xty.
static void solve_calib(JointMagCal *c) {
    double A[4][4];
    memcpy(A, c->XtX, sizeof(A));
    for (int row = 0; row < 3; row++) {
        double rhs[4], x[4];
        memcpy(rhs, c->Xty[row], sizeof(rhs));
        if (!solve4(A, rhs, x)) return;
        c->H[row][0] = (float)x[0];
        c->H[row][1] = (float)x[1];
        c->H[row][2] = (float)x[2];
        c->b[row]    = (float)x[3];
    }
    if (c->n_total >= MAG_CALIB_MIN_SAMPLES) c->valid = 1;
}

// ── INI write ────────────────────────────────────────────────────────────────

static void write_ini(const char *path) {
    if (!path || !*path) return;
    // Write all joints to a temp file, then rename.
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    FILE *f = fopen(tmp, "w");
    if (!f) return;
    for (int i = 0; i < NJOINTS; i++) {
        JointMagCal *c = &g_cal[i];
        if (!c->joint_name || c->n_total == 0) continue;
        fprintf(f, "[%s]\n", c->joint_name);
        fprintf(f, "n = %d\n", c->n_total);
        fprintf(f, "b = %.6f %.6f %.6f\n", c->b[0], c->b[1], c->b[2]);
        fprintf(f, "H = %.6f %.6f %.6f  %.6f %.6f %.6f  %.6f %.6f %.6f\n",
                c->H[0][0], c->H[0][1], c->H[0][2],
                c->H[1][0], c->H[1][1], c->H[1][2],
                c->H[2][0], c->H[2][1], c->H[2][2]);
        fprintf(f, "Bg = %.6f %.6f %.6f\n\n", c->Bg[0], c->Bg[1], c->Bg[2]);
    }
    fclose(f);
    rename(tmp, path);
}

// ── INI load ─────────────────────────────────────────────────────────────────

int mag_calib_load(const char *path) {
    ensure_init();
    if (!path || !*path) return 0;
    FILE *f = fopen(path, "r");
    if (!f) return 0;

    JointMagCal *cur = NULL;
    char line[256];
    int loaded = 0;

    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (!*p || *p == '\n' || *p == ';' || *p == '#') continue;

        if (*p == '[') {
            char *end = strchr(p+1, ']');
            if (!end) continue;
            *end = '\0';
            cur = NULL;
            for (int i = 0; i < NJOINTS; i++) {
                if (g_cal[i].joint_name && strcmp(g_cal[i].joint_name, p+1) == 0) {
                    cur = &g_cal[i]; break;
                }
            }
            continue;
        }
        if (!cur) continue;

        char *eq = strchr(p, '=');
        if (!eq) continue;
        *eq = '\0';
        char *val = eq + 1;

        // trim key
        char *kend = eq - 1;
        while (kend > p && (*kend == ' ' || *kend == '\t')) kend--;
        *(kend+1) = '\0';

        if (strcmp(p, "n") == 0) {
            sscanf(val, "%d", &cur->n_total);
        } else if (strcmp(p, "b") == 0) {
            sscanf(val, "%f %f %f", &cur->b[0], &cur->b[1], &cur->b[2]);
        } else if (strcmp(p, "H") == 0) {
            sscanf(val, "%f %f %f %f %f %f %f %f %f",
                   &cur->H[0][0], &cur->H[0][1], &cur->H[0][2],
                   &cur->H[1][0], &cur->H[1][1], &cur->H[1][2],
                   &cur->H[2][0], &cur->H[2][1], &cur->H[2][2]);
        } else if (strcmp(p, "Bg") == 0) {
            sscanf(val, "%f %f %f", &cur->Bg[0], &cur->Bg[1], &cur->Bg[2]);
            cur->Bg_n = 200;   // mark bootstrap done
        }

        if (cur->n_total >= MAG_CALIB_MIN_SAMPLES && cur->Bg_n >= 200) {
            cur->valid = 1;
            loaded++;
        }
    }
    fclose(f);
    return loaded;
}

// ── push ─────────────────────────────────────────────────────────────────────

void mag_calib_push(int ji, Quat q_clean, Accel Braw, Accel araw, const char *ini_path) {
    ensure_init();
    if (ji < 0 || ji >= NJOINTS) return;
    JointMagCal *c = &g_cal[ji];

    float bv[3] = {Braw.x, Braw.y, Braw.z};
    float blen = sqrtf(bv[0]*bv[0] + bv[1]*bv[1] + bv[2]*bv[2]);
    if (blen < 1e-6f) return;   // degenerate reading

    // ── phase 1: bootstrap B_global and A_global ─────────────────────────────
    // Only accumulate bootstrap frames where the sensor has rotated enough from
    // the previous bootstrap frame — this ensures hard iron bias (which is fixed
    // in sensor frame) averages out rather than accumulating coherently.
    if (c->Bg_n < 200) {
        // Require |dot(q_clean, q_prev_bootstrap)| < 0.9998 (~1°) before sampling.
        int sample = 0;
        if (c->Bg_n == 0) {
            sample = 1;
        } else {
            float dw = c->Bg_q_prev.w*q_clean.w + c->Bg_q_prev.x*q_clean.x
                     + c->Bg_q_prev.y*q_clean.y + c->Bg_q_prev.z*q_clean.z;
            if (dw < 0.f) dw = -dw;
            sample = (dw < 0.9998f);
        }
        if (!sample) return;
        c->Bg_q_prev = q_clean;

        // rotate raw mag reading into global frame and accumulate
        float Bg_sample[3];
        quat_rot_vec(q_clean, bv, Bg_sample);
        c->Bg_acc[0] += Bg_sample[0];
        c->Bg_acc[1] += Bg_sample[1];
        c->Bg_acc[2] += Bg_sample[2];

        // rotate raw accel into global frame and accumulate gravity direction
        float av[3] = {araw.x, araw.y, araw.z};
        float Ag_sample[3];
        quat_rot_vec(q_clean, av, Ag_sample);
        c->Ag_acc[0] += Ag_sample[0];
        c->Ag_acc[1] += Ag_sample[1];
        c->Ag_acc[2] += Ag_sample[2];

        c->Bg_n++;
        if (c->Bg_n == 200) {
            float inv = 1.f / 200.f;
            c->Bg[0] = c->Bg_acc[0] * inv;
            c->Bg[1] = c->Bg_acc[1] * inv;
            c->Bg[2] = c->Bg_acc[2] * inv;

            // normalize gravity direction estimate
            float agx = c->Ag_acc[0] * inv;
            float agy = c->Ag_acc[1] * inv;
            float agz = c->Ag_acc[2] * inv;
            float agn = sqrtf(agx*agx + agy*agy + agz*agz);
            if (agn > 1e-6f) {
                c->Ag[0] = agx / agn;
                c->Ag[1] = agy / agn;
                c->Ag[2] = agz / agn;
                c->Ag_set = 1;
            }
        }
        return;   // don't accumulate LS until we have Bg
    }

    // ── phase 2: accumulate least-squares ────────────────────────────────────
    // Expected reading in sensor frame: R_cleanᵀ · Bg
    float y[3];
    quat_invrot_vec(q_clean, c->Bg, y);

    // x = [Braw; 1]
    double x[4] = {bv[0], bv[1], bv[2], 1.0};

    for (int r = 0; r < 4; r++)
        for (int s = 0; s < 4; s++)
            c->XtX[r][s] += x[r] * x[s];

    for (int i = 0; i < 3; i++)
        for (int s = 0; s < 4; s++)
            c->Xty[i][s] += y[i] * x[s];

    c->n_total++;
    c->n_since_solve++;

    if (c->n_since_solve >= MAG_CALIB_SOLVE_EVERY) {
        c->n_since_solve = 0;
        solve_calib(c);
        write_ini(ini_path);
    }
}

// ── get_refs ─────────────────────────────────────────────────────────────────

int mag_calib_get_refs(int ji, float Bg[3], float Ag[3]) {
    ensure_init();
    if (ji < 0 || ji >= NJOINTS) return 0;
    JointMagCal *c = &g_cal[ji];
    if (c->Bg_n < 200 || !c->Ag_set) return 0;
    Bg[0] = c->Bg[0]; Bg[1] = c->Bg[1]; Bg[2] = c->Bg[2];
    Ag[0] = c->Ag[0]; Ag[1] = c->Ag[1]; Ag[2] = c->Ag[2];
    return 1;
}

float mag_calib_get_progress(int ji) {
    ensure_init();
    if (ji < 0 || ji >= NJOINTS) return 0.f;
    JointMagCal *c = &g_cal[ji];
    float bootstrap = (c->Bg_n < 200) ? (c->Bg_n / 200.f) * 0.5f : 0.5f;
    float ls = (c->Bg_n >= 200)
        ? 0.5f * (c->n_total < MAG_CALIB_MIN_SAMPLES
                  ? (float)c->n_total / MAG_CALIB_MIN_SAMPLES
                  : 1.f)
        : 0.f;
    return bootstrap + ls;
}

// ── apply ────────────────────────────────────────────────────────────────────

int mag_calib_apply(int ji, Accel Braw, Accel *Bcal_out) {
    ensure_init();
    if (ji < 0 || ji >= NJOINTS || !g_cal[ji].valid) {
        *Bcal_out = Braw;
        return 0;
    }
    JointMagCal *c = &g_cal[ji];
    float bv[3] = {Braw.x, Braw.y, Braw.z};
    float out[3];
    for (int i = 0; i < 3; i++) {
        out[i] = c->H[i][0]*bv[0] + c->H[i][1]*bv[1] + c->H[i][2]*bv[2] + c->b[i];
    }
    Bcal_out->x = out[0]; Bcal_out->y = out[1]; Bcal_out->z = out[2];
    return 1;
}
