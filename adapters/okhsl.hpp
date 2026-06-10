// SPDX-License-Identifier: MIT
// OKHSL → sRGB, after Björn Ottosson (bottosson.github.io/posts/colorpicker).
// Public API: okhsl::c(h, s, l) returns ftxui::Color::RGB.
// h, s, l are all in [0, 1].  h = hue/360°.
#pragma once
#include <cmath>
#include <algorithm>
#include <cfloat>
#include <ftxui/screen/color.hpp>

namespace okhsl {
namespace detail {

// sRGB gamma encode
static inline float xfer(float x) {
	return x <= 0.0031308f ? 12.92f * x : 1.055f * std::pow(x, 1.f / 2.4f) - 0.055f;
}

// Oklab → linear sRGB
static inline void oklab_to_lrgb(float L, float a, float b, float& r, float& g, float& bl) {
	float l_ = L + 0.3963377774f * a + 0.2158037573f * b;
	float m_ = L - 0.1055613458f * a - 0.0638541728f * b;
	float s_ = L - 0.0894841775f * a - 1.2914855480f * b;
	float l = l_ * l_ * l_, m = m_ * m_ * m_, s = s_ * s_ * s_;
	r = +4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s;
	g = -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s;
	bl = -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s;
}

// Maximum Oklab saturation (S = C/L) for a given hue direction (a,b normalised)
// before any sRGB component clips.  One Halley refinement step.
static float max_sat(float a, float b) {
	float k0, k1, k2, k3, k4, wl, wm, ws;
	if (-1.88170328f * a - 0.80936493f * b > 1.f) {
		k0 = +1.19086277f;
		k1 = +1.76576728f;
		k2 = +0.59662007f;
		k3 = +0.75515197f;
		k4 = +0.56771245f;
		wl = +4.0767416621f;
		wm = -3.3077115913f;
		ws = +0.2309699292f;
	} else if (1.81444104f * a - 1.19445276f * b > 1.f) {
		k0 = +0.73956515f;
		k1 = -0.45954404f;
		k2 = +0.08285427f;
		k3 = +0.12541070f;
		k4 = +0.14503204f;
		wl = -1.2684380046f;
		wm = +2.6097574011f;
		ws = -0.3413193965f;
	} else {
		k0 = +1.35733652f;
		k1 = -0.00915799f;
		k2 = -1.15130210f;
		k3 = -0.50559606f;
		k4 = +0.00692167f;
		wl = -0.0041960863f;
		wm = -0.7034186147f;
		ws = +1.7076147010f;
	}
	float S = k0 + k1 * a + k2 * b + k3 * a * a + k4 * a * b;
	float k_l = +0.3963377774f * a + 0.2158037573f * b;
	float k_m = -0.1055613458f * a - 0.0638541728f * b;
	float k_s = -0.0894841775f * a - 1.2914855480f * b;
	float l_ = 1 + S * k_l, m_ = 1 + S * k_m, s_ = 1 + S * k_s;
	float l = l_ * l_ * l_, m = m_ * m_ * m_, s = s_ * s_ * s_;
	float f = wl * l + wm * m + ws * s;
	float f1 = wl * (3 * k_l * l_ * l_) + wm * (3 * k_m * m_ * m_) + ws * (3 * k_s * s_ * s_);
	float f2 = wl * (6 * k_l * k_l * l_) + wm * (6 * k_m * k_m * m_) + ws * (6 * k_s * k_s * s_);
	return S - f * f1 / (f1 * f1 - 0.5f * f * f2);
}

// Cusp of the sRGB gamut triangle for hue direction (a,b).
static void find_cusp(float a, float b, float& Lc, float& Cc) {
	float Smax = max_sat(a, b);
	float r, g, bl;
	oklab_to_lrgb(1.f, Smax * a, Smax * b, r, g, bl);
	float Lmax = std::cbrt(1.f / std::max({0.001f, r, g, bl}));
	Lc = Lmax;
	Cc = Lmax * Smax;
}

// Gamut boundary along line (L0→L1, 0→C1), one Halley refinement per channel.
static float gamut_intersect(float a, float b, float L1, float C1, float L0, float Lc, float Cc) {
	float t;
	if ((L1 - L0) * Cc - (Lc - L0) * C1 <= 0.f) {
		t = Cc * L0 / (C1 * Lc + Cc * (L0 - L1));
	} else {
		t = Cc * (L0 - 1.f) / (C1 * (Lc - 1.f) + Cc * (L0 - L1));
		float dL = L1 - L0, dC = C1;
		float k_l = +0.3963377774f * a + 0.2158037573f * b;
		float k_m = -0.1055613458f * a - 0.0638541728f * b;
		float k_s = -0.0894841775f * a - 1.2914855480f * b;
		float l_dt = dL + dC * k_l, m_dt = dL + dC * k_m, s_dt = dL + dC * k_s;
		float L = L0 * (1 - t) + t * L1, C = t * C1;
		float l_ = L + C * k_l, m_ = L + C * k_m, s_ = L + C * k_s;
		float l = l_ * l_ * l_, m = m_ * m_ * m_, s = s_ * s_ * s_;
		float ldt = 3 * l_dt * l_ * l_, mdt = 3 * m_dt * m_ * m_, sdt = 3 * s_dt * s_ * s_;
		float ldt2 = 6 * l_dt * l_dt * l_, mdt2 = 6 * m_dt * m_dt * m_, sdt2 = 6 * s_dt * s_dt * s_;
		float t_r, t_g, t_b;
		{
			float f0 = +4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s - 1;
			float f1 = +4.0767416621f * ldt - 3.3077115913f * mdt + 0.2309699292f * sdt;
			float f2 = +4.0767416621f * ldt2 - 3.3077115913f * mdt2 + 0.2309699292f * sdt2;
			float u = f1 / (f1 * f1 - 0.5f * f0 * f2);
			t_r = u >= 0 ? -f0 * u : FLT_MAX;
		}
		{
			float f0 = -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s - 1;
			float f1 = -1.2684380046f * ldt + 2.6097574011f * mdt - 0.3413193965f * sdt;
			float f2 = -1.2684380046f * ldt2 + 2.6097574011f * mdt2 - 0.3413193965f * sdt2;
			float u = f1 / (f1 * f1 - 0.5f * f0 * f2);
			t_g = u >= 0 ? -f0 * u : FLT_MAX;
		}
		{
			float f0 = -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s - 1;
			float f1 = -0.0041960863f * ldt - 0.7034186147f * mdt + 1.7076147010f * sdt;
			float f2 = -0.0041960863f * ldt2 - 0.7034186147f * mdt2 + 1.7076147010f * sdt2;
			float u = f1 / (f1 * f1 - 0.5f * f0 * f2);
			t_b = u >= 0 ? -f0 * u : FLT_MAX;
		}
		t += std::min({t_r, t_g, t_b});
	}
	return t;
}

// Perceptual toe (maps Oklab L to OKHSL l with a soft knee near black).
static inline float toe(float x) {
	constexpr float k1 = 0.206f, k2 = 0.03f, k3 = (1 + k1) / (1 + k2);
	return 0.5f * (k3 * x - k1 + std::sqrt((k3 * x - k1) * (k3 * x - k1) + 4 * k2 * k3 * x));
}
static inline float toe_inv(float x) {
	constexpr float k1 = 0.206f, k2 = 0.03f, k3 = (1 + k1) / (1 + k2);
	return (x * x + k1 * x) / (k3 * (x + k2));
}

}  // namespace detail

// Convert OKHSL (h,s,l ∈ [0,1]) to ftxui::Color using true-colour RGB.
// h = hue/360°.  Results are clamped to the sRGB gamut.
inline ftxui::Color c(float h, float s, float l) {
	using namespace detail;
	if (l >= 1.f) {
		return ftxui::Color::RGB(255, 255, 255);
	}
	if (l <= 0.f) {
		return ftxui::Color::RGB(0, 0, 0);
	}

	float a_ = std::cos(6.28318530f * h);
	float b_ = std::sin(6.28318530f * h);
	float L = toe_inv(l);

	float Lc, Cc;
	find_cusp(a_, b_, Lc, Cc);

	float C_max = gamut_intersect(a_, b_, L, 1.f, L, Lc, Cc);
	float ST_S = Cc / Lc;
	float ST_T = Cc / (1.f - Lc);
	float k = C_max / std::min(L * ST_S, (1.f - L) * ST_T);

	float S_mid =
	    0.11516993f + 1.f / (+7.44778970f + 4.15901240f * b_ - 2.19557347f * a_ +
	                         1.75152436f * a_ * a_ + 1.64850490f * a_ * b_ + 0.82754344f * b_ * b_);
	float T_mid =
	    0.11239642f + 1.f / (+1.61320320f - 0.68124379f * b_ + 0.40370612f * a_ +
	                         0.90148123f * a_ * a_ - 0.27087943f * a_ * b_ + 0.61223990f * b_ * b_);

	float C_mid, C_0;
	{
		float Ca = L * S_mid, Cb = (1 - L) * T_mid;
		C_mid = 0.9f * k *
		        std::sqrt(std::sqrt(1.f / (1.f / (Ca * Ca * Ca * Ca) + 1.f / (Cb * Cb * Cb * Cb))));
	}
	{
		float Ca = L * 0.4f, Cb = (1 - L) * 0.8f;
		C_0 = std::sqrt(1.f / (1.f / (Ca * Ca) + 1.f / (Cb * Cb)));
	}

	constexpr float mid = 0.8f, mid_inv = 1.25f;
	float C;
	if (s < mid) {
		float t = mid_inv * s;
		float k1 = mid * C_0, k2 = 1.f - k1 / C_mid;
		C = t * k1 / (1.f - k2 * t);
	} else {
		float t = (s - mid) / (1.f - mid);
		float k0 = C_mid;
		float k1 = (1.f - mid) * C_mid * C_mid * mid_inv * mid_inv / C_0;
		float k2 = 1.f - k1 / (C_max - C_mid);
		C = k0 + t * k1 / (1.f - k2 * t);
	}

	float r, g, b;
	oklab_to_lrgb(L, C * a_, C * b_, r, g, b);
	return ftxui::Color::RGB((uint8_t)(xfer(std::clamp(r, 0.f, 1.f)) * 255.f + 0.5f),
	                         (uint8_t)(xfer(std::clamp(g, 0.f, 1.f)) * 255.f + 0.5f),
	                         (uint8_t)(xfer(std::clamp(b, 0.f, 1.f)) * 255.f + 0.5f));
}

}  // namespace okhsl
