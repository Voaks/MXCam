// motion.h
//   Velocity and acceleration of a sampled position, by a least-squares quadratic fit over a
//   short time window.
//
//   Finite differences (position delta / frame time, twice) amplify every bit of frame-timing
//   jitter: if the game moves the bike evenly but our timestamps wobble by a millisecond, a
//   two-step difference sees accelerations of tens of g. A fit through a dozen samples averages
//   that out, which is what keeps airborne detection and the neck from twitching.
#pragma once

#include "mathlib.h"

namespace mxcam {

class MotionFit {
public:
    void Reset() { count_ = 0; }

    void Add(double time, Vec3 position)
    {
        if (count_ && time <= time_[head_]) {  // same frame time again: keep the newest position
            pos_[head_] = position;
            return;
        }
        head_ = (head_ + 1) % kMax;
        time_[head_] = time;
        pos_[head_] = position;
        if (count_ < kMax)
            ++count_;
    }

    // Fits the samples of the last `window` seconds (at least 4 when available, never older
    // than kMaxAge). False until there are two samples. With fewer than four, only the
    // velocity is estimated (acceleration 0).
    bool Estimate(double window, Vec3& velocity, Vec3& accel) const
    {
        if (count_ < 2)
            return false;
        const double t0 = time_[head_];
        const Vec3   p0 = pos_[head_];
        int n = 0;
        while (n < count_) {
            const int i = (head_ - n + kMax) % kMax;
            const double age = t0 - time_[i];
            if (age > kMaxAge || (age > window && n >= 4))
                break;
            ++n;
        }
        if (n < 2)
            return false;

        // Times relative to the newest sample, scaled to about [-1, 0] for conditioning.
        const double scale = std::fmax(t0 - time_[(head_ - (n - 1) + kMax) % kMax], 1e-4);
        double s[5] = {};
        Vec3 r0, r1, r2;
        for (int k = 0; k < n; ++k) {
            const int i = (head_ - k + kMax) % kMax;
            const double tau = (time_[i] - t0) / scale;
            const Vec3 y = pos_[i] - p0;
            double p = 1;
            for (int e = 0; e < 5; ++e, p *= tau)
                s[e] += p;
            r0 = r0 + y;
            r1 = r1 + y * tau;
            r2 = r2 + y * (tau * tau);
        }

        if (n < 4) {
            // Straight line: y = a + b tau.
            const double det = s[0] * s[2] - s[1] * s[1];
            if (std::fabs(det) < 1e-12)
                return false;
            velocity = (r1 * s[0] - r0 * s[1]) / det / scale;
            accel = Vec3{};
            return true;
        }

        // Parabola: y = a + b tau + c tau^2, normal equations solved by Cramer's rule.
        const double m[3][3] = { { s[0], s[1], s[2] }, { s[1], s[2], s[3] }, { s[2], s[3], s[4] } };
        const double det = Det3(m);
        if (std::fabs(det) < 1e-12)
            return false;
        const auto Solve = [&](double y0, double y1, double y2, double& b, double& c) {
            const double mb[3][3] = { { s[0], y0, s[2] }, { s[1], y1, s[3] }, { s[2], y2, s[4] } };
            const double mc[3][3] = { { s[0], s[1], y0 }, { s[1], s[2], y1 }, { s[2], s[3], y2 } };
            b = Det3(mb) / det;
            c = Det3(mc) / det;
        };
        double bx, cx, by, cy, bz, cz;
        Solve(r0.x, r1.x, r2.x, bx, cx);
        Solve(r0.y, r1.y, r2.y, by, cy);
        Solve(r0.z, r1.z, r2.z, bz, cz);
        velocity = Vec3{ bx, by, bz } / scale;
        accel    = Vec3{ cx, cy, cz } * (2.0 / (scale * scale));
        return true;
    }

private:
    static double Det3(const double m[3][3])
    {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    static constexpr int    kMax    = 64;
    static constexpr double kMaxAge = 0.3;
    double time_[kMax] = {};
    Vec3   pos_[kMax];
    int    head_  = 0;
    int    count_ = 0;
};

}  // namespace mxcam
