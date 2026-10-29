// mathlib.h
//   Minimal double-precision vector / quaternion / matrix helpers.
//   Matrices are 4x4 and column-major (OpenGL memory layout): row r, column c is m[c*4 + r].
#pragma once

#include <cmath>

namespace mxcam {

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kGravity  = 9.81;

template <class T>
inline T Clamp(T value, T lo, T hi) { return value < lo ? lo : (value > hi ? hi : value); }

// ---------------------------------------------------------------------------
// Vec3
// ---------------------------------------------------------------------------
struct Vec3 {
    double x = 0, y = 0, z = 0;
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
inline Vec3 operator-(Vec3 a) { return { -a.x, -a.y, -a.z }; }
inline Vec3 operator*(Vec3 a, double s) { return { a.x * s, a.y * s, a.z * s }; }
inline Vec3 operator*(double s, Vec3 a) { return a * s; }
inline Vec3 operator/(Vec3 a, double s) { return { a.x / s, a.y / s, a.z / s }; }

inline double Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 Cross(Vec3 a, Vec3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
inline double Length(Vec3 a) { return std::sqrt(Dot(a, a)); }
inline Vec3 Normalize(Vec3 a)
{
    const double length = Length(a);
    return length > 1e-12 ? a / length : Vec3{ 0, 1, 0 };
}

// ---------------------------------------------------------------------------
// Quat - unit quaternion. In this mod it always means "camera-local -> world" orientation.
// ---------------------------------------------------------------------------
struct Quat {
    double x = 0, y = 0, z = 0, w = 1;
};

// a * b applies b first, then a (same order as matrices).
inline Quat operator*(Quat a, Quat b)
{
    return { a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
             a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
             a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
             a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
}

inline double Dot(Quat a, Quat b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

inline Quat Normalize(Quat q)
{
    const double length = std::sqrt(Dot(q, q));
    if (length < 1e-12)
        return {};
    return { q.x / length, q.y / length, q.z / length, q.w / length };
}

inline Quat AxisAngle(Vec3 unitAxis, double radians)
{
    const double s = std::sin(radians * 0.5);
    return { unitAxis.x * s, unitAxis.y * s, unitAxis.z * s, std::cos(radians * 0.5) };
}

inline Vec3 Rotate(Quat q, Vec3 v)
{
    const Vec3 u{ q.x, q.y, q.z };
    const Vec3 t = 2.0 * Cross(u, v);
    return v + q.w * t + Cross(u, t);
}

// Smallest rotation angle (radians) taking a to b.
inline double AngleBetween(Quat a, Quat b)
{
    return 2.0 * std::acos(Clamp(std::fabs(Dot(a, b)), 0.0, 1.0));
}

inline Quat Slerp(Quat a, Quat b, double t)
{
    double d = Dot(a, b);
    if (d < 0) {  // q and -q are the same rotation; take the short way round
        b = { -b.x, -b.y, -b.z, -b.w };
        d = -d;
    }
    if (d > 0.9995) {  // nearly identical: normalised lerp is exact enough and avoids sin(~0)
        return Normalize({ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                           a.z + (b.z - a.z) * t, a.w + (b.w - a.w) * t });
    }
    const double theta = std::acos(d);
    const double s = std::sin(theta);
    const double wa = std::sin((1.0 - t) * theta) / s;
    const double wb = std::sin(t * theta) / s;
    return { a.x * wa + b.x * wb, a.y * wa + b.y * wb, a.z * wa + b.z * wb, a.w * wa + b.w * wb };
}

// Quaternion for the rotation whose columns are the orthonormal axes x, y, z.
inline Quat QuatFromBasis(Vec3 x, Vec3 y, Vec3 z)
{
    const double m00 = x.x, m01 = y.x, m02 = z.x;
    const double m10 = x.y, m11 = y.y, m12 = z.y;
    const double m20 = x.z, m21 = y.z, m22 = z.z;
    const double trace = m00 + m11 + m22;
    Quat q;
    if (trace > 0) {
        const double s = std::sqrt(trace + 1.0) * 2.0;
        q = { (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s };
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
        q = { 0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s };
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
        q = { (m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s };
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
        q = { (m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s };
    }
    return Normalize(q);
}

// ---------------------------------------------------------------------------
// 4x4 column-major matrices
// ---------------------------------------------------------------------------

// out = a * b
inline void MatMul(const double a[16], const double b[16], double out[16])
{
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            double sum = 0;
            for (int k = 0; k < 4; ++k)
                sum += a[k * 4 + r] * b[c * 4 + k];
            out[c * 4 + r] = sum;
        }
    }
}

// Inverse of an affine matrix (bottom row 0,0,0,1). Returns false if it is singular.
inline bool AffineInverse(const double m[16], double out[16])
{
    const double a00 = m[0], a01 = m[4], a02 = m[8];
    const double a10 = m[1], a11 = m[5], a12 = m[9];
    const double a20 = m[2], a21 = m[6], a22 = m[10];

    const double det = a00 * (a11 * a22 - a12 * a21)
                     - a01 * (a10 * a22 - a12 * a20)
                     + a02 * (a10 * a21 - a11 * a20);
    if (std::fabs(det) < 1e-12)
        return false;
    const double inv = 1.0 / det;

    // Inverse of the 3x3 part (adjugate / determinant).
    const double i00 = (a11 * a22 - a12 * a21) * inv;
    const double i01 = (a02 * a21 - a01 * a22) * inv;
    const double i02 = (a01 * a12 - a02 * a11) * inv;
    const double i10 = (a12 * a20 - a10 * a22) * inv;
    const double i11 = (a00 * a22 - a02 * a20) * inv;
    const double i12 = (a02 * a10 - a00 * a12) * inv;
    const double i20 = (a10 * a21 - a11 * a20) * inv;
    const double i21 = (a01 * a20 - a00 * a21) * inv;
    const double i22 = (a00 * a11 - a01 * a10) * inv;

    const double tx = m[12], ty = m[13], tz = m[14];
    out[0] = i00;  out[4] = i01;  out[8]  = i02;  out[12] = -(i00 * tx + i01 * ty + i02 * tz);
    out[1] = i10;  out[5] = i11;  out[9]  = i12;  out[13] = -(i10 * tx + i11 * ty + i12 * tz);
    out[2] = i20;  out[6] = i21;  out[10] = i22;  out[14] = -(i20 * tx + i21 * ty + i22 * tz);
    out[3] = 0;    out[7] = 0;    out[11] = 0;    out[15] = 1;
    return true;
}

}  // namespace mxcam
