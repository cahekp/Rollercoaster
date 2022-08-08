#pragma once
#include <UnigineMathLib.h>

// common arguments:
// p0, p1, p2, p3 - control points
// t - time [0, 1]

//////////////////////////////////////////////////////////////////
// Bezier curve
//////////////////////////////////////////////////////////////////

// Bezier curve (quadratic)
Unigine::Math::Vec3 bezierQuad(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, float t);

// Bezier curve (cubic)
Unigine::Math::Vec3 bezierCubic(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t);



//////////////////////////////////////////////////////////////////
// Catmull-Rom spline (Uniform parametrization, alpha = 0)
//////////////////////////////////////////////////////////////////

Unigine::Math::Vec3 catmullRomUniform(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t);

// return first derivative of catmull-rom spline (velocity, tangent)
Unigine::Math::Vec3 catmullRomUniformTangent(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t);

// return second derivative of catmull-rom spline (acceleration, curvature)
Unigine::Math::Vec3 catmullRomUniformCurvature(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t);

// calculate "Frenet Frame" from Catmull-Rom spline
// return transformation matrix of a point (position + rotation)
Unigine::Math::Mat4 catmullRomUniformFrenet(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t);

// calculate "Rotation Minimizing Frame" from Catmull-Rom spline
// return transformation matrix of a point (position + rotation) based on binormal from previous iteration
Unigine::Math::Mat4 catmullRomUniformRMF(const Unigine::Math::Vec3 &prev_binormal, const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t);



//////////////////////////////////////////////////////////////////
// Catmull-Rom spline
// alpha = 0.0: Uniform Parametrization
// alpha = 0.5: Centripetal Parametrization
// alpha = 1.0: Chordal Parametrization
//////////////////////////////////////////////////////////////////

Unigine::Math::Vec3 catmullRom(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t, float alpha = 0.5f);

// return first derivative of catmull-rom spline (velocity, tangent)
Unigine::Math::Vec3 catmullRomTangent(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t, float alpha = 0.5f);

// return second derivative of catmull-rom spline (acceleration, curvature)
Unigine::Math::Vec3 catmullRomCurvature(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t, float alpha = 0.5f);

// calculate "Frenet Frame" from Catmull-Rom spline
// return transformation matrix of a point (position + rotation)
Unigine::Math::Mat4 catmullRomFrenet(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t, float alpha = 0.5f);

// calculate "Rotation Minimizing Frame" from Catmull-Rom spline
// return transformation matrix of a point (position + rotation) based on binormal from previous iteration
Unigine::Math::Mat4 catmullRomRMF(const Unigine::Math::Vec3 &prev_binormal, const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t, float alpha = 0.5f);



//////////////////////////////////////////////////////////////////
// Hermite spline
//////////////////////////////////////////////////////////////////

Unigine::Math::Vec3 hermite(const Unigine::Math::Vec3 &p0, const Unigine::Math::Vec3 &p1, const Unigine::Math::Vec3 &p2, const Unigine::Math::Vec3 &p3, float t);



//////////////////////////////////////////////////////////////////
// Complex Numbers
//////////////////////////////////////////////////////////////////

Unigine::Math::Vec2 complex_mul(const Unigine::Math::Vec2 &c0, const Unigine::Math::Vec2 &c1);



//////////////////////////////////////////////////////////////////
// SQUAD - Spherical Quadrangle Interpolation
//////////////////////////////////////////////////////////////////

// spherical quadratic interpolation
// uses rotation from q1 to q2 (the same as Catmull-Rom), q0 and q3 - control points
// (basically Catmull Rom splines for quaternions, gives C2-smooth rotation curve)
Unigine::Math::quat squad(const Unigine::Math::quat &q0, const Unigine::Math::quat &q1, const Unigine::Math::quat &q2, const Unigine::Math::quat &q3, float t);

Unigine::Math::quat squad2(const Unigine::Math::quat &q0, const Unigine::Math::quat &q1, const Unigine::Math::quat &q2, const Unigine::Math::quat &q3, float t);

//////////////////////////////////////////////////////////////////
// Easing Functions
// https://easings.net/
//////////////////////////////////////////////////////////////////

float easeInQuad(float x);
float easeOutQuad(float x);
float easeInOutQuad(float x);
float easeInCubic(float x);
float easeOutCubic(float x);
float easeInOutCubic(float x);
float easeInQuart(float x);
float easeOutQuart(float x);
float easeInOutQuart(float x);
float easeInQuint(float x);
float easeOutQuint(float x);
float easeInOutQuint(float x);
float easeInExpo(float x);
float easeOutExpo(float x);
float easeInOutExpo(float x);
float easeInSine(float x);
float easeOutSine(float x);
float easeInOutSine(float x);
float easeInCirc(float x);
float easeOutCirc(float x);
float easeInOutCirc(float x);
float easeInBack(float x);
float easeOutBack(float x);
float easeInOutBack(float x);
float easeInElastic(float x);
float easeOutElastic(float x);
float easeInOutElastic(float x);
float easeOutBounce(float x);
float easeInBounce(float x);
float easeInOutBounce(float x);