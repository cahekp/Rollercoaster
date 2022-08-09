#include "Math.h"

using namespace Unigine;
using namespace Math;

Vec3 bezierQuad(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, float t)
{
	float it = 1.0f - t;
	return p0 * (it * it) +
		p1 * (2.0f * it * t) +
		p2 * (t * t);
}

Vec3 bezierCubic(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t)
{
	float tt = t * t;
	float it = 1.0f - t;
	float itt = it * it;
	return p0 * (itt * it) +
		p1 * (3.0f * itt * t) +
		p2 * (3.0f * it * tt) +
		p3 * (tt * t);
}

Vec3 catmullRomUniform(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t)
{
	Vec3 a = p1 * 2.0f;
	Vec3 b = p2 - p0;
	Vec3 c = p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3;
	Vec3 d = -p0 + p1 * 3.0f - p2 * 3.0f + p3;
	return (a + b*t + c*t*t + d*t*t*t) * 0.5f;
}

Vec3 catmullRomUniformTangent(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t)
{
	Vec3 b = p2 - p0;
	Vec3 c = p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3;
	Vec3 d = -p0 + p1 * 3.0f - p2 * 3.0f + p3;
	return b*0.5f + c*t + d*1.5f*t*t;
}

Vec3 catmullRomUniformCurvature(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t)
{
	Vec3 c = p0 * 2.0f - p1 * 5.0f + p2 * 4.0f - p3;
	Vec3 d = -p0 + p1 * 3.0f - p2 * 3.0f + p3;
	return c + d*3.0f*t;
}

Mat4 catmullRomUniformFrenet(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t)
{
	Vec3 pos = catmullRomUniform(p0, p1, p2, p3, t);
	Vec3 vel = catmullRomUniformTangent(p0, p1, p2, p3, t);
	Vec3 accel = catmullRomUniformCurvature(p0, p1, p2, p3, t);

	Vec3 tangent = normalize(vel);
	Vec3 binormal = normalize(cross(vel, accel));
	Vec3 normal = normalize(cross(binormal, tangent));

	Mat4 ret;
	ret.setColumn3(0, binormal);
	ret.setColumn3(1, tangent);
	ret.setColumn3(2, normal);
	ret.setColumn3(3, pos);
	return ret;
}

Mat4 catmullRomUniformRMF(const Vec3 &prev_binormal, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t)
{
	Vec3 pos = catmullRomUniform(p0, p1, p2, p3, t);
	Vec3 vel = catmullRomUniformTangent(p0, p1, p2, p3, t);

	Vec3 tangent = normalize(vel);
	Vec3 normal = normalize(cross(prev_binormal, tangent));
	Vec3 binormal = normalize(cross(tangent, normal));

	Mat4 ret;
	ret.setColumn3(0, binormal);
	ret.setColumn3(1, tangent);
	ret.setColumn3(2, normal);
	ret.setColumn3(3, pos);
	return ret;
}

float getLengthCatmullRomUniform(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, int subdivisions)
{
	float len = 0;
	Vec3 start = catmullRomUniform(p0, p1, p2, p3, 0);
	for (int i = 1; i < subdivisions; i++)
	{
		Vec3 end = catmullRomUniform(p0, p1, p2, p3, float(i) / (subdivisions - 1));
		len += length(end - start);
		start = end;
	}
	return len;
}

Vec3 catmullRom(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t, float alpha)
{
	// knot interval
	auto get_time = [](const Vec3 &p0, const Vec3 &p1, float alpha) -> float
	{
		if (p0 == p1)
			return 1;
		return Math::pow(length2(p1 - p0), 0.5f * alpha);
	};

	float dt0 = get_time(p0, p1, alpha);
	float dt1 = get_time(p1, p2, alpha);
	float dt2 = get_time(p2, p3, alpha);

	Vec3 t1 = ((p1 - p0) / dt0) - ((p2 - p0) / (dt0 + dt1)) + ((p2 - p1) / dt1);
	Vec3 t2 = ((p2 - p1) / dt1) - ((p3 - p1) / (dt1 + dt2)) + ((p3 - p2) / dt2);

	t1 *= dt1;
	t2 *= dt1;

	Vec3 c0 = p1;
	Vec3 c1 = t1;
	Vec3 c2 = (p2 * 3) - (p1 * 3) - (t1 * 2) - t2;
	Vec3 c3 = (p1 * 2) - (p2 * 2) + t1 + t2;
	return c0 + c1*t + c2*t*t + c3*t*t*t;
}

Vec3 catmullRomTangent(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t, float alpha)
{
	// knot interval
	auto get_time = [](const Vec3 &p0, const Vec3 &p1, float alpha) -> float
	{
		if (p0 == p1)
			return 1;
		return Math::pow(length2(p1 - p0), 0.5f * alpha);
	};

	float dt0 = get_time(p0, p1, alpha);
	float dt1 = get_time(p1, p2, alpha);
	float dt2 = get_time(p2, p3, alpha);

	Vec3 t1 = ((p1 - p0) / dt0) - ((p2 - p0) / (dt0 + dt1)) + ((p2 - p1) / dt1);
	Vec3 t2 = ((p2 - p1) / dt1) - ((p3 - p1) / (dt1 + dt2)) + ((p3 - p2) / dt2);

	t1 *= dt1;
	t2 *= dt1;

	// p'(t) = (6t^2 - 6t)p0 + (3t^2 - 4t + 1)m0 + (-6t^2 + 6t)p1 + (3t^2 - 2t)m1
	float tt = t*t;
	return p1 * (6*tt - 6*t) +
		t1 * (3*tt - 4*t + 1) +
		p2 * (-6*tt + 6*t) +
		t2 * (3*tt - 2*t);
}

Vec3 catmullRomCurvature(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t, float alpha)
{
	// knot interval
	auto get_time = [](const Vec3 &p0, const Vec3 &p1, float alpha) -> float
	{
		if (p0 == p1)
			return 1;
		return Math::pow(length2(p1 - p0), 0.5f * alpha);
	};

	float dt0 = get_time(p0, p1, alpha);
	float dt1 = get_time(p1, p2, alpha);
	float dt2 = get_time(p2, p3, alpha);

	Vec3 t1 = ((p1 - p0) / dt0) - ((p2 - p0) / (dt0 + dt1)) + ((p2 - p1) / dt1);
	Vec3 t2 = ((p2 - p1) / dt1) - ((p3 - p1) / (dt1 + dt2)) + ((p3 - p2) / dt2);

	t1 *= dt1;
	t2 *= dt1;

	// p''(t) = (12t - 6)p0 + (6t - 4)m0 + (-12t + 6)p1 + (6t - 2)m1
	return p1 * (12*t - 6) +
		t1 * (6*t - 4) +
		p2 * (-12*t + 6) +
		t2 * (6*t - 2);
}

Mat4 catmullRomFrenet(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t, float alpha)
{
	Vec3 pos = catmullRom(p0, p1, p2, p3, t, alpha);
	Vec3 vel = catmullRomTangent(p0, p1, p2, p3, t, alpha);
	Vec3 accel = catmullRomCurvature(p0, p1, p2, p3, t, alpha);

	Vec3 tangent = normalize(vel);
	Vec3 binormal = normalize(cross(vel, accel));
	Vec3 normal = normalize(cross(binormal, tangent));

	Mat4 ret;
	ret.setColumn3(0, binormal);
	ret.setColumn3(1, tangent);
	ret.setColumn3(2, normal);
	ret.setColumn3(3, pos);
	return ret;
}

Mat4 catmullRomRMF(const Unigine::Math::Vec3 &prev_binormal, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t, float alpha)
{
	Vec3 pos = catmullRom(p0, p1, p2, p3, t, alpha);
	Vec3 vel = catmullRomTangent(p0, p1, p2, p3, t, alpha);

	Vec3 tangent = normalize(vel);
	Vec3 normal = normalize(cross(prev_binormal, tangent));
	Vec3 binormal = normalize(cross(tangent, normal));

	Mat4 ret;
	ret.setColumn3(0, binormal);
	ret.setColumn3(1, tangent);
	ret.setColumn3(2, normal);
	ret.setColumn3(3, pos);
	return ret;
}

float getLengthCatmullRom(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float alpha, int subdivisions)
{
	float len = 0;
	Vec3 start = catmullRom(p0, p1, p2, p3, 0, alpha);
	for (int i = 1; i < subdivisions; i++)
	{
		Vec3 end = catmullRom(p0, p1, p2, p3, float(i) / (subdivisions - 1), alpha);
		len += length(end - start);
		start = end;
	}
	return len;
}

Vec3 hermite(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &p3, float t)
{
	float t2 = t * t;
	float t3 = t2 * t;

	float tension = 0.5f; // 0.5 equivalent a catmull-rom

	vec3 pp1 = (p2 - p0) * tension;
	vec3 pp2 = (p3 - p1) * tension;

	float blend1 = 2.0f * t3 - 3.0f * t2 + 1.0f;
	float blend2 = -2.0f * t3 + 3.0f * t2;
	float blend3 = t3 - 2.0f * t2 + t;
	float blend4 = t3 - t2;

	return p1 * blend1 + p2 * blend2 + pp1 * blend3 + pp2 * blend4;
}

Vec2 complex_mul(const Vec2 &c0, const Vec2 &c1)
{
	// (x + yi)(u + vi) =
	//     xu + xv(i) + yu(i) + yv(i*i) = 
	//     xu + (xv + yu)i - yv =
	// (xu - yv) + (xv + yu)i
	return Vec2(
		c0.x * c1.x - c0.y * c1.y,
		c0.x * c1.y + c0.y * c1.x
	);
}

quat squad(const quat &q0, const quat &q1, const quat &q2, const quat &q3, float t)
{
	auto qexp = [](const quat &q)
	{
		float a = Math::fsqrt(q.x*q.x + q.y*q.y + q.z*q.z);
		float sin_a = Math::sin(a);
		vec4 r;
		r.w = Math::cos(a);
		if (Math::abs(sin_a) >= 1.0e-15)
		{
			float coeff = sin_a / a;
			r.x = q.x * coeff;
			r.y = q.y * coeff;
			r.z = q.z * coeff;
		}
		return quat(r);
	};
	auto qlog = [](const quat &q)
	{
		float a = Math::acos(q.w);
		float sin_a = Math::sin(a);
		if (Math::abs(sin_a) >= 1.0e-15)
		{
			float coeff = a / sin_a;
			vec4 r = vec4(
				q.x * coeff,
				q.y * coeff,
				q.z * coeff,
				0
			);
			return quat(r);
		}
		return quat(vec4_zero);
	};
	auto slerp_no_invert = [](const quat &q0, const quat &q1, float t)
	{
		float d = dot(q0, q1);
		if (Math::abs(d) > 0.9999f)
			return q0;

		float theta = Math::acos(d);
		float sin_t = 1.0f / Math::sin(theta);
		float t0 = Math::sin((1.0f - t) * theta) * sin_t;
		float t1 = Math::sin(t * theta) * sin_t;
		return (q0 * t0 + q1 * t1);//.normalizeValid();
	};

	// modify quaternions for shortest path
	auto get_length = [](const quat &q) { return Math::fsqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w); };
	quat q0m = get_length(q1 - q0) < get_length(q1 + q0) ? q0 : -q0;
	quat q2m = get_length(q1 - q2) < get_length(q1 + q2) ? q2 : -q2;
	quat q3m = get_length(q2m - q3) < get_length(q2m + q3) ? q3 : -q3;

	// calculate helper quaternions (tangent values)
	// https://www.geometrictools.com/Documentation/Quaternions.pdf (page 9 [31])
	quat q1inv = inverse(q1);
	quat q2inv = inverse(q2m);
	quat a = (q1 * qexp((qlog(q1inv * q2m) + qlog(q1inv * q0m)) * -0.25f));
	quat b = (q2m * qexp((qlog(q2inv * q3m) + qlog(q2inv * q1)) * -0.25f));

	return slerp(slerp(q1, q2m, t), slerp(a, b, t), 2.0f * t * (1.0f - t)).normalizeValid();
}

quat squad_v2(const quat &q0, const quat &q1, const quat &q2, const quat &q3, float t)
{
	auto get_squad_intermediate = [](const quat &q0, const quat &q1, const quat &q2) -> quat
	{
		auto get_quat_conjugate = [](const quat &q) -> quat
		{
			quat ret;
			ret.x = -q.x;
			ret.y = -q.y;
			ret.z = -q.z;
			ret.w = q.w;
			return ret;
		};

		// logarithm of a unit quaternion
		// (the result is not necessary a unit quaternion)
		auto get_quat_log = [](const quat &q) -> quat
		{
			quat res = q;
			res.w = 0;

			if (Math::abs(q.w) < 1.0f)
			{
				float theta = Math::acos(q.w);
				float sin_theta = Math::sin(theta);

				if (Math::abs(sin_theta) > 0.0001f)
				{
					float coef = theta / sin_theta;
					res.x = q.x * coef;
					res.y = q.y * coef;
					res.z = q.z * coef;
				}
			}

			return res;
		};

		auto get_quat_exp = [](const quat &q) -> quat
		{
			quat res = q;
			float f_angle = Math::fsqrt(q.x*q.x + q.y*q.y + q.z*q.z);
			float f_sin = Math::sin(f_angle);

			res.w = Math::cos(f_angle);

			if (Math::abs(f_sin) > 0.0001f)
			{
				float coef = f_sin / f_angle;
				res.x = coef*q.x;
				res.y = coef*q.y;
				res.z = coef*q.z;
			}

			return res;
		};

		quat q1Inv = get_quat_conjugate(q1);
		quat p0 = get_quat_log(q1Inv*q0);
		quat p2 = get_quat_log(q1Inv*q2);
		quat sum;
		sum.x = -0.25f * (p0.x+p2.x);
		sum.y = -0.25f * (p0.y+p2.y);
		sum.z = -0.25f * (p0.z+p2.z);
		sum.w = -0.25f * (p0.w+p2.w);
		return q1 * get_quat_exp(sum);
	};

	auto get_quat_squad = [](const quat &q0, const quat &q1, const quat &a0, const quat &a1, float t) -> quat
	{
		auto slerp_no_invert = [](const quat &p, const quat &q, float t) -> quat
		{
			quat ret;
			float f_cos = dot(p, q);
			float f_coeff0 = 1.0f - t;
			float f_coeff1 = t;

			if ((1.0f + f_cos) > 0.00001f)
			{
				if ((1.0f - f_cos) > 0.00001f)
				{
					float omega = Math::acos(f_cos);
					float inv_sin = 1.0f / Math::sin(omega);
					f_coeff0 = Math::sin((1.0f - t) * omega) * inv_sin;
					f_coeff1 = Math::sin(t * omega) * inv_sin;
				}
				ret.x = f_coeff0 * p.x + f_coeff1 * q.x;
				ret.y = f_coeff0 * p.y + f_coeff1 * q.y;
				ret.z = f_coeff0 * p.z + f_coeff1 * q.z;
				ret.w = f_coeff0 * p.w + f_coeff1 * q.w;
			}
			else
			{
				f_coeff0 = Math::sin((1.0f - t) * Consts::PI * 0.5f);
				f_coeff1 = Math::sin(t * Consts::PI * 0.5f);

				ret.x = f_coeff0 * p.x - f_coeff1 * p.y;
				ret.y = f_coeff0 * p.y + f_coeff1 * p.x;
				ret.z = f_coeff0 * p.z - f_coeff1 * p.w;
				ret.w =  p.z;
			}

			return ret;
		};

		float slerp_t = 2.0f * t * (1.0f - t);
		quat slerp_p = slerp_no_invert(q0, q1, t);
		quat slerp_q = slerp_no_invert(a0, a1, t);
		return slerp_no_invert(slerp_p, slerp_q, slerp_t);
	};

	quat t1 = get_squad_intermediate(q0, q1, q2);
	quat t2 = get_squad_intermediate(q1, q2, q3);
	return get_quat_squad(q1, q2, t1, t2, t);
}

float easeInQuad(float x)
{
	return x * x;
}

float easeOutQuad(float x)
{
	float t = 1 - x;
	return 1 - t * t;
}

float easeInOutQuad(float x)
{
	return x < 0.5f ? 2 * x * x : 1 - Math::pow(-2 * x + 2, 2) / 2;
}

float easeInCubic(float x)
{
	return x * x * x;
}

float easeOutCubic(float x)
{
	return 1 - Math::pow(1 - x, 3);
}

float easeInOutCubic(float x)
{
	return x < 0.5f ? 4 * x * x * x : 1 - Math::pow(-2 * x + 2, 3) / 2;
}

float easeInQuart(float x)
{
	return x * x * x * x;
}

float easeOutQuart(float x)
{
	return 1 - Math::pow(1 - x, 4);
}

float easeInOutQuart(float x)
{
	return x < 0.5f ? 8 * x * x * x * x : 1 - Math::pow(-2 * x + 2, 4) / 2;
}

float easeInQuint(float x)
{
	return x * x * x * x * x;
}

float easeOutQuint(float x)
{
	return 1 - Math::pow(1 - x, 5);
}

float easeInOutQuint(float x)
{
	return x < 0.5f ? 16 * x * x * x * x * x : 1 - Math::pow(-2 * x + 2, 5) / 2;
}

float easeInExpo(float x)
{
	return x == 0 ? 0 : Math::pow(2, 10 * x - 10);
}

float easeOutExpo(float x)
{
	return x == 1 ? 1 : 1 - Math::pow(2, -10 * x);
}

float easeInOutExpo(float x)
{
	return x == 0
		? 0
		: x == 1
		? 1
		: x < 0.5f ? Math::pow(2, 20 * x - 10) / 2
		: (2 - Math::pow(2, -20 * x + 10)) / 2;
}

float easeInSine(float x)
{
	return 1 - Math::cos((x * Consts::PI) / 2);
}

float easeOutSine(float x)
{
	return Math::sin((x * Consts::PI) / 2);
}

float easeInOutSine(float x)
{
	return -(Math::cos(Consts::PI * x) - 1) / 2;
}

float easeInCirc(float x)
{
	return 1 - Math::fsqrt(1 - Math::pow(x, 2));
}

float easeOutCirc(float x)
{
	return Math::fsqrt(1 - Math::pow(x - 1, 2));
}

float easeInOutCirc(float x)
{
	return x < 0.5f
		? (1 - Math::fsqrt(1 - Math::pow(2 * x, 2))) / 2
		: (Math::fsqrt(1 - Math::pow(-2 * x + 2, 2)) + 1) / 2;
}

float easeInBack(float x)
{
	const float c1 = 1.70158f;
	const float c3 = c1 + 1;
	return c3 * x * x * x - c1 * x * x;
}

float easeOutBack(float x)
{
	const float c1 = 1.70158f;
	const float c3 = c1 + 1;
	return 1 + c3 * Math::pow(x - 1, 3) + c1 * Math::pow(x - 1, 2);
}

float easeInOutBack(float x)
{
	const float c1 = 1.70158f;
	const float c2 = c1 * 1.525f;
	return x < 0.5f
		? (Math::pow(2 * x, 2) * ((c2 + 1) * 2 * x - c2)) / 2
		: (Math::pow(2 * x - 2, 2) * ((c2 + 1) * (x * 2 - 2) + c2) + 2) / 2;
}

float easeInElastic(float x)
{
	const float c4 = (2 * Consts::PI) / 3;
	return x == 0
		? 0
		: x == 1
		? 1
		: -Math::pow(2, 10 * x - 10) * Math::sin((x * 10 - 10.75f) * c4);
}

float easeOutElastic(float x)
{
	const float c4 = (2 * Consts::PI) / 3;
	return x == 0
		? 0
		: x == 1
		? 1
		: Math::pow(2, -10 * x) * Math::sin((x * 10 - 0.75f) * c4) + 1;
}

float easeInOutElastic(float x)
{
	const float c5 = (2 * Consts::PI) / 4.5f;
	return x == 0
		? 0
		: x == 1
		? 1
		: x < 0.5f
		? -(Math::pow(2, 20 * x - 10) * Math::sin((20 * x - 11.125f) * c5)) / 2
		: (Math::pow(2, -20 * x + 10) * Math::sin((20 * x - 11.125f) * c5)) / 2 + 1;
}

float easeOutBounce(float x)
{
	const float n1 = 7.5625f;
	const float d1 = 2.75f;
	if (x < 1 / d1)
		return n1 * x * x;
	else if (x < 2 / d1)
		return n1 * (x -= 1.5f / d1) * x + 0.75f;
	else if (x < 2.5f / d1)
		return n1 * (x -= 2.25f / d1) * x + 0.9375f;
	else
		return n1 * (x -= 2.625f / d1) * x + 0.984375f;
}

float easeInBounce(float x)
{
	return 1 - easeOutBounce(1 - x);
}

float easeInOutBounce(float x)
{
	return x < 0.5f
		? (1 - easeOutBounce(1 - 2 * x)) / 2
		: (1 + easeOutBounce(2 * x - 1)) / 2;
}