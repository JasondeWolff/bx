#pragma once

#include "bx/engine/core/byte_types.hpp"

#include <cmath>

#define SAFE_DIV_EPSILON 0.00001

namespace Math
{
	template <typename T>
	static T FMod(const T& a, const T& b)
	{
		return std::fmod(a, b);
	}

	template <typename T>
	static T Max(const T& a, const T& b)
	{
		return (a > b) ? a : b;
	}

	template <typename T>
	static T Min(const T& a, const T& b)
	{
		return (a < b) ? a : b;
	}

	template <typename T>
	static T Clamp(const T& x, const T& min, const T& max)
	{
		return Max(Min(x, max), min);
	}

	template <typename T>
	static T Lerp(const T& a, const T& b, f32 t)
	{
		return a + (b - a) * t;
	}

	template <typename T>
	static T Pow2(const T& x)
	{
		return x * x;
	}

	template <typename T>
	static T Pow3(const T& x)
	{
		return Pow2(x) * x;
	}

	template <typename T>
	static T Pow4(const T& x)
	{
		T x2 = Pow2(x);
		return Pow2(x2);
	}

	template <typename T>
	static T Pow5(const T& x)
	{
		return Pow4(x) * x;
	}
}

struct Vec2
{
	Vec2() : data{ 0, 0 } {}
	Vec2(f32 x, f32 y)
		: data{ x, y }
	{}

	union
	{
		f32 data[2];
		struct { f32 x, y; };
	};

	f32 At(i32 i) const;
	inline f32& operator[](i32 i) { return data[i]; }
	inline const f32& operator[](i32 i) const { return data[i]; }

	f32 SqrMagnitude() const;
	f32 Magnitude() const;
	Vec2 Normalized() const;
	Vec2 Abs() const;

	void Set(f32 x, f32 y);

	Vec2 PlusF32(f32 rhs) const;
	Vec2 Plus(const Vec2& rhs) const;
	inline Vec2 operator+(const f32& rhs) const { return PlusF32(rhs); }
	inline Vec2& operator+=(const f32& rhs) { *this = *this + rhs; return *this; }
	inline Vec2 operator+(const Vec2& rhs) const { return Plus(rhs); }
	inline Vec2& operator+=(const Vec2& rhs) { *this = *this + rhs; return *this; }

	Vec2 Negate() const;
	inline Vec2 operator-() const { return Negate(); }

	Vec2 MinusF32(f32 rhs) const;
	Vec2 Minus(const Vec2& rhs) const;
	inline Vec2 operator-(const f32& rhs) const { return MinusF32(rhs); }
	inline Vec2& operator-=(const f32& rhs) { *this = *this - rhs; return *this; }
	inline Vec2 operator-(const Vec2& rhs) const { return Minus(rhs); }
	inline Vec2& operator-=(const Vec2& rhs) { *this = *this - rhs; return *this; }

	Vec2 MulF32(f32 rhs) const;
	Vec2 Mul(const Vec2& rhs) const;
	inline Vec2 operator*(const f32& rhs) const { return MulF32(rhs); }
	inline Vec2& operator*=(const f32& rhs) { *this = *this * rhs; return *this; }
	inline Vec2 operator*(const Vec2& rhs) const { return Mul(rhs); }
	inline Vec2& operator*=(const Vec2& rhs) { *this = *this * rhs; return *this; }

	Vec2 DivF32(f32 rhs) const;
	Vec2 Div(const Vec2& rhs) const;
	inline Vec2 operator/(const f32& rhs) const { return DivF32(rhs); }
	inline Vec2& operator/=(const f32& rhs) { *this = *this / rhs; return *this; }
	inline Vec2 operator/(const Vec2& rhs) const { return Div(rhs); }
	inline Vec2& operator/=(const Vec2& rhs) { *this = *this / rhs; return *this; }

	static Vec2 Splat(f32 x) { return Vec2(x, x); }
	static Vec2 Zero() { return Vec2(0, 0); }
	static Vec2 One() { return Vec2(1, 1); }
	static Vec2 Right() { return Vec2(1, 0); }
	static Vec2 Up() { return Vec2(0, 1); }

	static f32 Dot(const Vec2& a, const Vec2& b);

	static Vec2 Lerp(const Vec2& a, const Vec2& b, f32 t);

	static void Normalize(Vec2& v);

	static Vec2 FromValuePtr(f32* v);
};

struct Vec3
{
	Vec3() : data{ 0, 0, 0 } {}
	Vec3(f32 x, f32 y, f32 z)
		: data{ x, y, z }
	{}

	union
	{
		f32 data[3];
		struct { f32 x, y, z; };
	};

	f32 At(i32 i) const;
	inline f32& operator[](i32 i) { return data[i]; }
	inline const f32& operator[](i32 i) const { return data[i]; }

	f32 SqrMagnitude() const;
	f32 Magnitude() const;
	Vec3 Normalized() const;
	Vec3 Abs() const;

	void Set(f32 x, f32 y, f32 z);

	Vec3 PlusF32(f32 rhs) const;
	Vec3 Plus(const Vec3& rhs) const;
	inline Vec3 operator+(const f32& rhs) const { return PlusF32(rhs); }
	inline Vec3& operator+=(const f32& rhs) { *this = *this + rhs; return *this; }
	inline Vec3 operator+(const Vec3& rhs) const { return Plus(rhs); }
	inline Vec3& operator+=(const Vec3& rhs) { *this = *this + rhs; return *this; }

	Vec3 Negate() const;
	inline Vec3 operator-() const { return Negate(); }

	Vec3 MinusF32(f32 rhs) const;
	Vec3 Minus(const Vec3& rhs) const;
	inline Vec3 operator-(const f32& rhs) const { return MinusF32(rhs); }
	inline Vec3& operator-=(const f32& rhs) { *this = *this - rhs; return *this; }
	inline Vec3 operator-(const Vec3& rhs) const { return Minus(rhs); }
	inline Vec3& operator-=(const Vec3& rhs) { *this = *this - rhs; return *this; }

	Vec3 MulF32(f32 rhs) const;
	Vec3 Mul(const Vec3& rhs) const;
	inline Vec3 operator*(const f32& rhs) const { return MulF32(rhs); }
	inline Vec3& operator*=(const f32& rhs) { *this = *this * rhs; return *this; }
	inline Vec3 operator*(const Vec3& rhs) const { return Mul(rhs); }
	inline Vec3& operator*=(const Vec3& rhs) { *this = *this * rhs; return *this; }

	Vec3 DivF32(f32 rhs) const;
	Vec3 Div(const Vec3& rhs) const;
	inline Vec3 operator/(const f32& rhs) const { return DivF32(rhs); }
	inline Vec3& operator/=(const f32& rhs) { *this = *this / rhs; return *this; }
	inline Vec3 operator/(const Vec3& rhs) const { return Div(rhs); }
	inline Vec3& operator/=(const Vec3& rhs) { *this = *this / rhs; return *this; }

	static Vec3 Splat(f32 x) { return Vec3(x, x, x); }
	static Vec3 Zero() { return Vec3(0, 0, 0); }
	static Vec3 One() { return Vec3(1, 1, 1); }
	static Vec3 Right() { return Vec3(1, 0, 0); }
	static Vec3 Up() { return Vec3(0, 1, 0); }
	static Vec3 Forward() { return Vec3(0, 0, 1); }

	static f32 Dot(const Vec3& a, const Vec3& b);
	
	static Vec3 Lerp(const Vec3& a, const Vec3& b, f32 t);

	static void Normalize(Vec3& v);

	static Vec3 Cross(const Vec3& a, const Vec3& b);

	static Vec3 FromValuePtr(f32* v);
};

struct Vec4
{
	Vec4() : data{ 0, 0, 0, 0 } {}
	Vec4(f32 x, f32 y, f32 z, f32 w)
		: data{ x, y, z, w }
	{}

	union
	{
		f32 data[4];
		struct { f32 x, y, z, w; };
	};

	f32 At(i32 i) const;
	inline f32& operator[](i32 i) { return data[i]; }
	inline const f32& operator[](i32 i) const { return data[i]; }

	f32 SqrMagnitude() const;
	f32 Magnitude() const;
	Vec4 Normalized() const;
	Vec4 Abs() const;

	void Set(f32 x, f32 y, f32 z, f32 w);

	Vec4 PlusF32(f32 rhs) const;
	Vec4 Plus(const Vec4& rhs) const;
	inline Vec4 operator+(const f32& rhs) const { return PlusF32(rhs); }
	inline Vec4& operator+=(const f32& rhs) { *this = *this + rhs; return *this; }
	inline Vec4 operator+(const Vec4& rhs) const { return Plus(rhs); }
	inline Vec4& operator+=(const Vec4& rhs) { *this = *this + rhs; return *this; }

	Vec4 Negate() const;
	inline Vec4 operator-() const { return Negate(); }

	Vec4 MinusF32(f32 rhs) const;
	Vec4 Minus(const Vec4& rhs) const;
	inline Vec4 operator-(const f32& rhs) const { return MinusF32(rhs); }
	inline Vec4& operator-=(const f32& rhs) { *this = *this - rhs; return *this; }
	inline Vec4 operator-(const Vec4& rhs) const { return Minus(rhs); }
	inline Vec4& operator-=(const Vec4& rhs) { *this = *this - rhs; return *this; }

	Vec4 MulF32(f32 rhs) const;
	Vec4 Mul(const Vec4& rhs) const;
	inline Vec4 operator*(const f32& rhs) const { return MulF32(rhs); }
	inline Vec4& operator*=(const f32& rhs) { *this = *this * rhs; return *this; }
	inline Vec4 operator*(const Vec4& rhs) const { return Mul(rhs); }
	inline Vec4& operator*=(const Vec4& rhs) { *this = *this * rhs; return *this; }

	Vec4 DivF32(f32 rhs) const;
	Vec4 Div(const Vec4& rhs) const;
	inline Vec4 operator/(const f32& rhs) const { return DivF32(rhs); }
	inline Vec4& operator/=(const f32& rhs) { *this = *this / rhs; return *this; }
	inline Vec4 operator/(const Vec4& rhs) const { return Div(rhs); }
	inline Vec4& operator/=(const Vec4& rhs) { *this = *this / rhs; return *this; }

	static Vec4 Splat(f32 x) { return Vec4(x, x, x, x); }
	static Vec4 Zero() { return Vec4(0, 0, 0, 0); }
	static Vec4 One() { return Vec4(1, 1, 1, 1); }

	static f32 Dot(const Vec4& a, const Vec4& b);

	static Vec4 Lerp(const Vec4& a, const Vec4& b, f32 t);

	static void Normalize(Vec4& v);

	static Vec4 FromValuePtr(f32* v);
};

struct Color
{
	Color() : data{ 0, 0, 0, 0 } {}
	Color(f32 r, f32 g, f32 b, f32 a)
		: data{ r, g, b, a }
	{}

	union
	{
		f32 data[4];
		struct { f32 r, g, b, a; };
	};

	static Color Black() { return Color(0, 0, 0, 1); }
	static Color Blue() { return Color(0, 0, 1, 1); }
	static Color Cyan() { return Color(0, 1, 1, 1); }
	static Color Gray() { return Color(0.5, 0.5, 0.5, 1); }
	static Color Green() { return Color(0, 1, 0, 1); }
	static Color Magenta() { return Color(1, 0, 1, 1); }
	static Color Red() { return Color(1, 0, 0, 1); }
	static Color Transparent() { return Color(0, 0, 0, 0); }
	static Color White() { return Color(1, 1, 1, 1); }
	static Color Yellow() { return Color(1, 1, 0, 1); }

	f32 At(i32 i) const;
	inline f32& operator[](i32 i) { return data[i]; }
	inline const f32& operator[](i32 i) const { return data[i]; }

	static Color FromValuePtr(f32* v);
};

struct Vec4i
{
	Vec4i() : data{ 0, 0, 0, 0 } {}
	Vec4i(i32 x, i32 y, i32 z, i32 w)
		: data{ x, y, z, w }
	{}

	union
	{
		i32 data[4];
		struct { i32 x, y, z, w; };
	};

	//i32 At(i32 i);
	inline i32& operator[](i32 i) { return data[i]; }
	inline const i32& operator[](i32 i) const { return data[i]; }

	static Vec4i FromValuePtr(i32* v);
};

struct Quat
{
	Quat() : data{ 0, 0, 0, 0 } {}
	Quat(f32 x, f32 y, f32 z, f32 w)
		: data{ x, y, z, w }
	{}

	union
	{
		f32 data[4];
		struct { f32 x, y, z, w; };
	};

	f32 At(i32 i) const;
	inline f32& operator[](i32 i) { return data[i]; }
	inline const f32& operator[](i32 i) const { return data[i]; }

	Quat Normalized() const;
	f32 Magnitude() const;
	Quat Inverse() const;

	Quat MulQuat(const Quat& rhs) const;
	Vec3 MulVec3(const Vec3& rhs) const;
	inline Quat operator*(const Quat& rhs) const { return MulQuat(rhs); }
	inline Quat& operator*=(const Quat& rhs) { *this = *this * rhs; return *this; }
	inline Vec3 operator*(const Vec3& rhs) const { return MulVec3(rhs); }

	Vec3 EulerAngles() const;

	static Quat Euler(f32 x, f32 y, f32 z);
	static Quat AngleAxis(f32 angleInDegrees, const Vec3& axis);

	static Quat Slerp(const Quat& a, const Quat& b, f32 t);

	static Quat FromValuePtr(f32* v);
};

struct Mat4
{
	Mat4() : data{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 } {}
	Mat4(Vec4 x, Vec4 y, Vec4 z, Vec4 w)
		: basis{ x, y, z, w }
	{}

	union
	{
		f32 data[16];
		Vec4 basis[4];
		//f32 m00, m01, m02, m03,
		//	m10, m11, m12, m13,
		//	m20, m21, m22, m23,
		//	m30, m31, m32, m33;
	};

	//Vec4 At(i32 i);
	inline Vec4& operator[](i32 i) { return basis[i]; }
	inline const Vec4& operator[](i32 i) const { return basis[i]; }

	//f32 At(i32 i, i32 j);
	//inline f32 operator[](i32 i, i32 j) { return At(i, j); }

	Mat4 Mul(const Mat4& rhs) const;
	inline Mat4 operator*(const Mat4& rhs) const { return Mul(rhs); }

	Mat4 Inverse() const;

	static Mat4 Identity();
	static Mat4 Zero();

	static Mat4 LookAt(const Vec3& eye, const Vec3& target, const Vec3& up);
	static Mat4 Ortho(f32 left, f32 right, f32 bottom, f32 top, f32 zNear, f32 zFar);
	static Mat4 Perspective(f32 fov, f32 aspect, f32 zNear, f32 zFar);
	static Mat4 TRS(const Vec3& pos, const Quat& q, const Vec3& s);

	static Mat4 Translate(const Vec3& translation, const Mat4& mat = Mat4::Identity());
	static Mat4 Rotate(const Quat& rotation, const Mat4& mat = Mat4::Identity());
	static Mat4 Scale(const Vec3& scaling, const Mat4& mat = Mat4::Identity());

	static void Decompose(const Mat4& m, Vec3& pos, Quat& rot, Vec3& scl);

	static Mat4 FromValuePtr(f32* v);
};

struct Box3
{
	Box3() : min(0, 0, 0), max(0, 0, 0) {}
	Box3(Vec3 min, Vec3 max)
		: min(min)
		, max(max)
	{}

	Vec3 min;
	Vec3 max;

	inline bool Overlaps(const Box3& other) const
	{
		return min.x <= other.max.x && max.x >= other.min.x
			&& min.y <= other.max.y && max.y >= other.min.y
			&& min.z <= other.max.z && max.z >= other.min.z;
	}
};