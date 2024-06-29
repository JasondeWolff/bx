#include "bx/engine/core/math.hpp"

#define GLM_LANG_STL11_FORCED
#define GLM_ENABLE_EXPERIMENTAL
//#define GLM_FORCE_QUAT_DATA_XYZW

#include <glm/glm.hpp>

#include <glm/gtx/hash.hpp>
#include <glm/gtx/string_cast.hpp>

#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>

// FIXME: There is a bug with glm when GLM_FORCE_QUAT_DATA_XYZW is defined
#define GLM_PATCH_QUAT_DATA_XYZW

#ifdef GLM_PATCH_QUAT_DATA_XYZW
static Quat QuatFromGLM(glm::quat q)
{
	return Quat(q.x, q.y, q.z, q.w);
}

static glm::quat QuatToGLM(const Quat& q)
{
	return glm::quat(q.w, q.x, q.y, q.z);
}
#else
static Quat QuatFromGLM(glm::quat q)
{
	return Quat::FromValuePtr(glm::value_ptr(q));
}

static glm::quat QuatToGLM(const Quat& q)
{
	return glm::make_quat(q.data);
}
#endif

f32 Vec2::At(i32 i) const
{
	return data[i];
}

f32 Vec2::SqrMagnitude() const
{
	return Vec2::Dot(*this, *this);
}

f32 Vec2::Magnitude() const
{
	return sqrtf(SqrMagnitude());
}

Vec2 Vec2::Normalized() const
{
	f32 magnitude = Magnitude();
	if (magnitude > SAFE_DIV_EPSILON)
	{
		f32 invMagnitude = 1.0 / magnitude;
		return (*this) * invMagnitude;
	}
	else
	{
		return *this;
	}
}

Vec2 Vec2::Abs() const
{
	return Vec2(fabsf(x), fabsf(y));
}

void Vec2::Set(f32 x, f32 y)
{
	data[0] = x;
	data[1] = y;
}

Vec2 Vec2::PlusF32(f32 rhs) const
{
	return Vec2(x + rhs, y + rhs);
}

Vec2 Vec2::Plus(const Vec2& rhs) const
{
	return Vec2(x + rhs.x, y + rhs.y);
}

Vec2 Vec2::Negate() const
{
	return Vec2(-x, -y);
}

Vec2 Vec2::MinusF32(f32 rhs) const
{
	return Vec2(x - rhs, y - rhs);
}

Vec2 Vec2::Minus(const Vec2& rhs) const
{
	return Vec2(x - rhs.x, y - rhs.y);
}

Vec2 Vec2::MulF32(f32 rhs) const
{
	return Vec2(x * rhs, y * rhs);
}

Vec2 Vec2::Mul(const Vec2& rhs) const
{
	return Vec2(x * rhs.x, y * rhs.y);
}

Vec2 Vec2::DivF32(f32 rhs) const
{
	f32 invRhs = 1.0 / rhs;
	return Vec2(x * invRhs, y * invRhs);
}

Vec2 Vec2::Div(const Vec2& rhs) const
{
	return Vec2(x / rhs.x, y / rhs.y);
}

f32 Vec2::Dot(const Vec2& a, const Vec2& b)
{
	return a.x * b.x + a.y * b.y;
}

Vec2 Vec2::Lerp(const Vec2& a, const Vec2& b, f32 t)
{
	return Math::Lerp(a, b, t);
}

void Vec2::Normalize(Vec2& v)
{
	v = v.Normalized();
}

Vec2 Vec2::FromValuePtr(f32* vptr)
{
	Vec2 v;
	memcpy(v.data, vptr, sizeof(Vec2));
	return v;
}

f32 Vec3::At(i32 i) const
{
	return data[i];
}

f32 Vec3::SqrMagnitude() const
{
	return Vec3::Dot(*this, *this);
}

f32 Vec3::Magnitude() const
{
	return sqrtf(SqrMagnitude());
}

Vec3 Vec3::Normalized() const
{
	f32 magnitude = Magnitude();
	if (magnitude > SAFE_DIV_EPSILON)
	{
		f32 invMagnitude = 1.0 / magnitude;
		return (*this) * invMagnitude;
	}
	else
	{
		return *this;
	}
}

Vec3 Vec3::Abs() const
{
	return Vec3(fabsf(x), fabsf(y), fabsf(z));
}

void Vec3::Set(f32 x, f32 y, f32 z)
{
	data[0] = x;
	data[1] = y;
	data[2] = z;
}

Vec3 Vec3::PlusF32(f32 rhs) const
{
	return Vec3(x + rhs, y + rhs, z + rhs);
}

Vec3 Vec3::Plus(const Vec3& rhs) const
{
	return Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
}

Vec3 Vec3::Negate() const
{
	return Vec3(-x, -y, -z);
}

Vec3 Vec3::MinusF32(f32 rhs) const
{
	return Vec3(x - rhs, y - rhs, z - rhs);
}

Vec3 Vec3::Minus(const Vec3& rhs) const
{
	return Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
}

Vec3 Vec3::MulF32(f32 rhs) const
{
	return Vec3(x * rhs, y * rhs, z * rhs);
}

Vec3 Vec3::Mul(const Vec3& rhs) const
{
	return Vec3(x * rhs.x, y * rhs.y, z * rhs.z);
}

Vec3 Vec3::DivF32(f32 rhs) const
{
	f32 invRhs = 1.0 / rhs;
	return Vec3(x * invRhs, y * invRhs, z * invRhs);
}

Vec3 Vec3::Div(const Vec3& rhs) const
{
	return Vec3(x / rhs.x, y / rhs.y, z / rhs.z);
}

f32 Vec3::Dot(const Vec3& a, const Vec3& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 Vec3::Lerp(const Vec3& a, const Vec3& b, f32 t)
{
	return Math::Lerp(a, b, t);
}

void Vec3::Normalize(Vec3& v)
{
	v = v.Normalized();
}

Vec3 Vec3::Cross(const Vec3& a, const Vec3& b)
{
	return Vec3(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
	);
}

Vec3 Vec3::FromValuePtr(f32* vptr)
{
	Vec3 v;
	memcpy(v.data, vptr, sizeof(Vec3));
	return v;
}

f32 Vec4::At(i32 i) const
{
	return data[i];
}

f32 Vec4::SqrMagnitude() const
{
	return Vec4::Dot(*this, *this);
}

f32 Vec4::Magnitude() const
{
	return sqrtf(SqrMagnitude());
}

Vec4 Vec4::Normalized() const
{
	f32 magnitude = Magnitude();
	if (magnitude > SAFE_DIV_EPSILON)
	{
		f32 invMagnitude = 1.0 / magnitude;
		return (*this) * invMagnitude;
	}
	else
	{
		return *this;
	}
}

Vec4 Vec4::Abs() const
{
	return Vec4(fabsf(x), fabsf(y), fabsf(z), fabsf(w));
}

void Vec4::Set(f32 x, f32 y, f32 z, f32 w)
{
	data[0] = x;
	data[1] = y;
	data[2] = z;
	data[3] = w;
}

Vec4 Vec4::PlusF32(f32 rhs) const
{
	return Vec4(x + rhs, y + rhs, z + rhs, w + rhs);
}

Vec4 Vec4::Plus(const Vec4& rhs) const
{
	return Vec4(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
}

Vec4 Vec4::Negate() const
{
	return Vec4(-x, -y, -z, -w);
}

Vec4 Vec4::MinusF32(f32 rhs) const
{
	return Vec4(x - rhs, y - rhs, z - rhs, w - rhs);
}

Vec4 Vec4::Minus(const Vec4& rhs) const
{
	return Vec4(x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w);
}

Vec4 Vec4::MulF32(f32 rhs) const
{
	return Vec4(x * rhs, y * rhs, z * rhs, w * rhs);
}

Vec4 Vec4::Mul(const Vec4& rhs) const
{
	return Vec4(x * rhs.x, y * rhs.y, z * rhs.z, w * rhs.w);
}

Vec4 Vec4::DivF32(f32 rhs) const
{
	f32 invRhs = 1.0 / rhs;
	return Vec4(x * invRhs, y * invRhs, z * invRhs, w * invRhs);
}

Vec4 Vec4::Div(const Vec4& rhs) const
{
	return Vec4(x / rhs.x, y / rhs.y, z / rhs.z, w / rhs.w);
}

f32 Vec4::Dot(const Vec4& a, const Vec4& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

Vec4 Vec4::Lerp(const Vec4& a, const Vec4& b, f32 t)
{
	return Math::Lerp(a, b, t);
}

void Vec4::Normalize(Vec4& v)
{
	v = v.Normalized();
}

Vec4 Vec4::FromValuePtr(f32* vptr)
{
	Vec4 v;
	memcpy(v.data, vptr, sizeof(Vec4));
	return v;
}

f32 Color::At(i32 i) const
{
	return data[i];
}

Color Color::FromValuePtr(f32* vptr)
{
	Color v;
	memcpy(v.data, vptr, sizeof(Color));
	return v;
}

Vec4i Vec4i::FromValuePtr(i32* vptr)
{
	Vec4i v;
	memcpy(v.data, vptr, sizeof(Vec4i));
	return v;
}

f32 Quat::At(i32 i) const
{
	return data[i];
}

Quat Quat::Normalized() const
{
	glm::quat q = glm::normalize(QuatToGLM(*this));
	return QuatFromGLM(q);
}

f32 Quat::SqrMagnitude() const
{
	return x * x + y * y + z * z + w * w;
}

f32 Quat::Magnitude() const
{
	return sqrt(SqrMagnitude());
}

Quat Quat::Inverse() const
{
	f32 magnitude = Magnitude();
	if (magnitude > SAFE_DIV_EPSILON)
	{
		f32 invMagnitude = 1.0 / magnitude;
		return Quat(
			-x * invMagnitude,
			-y * invMagnitude,
			-z * invMagnitude,
			-w * invMagnitude
		);
	}
	else
	{
		return *this;
	}
}

Quat Quat::PlusQuat(const Quat& rhs) const
{
	return Quat(x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w);
}

Quat Quat::PlusF32(f32 rhs) const
{
	return Quat(x + rhs, y + rhs, z + rhs, w + rhs);
}

Quat Quat::Negate() const
{
	return Quat(-x, -y, -z, -w);
}

Quat Quat::MulQuat(const Quat& rhs) const
{
	return Quat(
		w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
		w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
		w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w,
		w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z
	);
}

Vec3 Quat::MulVec3(const Vec3& rhs) const
{
	Vec3 qv(x, y, z);
	Vec3 t = Vec3::Cross(qv, rhs) * 2.0;
	return rhs + t * w + Vec3::Cross(qv, t);
}

Quat Quat::MulF32(f32 rhs) const
{
	return Quat(x * rhs, y * rhs, z * rhs, w * rhs);
}

Quat Quat::DivF32(f32 rhs) const
{
	f32 invRhs = 1.0 / rhs;
	return Quat(x * invRhs, y * invRhs, z * invRhs, w * invRhs);
}

Vec3 Quat::EulerAngles() const
{
	Vec3 euler;
	f32 sinrCosp = 2.0 * (w * x + y * z);
	f32 cosrCosp = 1.0 - 2.0 * (x * x + y * y);
	euler.x = Math::Degrees(std::atan2(sinrCosp, cosrCosp));
	f32 sinp = 2.0 * (w * y - z * x);
	if (std::abs(sinp) >= 1.0)
		euler.y = Math::Degrees(std::copysignf(Math::PI_2, sinp));
	else
		euler.y = Math::Degrees(std::asin(sinp));
	f32 sinyCosp = 2.0 * (w * z + x * y);
	f32 cosyCosp = 1.0 - 2.0 * (y * y + z * z);
	euler.z = Math::Degrees(std::atan2f(sinyCosp, cosyCosp));
	return euler;
}

Quat Quat::Euler(f32 x, f32 y, f32 z)
{
	Vec3 halfEuler = Vec3(x, y, z) * 0.5;
	f32 cr = cosf(Math::Radians(halfEuler.x));
	f32 sr = sinf(Math::Radians(halfEuler.x));
	f32 cy = cosf(Math::Radians(halfEuler.z));
	f32 sy = sinf(Math::Radians(halfEuler.z));
	f32 cp = cosf(Math::Radians(halfEuler.y));
	f32 sp = sinf(Math::Radians(halfEuler.y));
	return Quat(
		sr * cp * cy - cr * sp * sy,
		cr * sp * cy + sr * cp * sy,
		cr * cp * sy - sr * sp * cy,
		cr* cp* cy + sr * sp * sy
	);
}

Quat Quat::AngleAxis(f32 angleInDegrees, const Vec3& axis)
{
	f32 rangle = Math::Radians(angleInDegrees);
	f32 sha = sinf(rangle * 0.5);
	return Quat(axis.x * sha, axis.y * sha, axis.z * sha, cosf(rangle * 0.5));
}

f32 Quat::Dot(const Quat& a, const Quat& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

Quat Quat::Slerp(const Quat& a, const Quat& b, f32 t)
{
	Quat z = b;

	f32 cosTheta = Dot(a, b);
	if (cosTheta < 0)
	{
		z = -b;
		cosTheta = -cosTheta;
	}

	if (cosTheta > 1 - SAFE_DIV_EPSILON)
	{
		return Quat(
			Math::Lerp(a.x, z.x, t),
			Math::Lerp(a.y, z.y, t),
			Math::Lerp(a.z, z.z, t),
			Math::Lerp(a.w, z.w, t)
		);
	}
	else
	{
		f32 angle = acosf(cosTheta);
		return (a * sin((1.0 - t) * angle) + z * sin(t * angle)) / sinf(angle);
	}
}

Quat Quat::FromValuePtr(f32* vptr)
{
	Quat q;
	memcpy(q.data, vptr, sizeof(Quat));
	return q;
}

Mat4 Mat4::Mul(const Mat4& rhs) const
{
	glm::mat4 l = glm::make_mat4(data);
	glm::mat4 r = glm::make_mat4(rhs.data);
	glm::mat4 m = l * r;
	return Mat4::FromValuePtr(glm::value_ptr(m));
}

Mat4 Mat4::Inverse() const
{
	glm::mat4 m = glm::make_mat4(data);
	glm::mat4 im = glm::inverse(m);
	return Mat4::FromValuePtr(glm::value_ptr(im));
}

Mat4 Mat4::Identity()
{
	static Mat4 s_identity = Mat4(Vec4(1, 0, 0, 0), Vec4(0, 1, 0, 0), Vec4(0, 0, 1, 0), Vec4(0, 0, 0, 1));
	return s_identity;
}

Mat4 Mat4::Zero()
{
	return Mat4(Vec4(0, 0, 0, 0), Vec4(0, 0, 0, 0), Vec4(0, 0, 0, 0), Vec4(0, 0, 0, 0));
}

Mat4 Mat4::LookAt(const Vec3& eye, const Vec3& center, const Vec3& up)
{
	glm::vec3 e = glm::make_vec3(eye.data);
	glm::vec3 c = glm::make_vec3(center.data);
	glm::vec3 u = glm::make_vec3(up.data);
	glm::mat4 m = glm::lookAt(e, c, u);
	return Mat4::FromValuePtr(glm::value_ptr(m));
}

Mat4 Mat4::Ortho(f32 left, f32 right, f32 bottom, f32 top, f32 zNear, f32 zFar)
{
	glm::mat4 m = glm::ortho(left, right, bottom, top, zNear, zFar);
	return Mat4::FromValuePtr(glm::value_ptr(m));
}

Mat4 Mat4::Perspective(f32 fov, f32 aspect, f32 zNear, f32 zFar)
{
	glm::mat4 m = glm::perspective(glm::radians(fov), aspect, zNear, zFar);
	return Mat4::FromValuePtr(glm::value_ptr(m));
}

Mat4 Mat4::TRS(const Vec3& pos, const Quat& rot, const Vec3& scl)
{
	/* This creates a mat4 directly from pos, euler, scale
	r = rad(r)
		var cr = r.map { |n| n.cos }.toList
		var sr = r.map { |n| n.sin }.toList

		return [s[0] * ( cr[1] * cr[2]),    s[1] * (sr[0] * sr[1] * cr[2] - cr[0] * sr[2]),     s[2] * (cr[0] * sr[1] * cr[2] + sr[0] * sr[2]),     0,
				s[0] * ( cr[1] * sr[2]),    s[1] * (sr[0] * sr[1] * sr[2] + cr[0] * cr[2]),     s[2] * (cr[0] * sr[1] * sr[2] - sr[0] * cr[2]),     0,
				s[0] * (-sr[1]),            s[1] * (sr[0] * cr[1]),                             s[2] * (cr[0] * cr[1]),                             0,
				p[0],                       p[1],                                               p[2],                                               1]
	*/

	glm::vec3 p = glm::make_vec3(pos.data);
	glm::quat r = QuatToGLM(rot);
	glm::vec3 s = glm::make_vec3(scl.data);
	glm::mat4 m = glm::translate(glm::mat4(1.0f), p) * glm::mat4(r) * glm::scale(glm::mat4(1.0f), s);
	return Mat4::FromValuePtr(glm::value_ptr(m));
}

Mat4 Mat4::Translate(const Vec3& translation, const Mat4& mat)
{
	glm::vec3 t = glm::make_vec3(translation.data);
	glm::mat4 m = glm::make_mat4(mat.data);
	glm::mat4 tm = glm::translate(m, t);
	return Mat4::FromValuePtr(glm::value_ptr(tm));
}

Mat4 Mat4::Rotate(const Quat& rotation, const Mat4& mat)
{
	glm::quat r = QuatToGLM(rotation);
	glm::mat4 m = glm::make_mat4(mat.data);
	glm::mat4 rm = m * glm::mat4(r);
	return Mat4::FromValuePtr(glm::value_ptr(rm));
}

Mat4 Mat4::Scale(const Vec3& scaling, const Mat4& mat)
{
	glm::vec3 s = glm::make_vec3(scaling.data);
	glm::mat4 m = glm::make_mat4(mat.data);
	glm::mat4 sm = glm::scale(m, s);
	return Mat4::FromValuePtr(glm::value_ptr(sm));
}

void Mat4::Decompose(const Mat4& m, Vec3& pos, Quat& rot, Vec3& scl)
{
	glm::mat4 transformation = glm::make_mat4(m.data);
	glm::vec3 scale;
	glm::quat rotation;
	glm::vec3 translation;
	glm::vec3 skew;
	glm::vec4 perspective;
	glm::decompose(transformation, scale, rotation, translation, skew, perspective);
	pos = Vec3::FromValuePtr(glm::value_ptr(translation));
	rot = QuatFromGLM(rotation);
	scl = Vec3::FromValuePtr(glm::value_ptr(scale));
}

Mat4 Mat4::FromValuePtr(f32* vptr)
{
	Mat4 m;
	memcpy(m.data, vptr, sizeof(Mat4));
	return m;
}