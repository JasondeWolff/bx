#pragma once

#ifndef MEMORY_CUSTOM_CONTAINERS

#include <functional>

template <typename T>
using Hash = std::hash<T>;

#else // MEMORY_CUSTOM_CONTAINERS

#include "Engine/Core/ByteTypes.hpp"
#include "Engine/Containers/String.hpp"

template <typename T>
struct Hash;

namespace HashFunctions
{
	static SizeType FNV1a(const u8* data, SizeType size)
	{
		// 32 bit params
		// uint32_t constexpr fnv_prime = 16777619U;
		// uint32_t constexpr fnv_offset_basis = 2166136261U;

		// 64 bit params
		u64 constexpr fnv_prime = 1099511628211ULL;
		u64 constexpr fnv_offset_basis = 14695981039346656037ULL;

		u64 hash = fnv_offset_basis;

		for (SizeType i = 0; i < size; ++i)
		{
			hash ^= data[i];
			hash *= fnv_prime;
		}

		return hash;
	}
}

#define DECLARE_PRIMITIVE_HASH(Type, HashFn) \
template <> struct Hash<Type> { inline SizeType operator()(const Type& v) const { return HashFn(reinterpret_cast<const u8*>(&v), sizeof(Type));} };

DECLARE_PRIMITIVE_HASH(i32, HashFunctions::FNV1a);
DECLARE_PRIMITIVE_HASH(u32, HashFunctions::FNV1a);
DECLARE_PRIMITIVE_HASH(f32, HashFunctions::FNV1a);
DECLARE_PRIMITIVE_HASH(f64, HashFunctions::FNV1a);
DECLARE_PRIMITIVE_HASH(SizeType, HashFunctions::FNV1a);

template <>
struct Hash<String>
{
	inline SizeType operator()(const String& v) const
	{
		return HashFunctions::FNV1a(reinterpret_cast<const u8*>(v.data()), v.size());
	}
};

#endif