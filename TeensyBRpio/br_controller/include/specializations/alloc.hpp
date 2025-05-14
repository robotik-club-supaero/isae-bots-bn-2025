#ifndef _SPEC_ALLOCATOR_HPP_
#define _SPEC_ALLOCATOR_HPP_

#if defined(ARDUINO)

#include "configuration.hpp"
#include "stl/PoolAllocator.hpp"

using memory_pool = PoolAllocator<BLOCK_SIZE, BLOCK_COUNT>;

extern memory_pool ALLOCATOR;

template <typename T>
struct allocator_s {
    using type = memory_pool;
    static constexpr type &get() { return ALLOCATOR; }
};

// Allocation failures are reported through the return value directly
#define MAY_ALLOC [[nodiscard]]

#else

#include "stl/SystemAllocator.hpp"

extern SystemAllocator ALLOCATOR;

template <typename T>
struct allocator_s {
    using type = SystemAllocator;
    static constexpr type get() { return ALLOCATOR; }
};

// Allocation failures are reported through exceptions
#define MAY_ALLOC [[maybe_unused]]

#endif

#endif
