#ifndef _POOL_ALLOC_HPP_
#define _POOL_ALLOC_HPP_

#include "defines/math.hpp"

#include <bitset>
#include <cstddef>
#include <memory>

/**
 * A memory pool, optimized for frequent allocations and deallocations of small arrays (one or a very few blocks each).
 *
 * This allocator is not state-less and is not compatible with STL containers (such as std::vector). It cannot be moved (because the moved-from
 * instance would not be left in a valid state), let alone copied. This allocator is not safe to share between threads.
 *
 * @tparam BlockSize The size of a block (in bytes). It must be close to the most frequent size to be allocated/deallocated. A single call to
 * `allocate` and `extend` (resp. `deallocate`) can reserve (resp. free) multiple blocks, but have a linear complexity with respect to the number of
 * blocks allocated.
 * @tparam BlockCount The number of blocks to pre-allocate. This defines the capacity of the memory pool.
 * `allocate` has the worst-case complexity O(BlockCount) (and best-case complexity O(1)).
 * @tparam Align The maximum alignment of a type supported by this allocator.
 *
 * If any of `BlockSize`, `BlockCount` or `Align` is zero the program is ill-formed. If `Align` is not a power of two, the program is ill-formed.
 */
template <std::size_t BlockSize, std::size_t BlockCount, std::size_t Align = alignof(std::max_align_t)>
class PoolAllocator {
    static_assert(BlockSize > 0, "BlockSize cannot be zero.");
    static_assert(BlockCount > 0, "BlockCount cannot be zero.");
    // We disallow BlockCount == SIZE_MAX so we can use SIZE_MAX as a marker for failure. Anyway, allocating `SIZE_MAX` blocks is very unlikely to be
    // a good idea...
    static_assert(BlockCount < SIZE_MAX, "BlockCount cannot be SIZE_MAX (reserved value).");
    static_assert(BlockCount <= SIZE_MAX / BlockSize, "BlockSize * BlockCount overflows size_t");
    static_assert(is_power_of_two(Align), "Illegal alignment requirement");

  public:
    constexpr std::size_t max_size() const { return BlockSize * BlockCount; }

    PoolAllocator() : m_buffer(std::make_unique<block[]>(BlockCount)), m_used(), m_hint(0) {}

    PoolAllocator(const PoolAllocator &) = delete;
    PoolAllocator(PoolAllocator &&) = delete;

    PoolAllocator &operator=(const PoolAllocator &) = delete;
    PoolAllocator &operator=(const PoolAllocator &&) = delete;

    /**
     * Allocates an uninitialized array.
     *
     * The allocated array can hold `n` elements of type `T`. The array can later be extended with `extend`, provided there is enough trailing free
     * space.
     *
     * If `n == 0`, this is a no-op and returns the null pointer.
     *
     * If `T` is a zero-sized or incomplete type, or has a stricter alignment than that permitted by this allocator, the program is ill-formed.
     *
     * @return A pointer to the allocated array, or `nullptr` if memory is exhausted or `n == 0`.
     */
    template <typename T>
    [[nodiscard]] T *allocate(std::size_t n) {
        static_assert(alignof(T) <= Align, "Alignment of T is too strict for this allocator.");
        static_assert(sizeof(T) != 0, "Cannot allocate zero-sized or incomplete types.");

        std::size_t block_count = blockCountFor<T>(n);
        return static_cast<T *>(allocateBlocks(block_count));
    }

    /**
     * Attempts to extend a previously allocated array.
     *
     * @param ptr This must be a living allocation of size `old_size` bytes, allocated for the exact same type `T` by this allocator.
     * @param old_size
     * @param new_size The new size of the array.
     * @return true if the array was extended, or false if there is not enough available memory after `ptr` to extend the array.
     *
     * If `new_size < old_size`, the behavior is undefined.
     */
    template <typename T>
    [[nodiscard]] bool extend(T *ptr, std::size_t old_size, std::size_t new_size) {
        std::size_t old_block_count = blockCountFor<T>(old_size);
        std::size_t new_block_count = blockCountFor<T>(new_size);
        return extendBlocks(ptr, old_block_count, new_block_count);
    }

    /**
     * Deallocates a previously allocated array. This invalidates `ptr`. Using `ptr` again in `extend` or `deallocate` is undefined behavior.
     *
     * @param ptr This must be a living allocation of size `n` bytes, allocated for the exact same type `T` by this allocator.
     * @param n
     */
    template <typename T>
    void deallocate(T *ptr, std::size_t n) {
        std::size_t block_count = blockCountFor<T>(n);
        freeBlocks(ptr, block_count);
    }

#ifdef _BR_DEBUG
    double_t memory_usage_percentage() const {
        std::size_t used_count = 0;
        for (std::size_t i = 0; i < BlockCount; i++) {
            if (m_used[i]) {
                used_count++;
            }
        }
        return static_cast<double_t>(used_count) / static_cast<double_t>(BlockCount);
    }
#endif

  private:
    static constexpr std::size_t ALLOC_FAILED = SIZE_MAX;

    template <typename T>
    [[nodiscard]] std::size_t blockCountFor(std::size_t n) const {
        if (SIZE_MAX / sizeof(T) < n) {
            // `BlockCount` cannot be `ALLOC_FAILED` so this signals overflow.
            return ALLOC_FAILED;
        }
        std::size_t bytes = n * sizeof(T);
        std::size_t block_count = bytes / BlockSize;
        if (bytes % BlockSize != 0) {
            block_count++;
        }
        return block_count;
    }

    // Non-template parts of `allocate`, `extend` and `deallocate` were extracted below

    /// The behavior is undefined if `ptr` is not the pointer to a non-empty, living allocation.
    [[nodiscard]] std::size_t getBlockIndex(void *ptr) const {
        return (static_cast<std::byte *>(ptr) - reinterpret_cast<std::byte *>(m_buffer.get())) / BlockSize;
    }

    [[nodiscard]] void *allocateBlocks(std::size_t block_count) {
        if (block_count == 0 || block_count == ALLOC_FAILED) {
            return nullptr;
        }
        std::size_t index = ALLOC_FAILED;

        if (isFree(m_hint, block_count)) {
            for (std::size_t i = m_hint; i < m_hint + block_count; i++) {
                m_used[i] = true;
            }
            index = m_hint;
        } else {
            for (std::size_t i = 0; i <= BlockCount - block_count;) {
                bool found = true;
                for (std::size_t j = i; j < i + block_count; j++) {
                    if (m_used[j]) {
                        found = false;
                        i = j + 1;
                        break;
                    }
                }
                if (found) {
                    for (std::size_t j = i; j < i + block_count; j++) {
                        m_used[j] = true;
                    }
                    index = i;
                    break;
                }
            }
        }
        if (index != ALLOC_FAILED) {
            m_hint = index + block_count;
            if (m_hint >= BlockCount) {
                m_hint = 0;
            }
            return &m_buffer[index];
        } else {
            return nullptr;
        }
    }

    [[nodiscard]] bool extendBlocks(void *ptr, std::size_t old_block_count, std::size_t new_block_count) {
        if (ptr == nullptr || new_block_count == ALLOC_FAILED) {
            return false;
        }

        std::size_t i = getBlockIndex(ptr);
        if (i + new_block_count > BlockCount) {
            return false;
        }
        for (std::size_t j = old_block_count; j < new_block_count; j++) {
            if (m_used[i + j]) {
                return false;
            }
        }

        for (std::size_t j = old_block_count; j < new_block_count; j++) {
            m_used[i + j] = true;
        }
        if (m_hint >= i + old_block_count && m_hint < i + new_block_count) {
            m_hint = i + new_block_count;
        }
        return true;
    }

    void freeBlocks(void *ptr, std::size_t block_count) {
        if (ptr == nullptr || block_count == 0) {
            return;
        }

        std::size_t i = getBlockIndex(ptr);
        for (std::size_t j = 0; j < block_count; j++) {
            m_used[i + j] = false;
        }
        if (i < m_hint) {
            m_hint = i;
        }
    }

    [[nodiscard]] bool isFree(std::size_t i, std::size_t block_count) const {
        if (i + block_count > BlockCount) {
            return false;
        }
        for (std::size_t j = i; j < i + block_count; j++) {
            if (m_used[j]) {
                return false;
            }
        }
        return true;
    }

    using block = std::aligned_storage<BlockSize, Align>::type;
    std::unique_ptr<block[]> m_buffer;
    std::bitset<BlockCount> m_used;
    /// Index of a block that **may** be free.
    std::size_t m_hint;
};

#endif