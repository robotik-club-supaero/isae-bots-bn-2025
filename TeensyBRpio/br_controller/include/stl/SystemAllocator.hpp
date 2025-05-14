#ifndef _DYNAMIC_ALLOCATOR_HPP_
#define _DYNAMIC_ALLOCATOR_HPP_

#include <memory>

class SystemAllocator {
  public:
    SystemAllocator() = default;

    template <typename T>
    [[nodiscard]] T *allocate(std::size_t n) {
        return std::allocator<T>().allocate(n);
    }

    /// @return false, because the default allocator does not support reallocation.
    template <typename T>
    bool extend(T *ptr, std::size_t old_size, std::size_t new_size) {
        return false;
    }

    template <typename T>
    void deallocate(T *ptr, std::size_t n) {
        std::allocator<T>().deallocate(ptr, n);
    }
};

#endif