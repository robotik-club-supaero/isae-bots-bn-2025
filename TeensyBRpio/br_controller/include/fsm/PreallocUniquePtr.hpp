// NB: this may not belong to directory "fsm", but technically it does store a state and I didn't want to create one more folder just for it.

#ifndef _PREALLOC_UNIQUE_PTR_HPP_
#define _PREALLOC_UNIQUE_PTR_HPP_

#include <cstddef>

constexpr bool is_power_of_two(std::size_t n) {
    return n != 0 && (n & (n - 1)) == 0;
}

/// @brief A unique pointer that pre-allocates a buffer for the contained object. The buffer is only allocated once, and is re-used
/// throughout the lifetime of the smart pointer for all the managed objects (there may only be one at a time).
/// It is a move-only type, just like `std::unique_ptr`.
/// @tparam T The base type to store.
/// @tparam Size The maximum size of a type this smart pointer can store.
/// @tparam Align The maximum alignment of a type this smart pointer can store.
/// Size and Align will be the size and alignment of the allocated buffer, no matter what the size and alignment of the contained object are.
template <typename T, std::size_t Size = sizeof(T), std::size_t Align = alignof(T)>
class prealloc_unique_ptr {
    static_assert(Size > 0, "prealloc_unique_ptr cannot store zero-sized types");
    static_assert(is_power_of_two(Align), "Illegal pointer alignment");

  private:
    struct Alloc {
        alignas(Align) char buffer[Size];
    };

    Alloc *m_alloc;
    bool m_empty;

  public:
    static constexpr std::size_t max_size = Size;
    static constexpr std::size_t max_align = Align;

    /// Constructs a pointer that holds nothing. This pre-allocates the buffer.
    prealloc_unique_ptr() : m_alloc(new Alloc()), m_empty(true) {}
    ~prealloc_unique_ptr() {
        if (m_alloc != nullptr) {
            reset();
            delete m_alloc;
        }
    }

    prealloc_unique_ptr(const prealloc_unique_ptr &) = delete;
    prealloc_unique_ptr &operator=(const prealloc_unique_ptr &) = delete;

    /// Takes the buffer and the managed object from `other`, leaving it empty. A new buffer will be allocated if `other` is `emplace`d again.
    prealloc_unique_ptr(prealloc_unique_ptr &&other) : m_alloc(other.m_alloc), m_empty(other.m_empty) {
        other.m_empty = true;
        other.m_alloc = nullptr;
    }
    prealloc_unique_ptr &operator=(prealloc_unique_ptr &&other) {
        if (this != &other) {
            reset();
            m_alloc = other.m_alloc;
            m_empty = other.m_empty;

            other.m_empty = true;
            other.m_alloc = nullptr;
        }
        return *this;
    }

    /// Destroys the managed object.
    void reset() {
        if (*this) {
            m_empty = true;
            get()->~T();
        }
    }

    /// Constructs an object of type `U` in the pre-allocated buffer. If this `prealloc_unique_ptr` already holds an object, it is destroyed first.
    /// If the buffer was moved to another `prealloc_unique_ptr`, a new buffer is allocated first.
    ///
    /// If the maximum size or alignment of the buffer are not compatible with the layout of `U`, or `U` is not constructible with `Args`,
    /// instanciating this function is a compile-time error.
    template <typename U, typename... Args>
    void emplace(Args &&...args) {
        static_assert(std::is_base_of<T, U>::value, "U must derive from T");
        static_assert(sizeof(U) > 0, "Cannot emplace a zero-sized type");
        static_assert(sizeof(U) <= Size, "U is too large for this alloc");
        static_assert(alignof(U) <= Align, "Aligment of U is too strict for this alloc");

        if (m_alloc == nullptr) {
            // This is not designed to be used after move, but we'll allow it
            m_alloc = new Alloc();
        }
        reset();
        new (&m_alloc->buffer) U(std::forward<Args>(args)...);
        m_empty = false;
    }

    /// Checks if this `prealloc_unique_ptr` currently holds an object.
    operator bool() const { return !m_empty; }

    /// Gets or dereferences a pointer to the managed object. Calling any of those functions is undefined behavior unless this `prealloc_unique_ptr`
    /// holds an object, which can be checked with `operator bool()`.
    ///
    /// Calling `reset`, `emplace` or destroying this `prealloc_unique_ptr` immediately invalidate the pointer returned by those functions.
    /// @{
    const T *get() const { return reinterpret_cast<const T *>(&m_alloc->buffer); }
    T *get() { return reinterpret_cast<T *>(&m_alloc->buffer); }
    const T &operator*() const { return *get(); }
    T &operator*() { return *get(); }
    const T *operator->() const { return get(); }
    T *operator->() { return get(); }
    /// @}
};

/// Helper class that computes a `prealloc_unique_ptr<Base>` able to store any type from the list `Types` (in addition to `Base` itself if it
/// is a concrete type).
///
/// The computed type may be able to store more types than those in `Types`, as long as they are derived from `Base` and have a size and alignment no
/// greater than the maximum size and alignment permitted by the computed type.
template <typename Base, typename... Types>
class unique_ptr_for;

template <typename Base>
class unique_ptr_for<Base> {
  public:
    using type = prealloc_unique_ptr<Base, sizeof(Base), alignof(Base)>;
};

template <typename Base, typename Derived, typename... Others>
class unique_ptr_for<Base, Derived, Others...> {
  private:
    using base_type = unique_ptr_for<Base, Others...>::type;
    static constexpr std::size_t max_size = base_type::max_size;
    static constexpr std::size_t max_align = base_type::max_align;

  public:
    using type = prealloc_unique_ptr<Base, std::max(max_size, sizeof(Derived)), std::max(max_align, alignof(Derived))>;
};

#endif