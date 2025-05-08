// NB: this may not belong to directory "fsm", but technically it does store a state and I didn't want to create one more folder just for it.

#ifndef _PREALLOC_UNIQUE_PTR_HPP_
#define _PREALLOC_UNIQUE_PTR_HPP_

#include <cstddef>

constexpr bool is_power_of_two(std::size_t n) {
    return n != 0 && (n & (n - 1)) == 0;
}

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

    prealloc_unique_ptr() : m_alloc(new Alloc()), m_empty(true) {}
    ~prealloc_unique_ptr() {
        if (m_alloc != nullptr) {
            reset();
            delete m_alloc;
        }
    }

    prealloc_unique_ptr(const prealloc_unique_ptr &) = delete;
    prealloc_unique_ptr &operator=(const prealloc_unique_ptr &) = delete;

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

    void reset() {
        if (*this) {
            m_empty = true;
            get()->~T();
        }
    }

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

    operator bool() const { return !m_empty; }
    const T *get() const { return reinterpret_cast<const T *>(&m_alloc->buffer); }
    T *get() { return reinterpret_cast<T *>(&m_alloc->buffer); }
    const T &operator*() const { return *get(); }
    T &operator*() { return *get(); }
    const T *operator->() const { return get(); }
    T *operator->() { return get(); }
};

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