#ifndef _SMALL_DEQUE_HPP_
#define _SMALL_DEQUE_HPP_

#include "logging.hpp"
#include "specializations/alloc.hpp"

#include <optional>

template <typename T>
class SmallDeque;

/// Iterator for `SmallDeque`.
template <typename T, bool Const, bool Rev>
class SmallDequeIterator {

  public:
    using iterator_category = std::bidirectional_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = std::conditional_t<Const, const T *, T *>;
    using reference = std::conditional_t<Const, const T &, T &>;

    SmallDequeIterator<T, Const, Rev> &operator++() {
        if (m_cap == 0) {
            return *this;
        }
        if constexpr (Rev) {
            if (m_pos == 0) {
                m_pos = m_cap;
            }
            m_pos--;
        } else {
            if (m_pos == m_cap) {
                m_pos = 0;
            }
            m_pos++;
        }
        return *this;
    }
    SmallDequeIterator<T, Const, Rev> operator++(int) {
        SmallDequeIterator<T, Const, Rev> cpy = *this;
        ++(*this);
        return cpy;
    }

    reference operator*() const { return m_ptr[m_pos]; }
    pointer operator->() const { return &m_ptr[m_pos]; }

    constexpr bool operator==(const SmallDequeIterator<T, Const, Rev> &other) const { return m_ptr == other.m_ptr && m_pos == other.m_pos; }
    constexpr bool operator!=(const SmallDequeIterator<T, Const, Rev> &other) const { return !(*this == other); }

  private:
    friend class SmallDeque<T>;

    using alloc_type = std::conditional_t<Const, const typename SmallDeque<T>::Alloc &, typename SmallDeque<T>::Alloc &>;

    SmallDequeIterator(alloc_type alloc, std::size_t pos) : SmallDequeIterator(alloc.raw(), alloc.capacity(), pos) {}
    SmallDequeIterator(pointer ptr, std::size_t cap, std::size_t pos) : m_ptr(ptr), m_cap(cap), m_pos(pos) {}

    pointer m_ptr;
    std::size_t m_cap;
    std::size_t m_pos;
};

/**
 * An implementation of a double-ended queue designed for small sequences and predictable memory usage.
 * The implementation uses a single ring buffer to store the elements.
 *
 * All the functions that may allocate return `true` on success or `false` on failure, but the return value may be ignored if the allocator signals
 * failure with an exception instead of a `nullptr`.
 */
template <typename T>
class SmallDeque {
  public:
    using size_type = std::size_t;
    using iterator = SmallDequeIterator<T, false, false>;
    using const_iterator = SmallDequeIterator<T, true, false>;
    using reverse_iterator = SmallDequeIterator<T, false, true>;
    using const_reverse_iterator = SmallDequeIterator<T, true, true>;

    SmallDeque() : m_alloc(), m_head(), m_size() {}
    ~SmallDeque() { clear(); }

#if defined(__EXCEPTIONS) && !defined(ARDUINO)
    SmallDeque(const SmallDeque<T> &other) : SmallDeque() { *this = other; }
    SmallDeque &operator=(const SmallDeque<T> &other) {
        if (!reserve(other.size())) {
            throw std::bad_alloc();
        }
        for (const T &element : other) {
            new (m_alloc.get(m_size++)) T(element);
        }
        return *this;
    }

    template <typename Iter>
    SmallDeque(Iter begin, Iter end) : SmallDeque() {
        extend(std::forward<Iter>(begin), std::forward<Iter>(end));
    }
#else
    /// We delete copy constructors because we don't want implicit copies + the copy is fallible (allocation may fail). Use `copy` explicitly instead.
    /// @{
    SmallDeque(const SmallDeque<T> &) = delete;
    SmallDeque &operator=(const SmallDeque<T> &) = delete;
/// @}
#endif

    SmallDeque(SmallDeque<T> &&other) : m_alloc(std::move(other.m_alloc)), m_head(other.m_head), m_size(other.m_size) {
        other.m_head = 0;
        other.m_size = 0;
    }
    SmallDeque &operator=(SmallDeque<T> &&other) {
        m_alloc = std::move(other.m_alloc);
        m_head = other.m_head;
        m_size = other.m_size;

        other.m_head = 0;
        other.m_size = 0;
        return *this;
    }

    /// Returns the maximum number of elements this deque can store without reallocating.
    constexpr std::size_t capacity() const { return m_alloc.capacity(); }
    /// Returns the number of elements currently in the deque.
    constexpr std::size_t size() const { return m_size; }
    /// Checks whether the deque is empty.
    constexpr bool empty() const { return m_size == 0; }
    /// Checks whether the currently allocated storage is full.
    constexpr bool full() const { return m_size == capacity(); }

    bool operator==(const SmallDeque<T> &other) const
        requires std::equality_comparable<T>
    {
        if (size() != other.size()) {
            return false;
        }
        for (std::size_t i = 0; i < m_size; i++) {
            if ((*this)[i] != other[i]) {
                return false;
            }
        }
        return true;
    }
    bool operator!=(const SmallDeque<T> &other) const
        requires std::equality_comparable<T>
    {
        return !(*this == other);
    }

    /// Returns a reference to the first element in the deque.
    ///
    /// If `empty()` is `true`, the behavior is undefined.
    /// @{
    const T &front() const { return (*this)[0]; }
    T &front() { return (*this)[0]; }
    /// @}

    /// Returns a reference to the last element in the deque.
    ///
    /// If `empty()` is `true`, the behavior is undefined.
    /// @{
    const T &back() const { return (*this)[m_size - 1]; }
    T &back() { return (*this)[m_size - 1]; }
    /// @}

    /// Returns a reference to the element at specified location pos.
    ///
    /// If `index >= size()`, the behavior is undefined.
    /// @{
    const T &operator[](std::size_t index) const { return *m_alloc.get(storage_index_of(index)); }
    T &operator[](std::size_t index) { return *m_alloc.get(storage_index_of(index)); }
    /// @}

    iterator begin() { return iterator(m_alloc, m_head); }
    iterator end() { return iterator(m_alloc, m_size > 0 ? storage_index_of(m_size - 1) + 1 : m_head); }

    const_iterator begin() const { return const_iterator(m_alloc, m_head); }
    const_iterator end() const { return const_iterator(m_alloc, m_size > 0 ? storage_index_of(m_size - 1) + 1 : m_head); }

    reverse_iterator rbegin() { return reverse_iterator(m_alloc, m_size > 0 ? storage_index_of(m_size - 1) : 0); }
    reverse_iterator rend() { return reverse_iterator(m_alloc, m_size > 0 ? (m_head + capacity() - 1) % capacity() : 0); }

    const_reverse_iterator rbegin() const { return const_reverse_iterator(m_alloc, m_size > 0 ? storage_index_of(m_size - 1) : 0); }
    const_reverse_iterator rend() const { return const_reverse_iterator(m_alloc, m_size > 0 ? (m_head + capacity() - 1) % capacity() : 0); }

    /// Reserves capacity for at least `count` additional elements to be inserted in this deque.
    ///
    /// If `size() + count > capacity()`, all iterators (including the `end()` iterator) and all references to the elements are invalidated.
    /// Otherwise, this is a no-op and no iterators or references are invalidated.
    ///
    /// @return true if this deque can store `count` additional elements without reallocating. false if memory allocation fails.
    /// @throws Any exception thrown by the allocator. If an exception is thrown by the allocator, this function has no effect. If an exception
    /// is thrown by the move constructor of an element, the behavior is undefined.
    MAY_ALLOC bool reserve(std::size_t count) {
        std::size_t old_cap = capacity();
        std::size_t new_cap = m_size + count;
        if (new_cap <= old_cap) {
            return true;
        }
        if (m_alloc.grow(new_cap)) {
            if (m_head + m_size > old_cap) {
                // Fixing layout: example with old_cap = 4, new_cap = 8, m_head = 3, m_size = 2:
                // `B - - A` needs to become `- - - A B - - -`
                std::size_t left_over = m_head + m_size - old_cap;
                std::size_t j = old_cap;
                for (std::size_t i = 0; i < left_over; i++) {
                    new (m_alloc.get(j)) T(std::move(*m_alloc.get(i)));
                    m_alloc.get(i)->~T();

                    if (++j == new_cap) {
                        j = 0;
                    }
                }
            }
            return true;
        } else {
            Alloc new_alloc = Alloc(new_cap);
            if (new_alloc.capacity() != new_cap) {
                return false;
            }

            for (std::size_t i = 0; i < m_size; i++) {
                std::size_t storage_i = storage_index_of(i);
                new (new_alloc.get(i)) T(std::move(*m_alloc.get(storage_i)));
                m_alloc.get(storage_i)->~T();
            }

            m_alloc = std::move(new_alloc);
            m_head = 0;

            return true;
        }
    }

    /// Inserts an element to the end of this deque. Reallocates first if `full()` is `true`.
    /// @returns true if the element was inserted, false if the deque is full and memory reallocation failed.
    ///
    /// If this returns `true`, all iterators (including the `end()` iterator) are invalidated. If a reallocation takes
    /// places, all references to the elements are also invalidated.
    /// @{
    MAY_ALLOC bool push_back(const T &value) { return emplace_back(value); }
    MAY_ALLOC bool push_back(T &&value) { return emplace_back<T>(std::move(value)); }
    /// @}

    /// Inserts an element to the beginning of this deque. Reallocates first if `full()` is `true`.
    /// @returns true if the element was inserted, false if the deque is full and memory reallocation failed.
    ///
    /// If this returns `true`, all iterators (including the `end()` iterator) and all references to the elements are invalidated.
    /// @{
    MAY_ALLOC bool push_front(const T &value) { return emplace_front(value); }
    MAY_ALLOC bool push_front(T &&value) { return emplace_front(std::move(value)); }
    /// @}

    /// Removes the last element.
    ////
    /// If `empty()` is `true`, or the implementation of the destructor of `T` fails, the behavior is undefined.
    /// All iterators (including the `end()` iterator) and the reference to the last element are invalidated.
    void pop_back() {
        (*this)[m_size - 1].~T();
        m_size--;
    }

    /// Removes the first element.
    ///
    /// If `empty()` is `true`, or the implementation of the destructor of `T` fails, the behavior is undefined.
    /// All iterators (including the `end()` iterator) and all references to the elements are invalidated.
    void pop_front() {
        (*this)[0].~T();
        m_head++;
        m_size--;
        if (m_head == capacity()) {
            m_head = 0;
        }
    }

    /// Clears the deque. This calls the destructors of the elements in an unspecified order.
    ///
    /// This does not affect the capacity of the deque. All iterators (including the `end()` iterator) and all references to the elements are
    /// invalidated.
    void clear() {
        for (std::size_t i = 0; i < m_size; i++) {
            (*this)[i].~T();
        }
        m_head = 0;
        m_size = 0;
    }

    /// Constructs an element in-place at the end of the deque. Reallocates first if `full()` is `true`.
    /// @return true if the element was inserted, false if the deque is full and memory reallocation failed.
    ///
    /// If this returns `true`, all iterators (including the `end()` iterator) are invalidated. If a reallocation takes
    /// places, all references to the elements are also invalidated.
    template <typename... Args>
    MAY_ALLOC bool emplace_back(Args &&...args) {
        if (full() && !grow()) {
            return false;
        }
        new (m_alloc.get(storage_index_of(m_size))) T(std::forward<Args>(args)...);
        m_size++;

        return true;
    }

    /// Constructs an element in-place at the beginning of the deque.
    /// @return true if the element was inserted, false if the deque is full and memory reallocation failed.
    ///
    /// If this returns `true`, all iterators (including the `end()` iterator) and all references to the elements are invalidated.
    template <typename... Args>
    MAY_ALLOC bool emplace_front(Args &&...args) {
        if (full() && !grow()) {
            return false;
        }
        if (m_head == 0) {
            m_head = capacity();
        }

        new (m_alloc.get(storage_index_of(m_head - 1))) T(std::forward<Args>(args)...);
        m_head--;
        m_size++;
        return true;
    }

    /// Extends the deque with the contents of the range [begin, end). Each iterator in [begin, end) is dereferenced exactly once.
    /// @returns The true if the deque was extended, or false if allocating memory for the additional elements failed.
    ///
    /// If the operation fails after some elements were added, they are discarded. If an exception is thrown (either by the allocator or the copy
    /// constructor of T), the behavior is undefined.
    template <typename Iter>
    MAY_ALLOC bool extend(Iter begin, Iter end) {
        std::size_t additional = 0;
        for (; begin != end; begin++) {
            if (!push_back(*begin)) {
                while (additional-- > 0) {
                    pop_back();
                }

                return false;
            }
            additional++;
        }
        return true;
    }

  private:
    friend class SmallDequeIterator<T, true, true>;
    friend class SmallDequeIterator<T, true, false>;
    friend class SmallDequeIterator<T, false, true>;
    friend class SmallDequeIterator<T, false, false>;
    struct Alloc {
        Alloc() : ptr(nullptr), cap(0) {}
        Alloc(std::size_t capacity)
            : ptr(capacity > 0 ? allocator_s<T>::get().template allocate<T>(capacity) : nullptr), cap(ptr == nullptr ? 0 : capacity) {}
        Alloc(T *ptr, std::size_t capacity) : ptr(ptr), cap(capacity) {}
        ~Alloc() { deallocate(); }

        Alloc(const Alloc &) = delete;
        Alloc &operator=(const Alloc &) = delete;

        Alloc(Alloc &&other) : Alloc(other.ptr, other.cap) {
            other.ptr = nullptr;
            other.cap = 0;
        }
        Alloc &operator=(Alloc &&other) {
            deallocate();
            ptr = other.ptr;
            cap = other.cap;
            other.forget();
            return *this;
        }

        constexpr std::size_t capacity() const { return cap; }

        bool grow(std::size_t new_cap) {
            if (new_cap > cap) {
                if (ptr != nullptr && allocator_s<T>::get().extend(ptr, cap, new_cap)) {
                    cap = new_cap;
                    return true;
                }
                return false;
            } else {
                return true;
            }
        }

        constexpr const T *get(std::size_t index) const { return ptr + index; }
        constexpr T *get(std::size_t index) { return ptr + index; }

        constexpr const T *raw() const { return ptr; }
        constexpr T *raw() { return ptr; }

      private:
        void deallocate() {
            if (ptr != nullptr) {
                allocator_s<T>::get().deallocate(ptr, cap);
            }
        }
        void forget() {
            ptr = nullptr;
            cap = 0;
        }

        T *ptr;
        std::size_t cap;
    };

    /// Double capacity every time for amortized constant-time complexity.
    /// If the capacity cannot be doubled, this tries to reserve only one additional element instead.
    MAY_ALLOC bool grow() { return reserve(std::max(static_cast<size_t>(1), capacity())) || reserve(1); }

    /// Undefined behavior if `index > size()`.
    constexpr std::size_t storage_index_of(std::size_t index) const {
        std::size_t storage_index = m_head + index;
        if (storage_index >= capacity()) {
            return storage_index - capacity();
        } else {
            return storage_index;
        }
    }

    Alloc m_alloc;
    std::size_t m_head;
    std::size_t m_size;
};

#endif