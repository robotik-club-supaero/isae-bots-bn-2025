#ifndef _DEFINE_STRING_H_
#define _DEFINE_STRING_H_

#include <string>

using string_t = std::string;

namespace __util {
template <typename T>
constexpr bool toString = requires(const T &t) { std::to_string(t); };
} // namespace __util

template <typename T>
string_t to_string(const T &obj) {

    if constexpr (__util::toString<T>) {
        return std::to_string(obj);
    } else {
        return std::string(obj);
    }
}

#endif