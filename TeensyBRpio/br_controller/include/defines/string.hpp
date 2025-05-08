
#if !defined(ARDUINO) || defined(_BR_DEBUG)
#ifndef _STRING_EXT_
#define _STRING_EXT_

#include <string>

namespace __util {
template <typename T>
constexpr bool toString = requires(const T &t) { std::to_string(t); };
} // namespace __util

template <typename T>
std::string to_string(const T &obj) {
    if constexpr (__util::toString<T>) {
        return std::to_string(obj);
    } else {
        return std::string(obj);
    }
}

#endif

#endif