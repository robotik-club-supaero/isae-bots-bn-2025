#ifndef _DEFINE_CONSTRAINT_HPP_
#define _DEFINE_CONSTRAINT_HPP_

#include <concepts>
#include <type_traits>

template <class T, class U>
concept Derived = std::is_base_of<U, T>::value;

#endif