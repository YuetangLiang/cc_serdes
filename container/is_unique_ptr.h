#pragma once

#include <memory>
#include <type_traits>

namespace hps {

template <typename T>
struct is_unique_ptr : std::false_type {};

template <typename T>
struct is_unique_ptr<std::unique_ptr<T>> : std::true_type {};

}  // namespace hps
