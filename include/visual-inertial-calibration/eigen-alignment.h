// Copyright (c) George Washington University, all rights reserved.  See the
// accompanying LICENSE file for more information.

#pragma once

#include <Eigen/Core>

template <template<typename ...> class ContainerT, typename T, typename ...Args>
using aligned = ContainerT<T, Args..., Eigen::aligned_allocator<T> >;

template <typename T>
using aligned_vector = aligned<std::vector, T>;
