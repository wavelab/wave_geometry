/**
 * @file Definitions of dynamic expressions
 */

#ifndef WAVE_GEOMETRY_DYNAMIC_HPP
#define WAVE_GEOMETRY_DYNAMIC_HPP

#include "core.hpp"

namespace wave {

template <typename Leaf>
class DynamicBase;

template <typename WrappedType>
struct Dynamic;

template <typename Leaf>
class Proxy;

template <typename Leaf>
class RefProxy;

}  // namespace wave

#include "src/dynamic/DynamicBase.hpp"
#include "src/dynamic/Dynamic.hpp"
#include "src/dynamic/Proxy.hpp"
#include "src/dynamic/RefProxy.hpp"

#endif  // WAVE_GEOMETRY_DYNAMIC_HPP
