/**
 * @file
 */

#ifndef WAVE_GEOMETRY_CORE_HPP
#define WAVE_GEOMETRY_CORE_HPP

#include <utility>
#include <tuple>
#include <memory>
#include <type_traits>

// For optional, used by JacobianEvaluator
#include <boost/optional.hpp>
// Used by DynamicReverseJacobianEvaluator
#include <boost/container/flat_map.hpp>

// Tick library for traits checking
#include <tick/trait_check.h>
#include <tick/requires.h>

// Standalone utilities
#include "src/util/macros.hpp"
#include "src/util/meta/template_helpers.hpp"
#include "src/util/meta/index_sequence.hpp"
#include "src/util/meta/type_list.hpp"
#include "src/util/math/CrossMatrix.hpp"
#include "src/util/math/IdentityMatrix.hpp"
#include "src/util/math/math.hpp"
#include "src/util/math/MatrixMap.hpp"

// Forward declarations and standalone type traits
#include "src/core/forward_declarations.hpp"
#include "src/core/traits/type_traits.hpp"
#include "src/core/storage/RefSelector.hpp"

// Evaluators
#include "src/core/functions/IsSame.hpp"
#include "src/core/functions/IsSameType.hpp"
#include "src/core/functions/AddConversions.hpp"
#include "src/core/functions/PrepareExpr.hpp"
#include "src/core/functions/Evaluator.hpp"
#include "src/core/functions/PrepareOutput.hpp"
#include "src/core/functions/DynamicJacobianEvaluator.hpp"
#include "src/core/functions/JacobianEvaluator.hpp"
#include "src/core/functions/TypedJacobianEvaluator.hpp"
#include "src/core/functions/ReverseJacobianEvaluator.hpp"
#include "src/core/functions/DynamicReverseJacobianEvaluator.hpp"
#include "src/core/functions/NumericalJacobian.hpp"

// Storage and traits bases
#include "src/core/storage/UnaryStorage.hpp"
#include "src/core/storage/BinaryStorage.hpp"
#include "src/core/storage/LeafStorage.hpp"
#include "src/core/storage/CompoundLeafStorage.hpp"
#include "src/core/storage/NaryStorage.hpp"
#include "src/core/traits/traits_bases.hpp"
#include "src/core/storage/BlockMatrix.hpp"


// Expressions
#include "src/core/base/ExpressionBase.hpp"
#include "src/core/op/Convert.hpp"
#include "src/core/op/MemberAccess.hpp"

#endif  // WAVE_GEOMETRY_CORE_HPP
