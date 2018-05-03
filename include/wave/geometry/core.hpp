/**
 * @file
 */

#ifndef WAVE_GEOMETRY_CORE_HPP
#define WAVE_GEOMETRY_CORE_HPP

// Tick library for traits checking
#include <tick/trait_check.h>
#include <tick/requires.h>

// Standalone utilities
#include "src/util/macros.hpp"
#include "src/util/meta/template_helpers.hpp"
#include "src/util/meta/index_sequence.hpp"
#include "src/util/meta/type_list.hpp"
#include "src/util/math/math.hpp"
#include "src/util/math/IdentityMatrix.hpp"

// Forward declarations and standalone type traits
#include "src/core/forward_declarations.hpp"
#include "src/core/traits/type_traits.hpp"
#include "src/core/storage/RefSelector.hpp"

// Evaluation helpers
#include "src/core/functions/IsSame.hpp"
#include "src/core/functions/IsSameType.hpp"
#include "src/core/functions/AddConversions.hpp"
#include "src/core/functions/PrepareExpr.hpp"
#include "src/core/functions/Evaluator.hpp"
#include "src/core/functions/PrepareOutput.hpp"
#include "src/core/functions/JacobianEvaluator.hpp"
#include "src/core/functions/TypedJacobianEvaluator.hpp"
#include "src/core/functions/ReverseJacobianEvaluator.hpp"

// Storage and traits bases
#include "src/core/storage/UnaryExpression.hpp"
#include "src/core/storage/BinaryExpression.hpp"
#include "src/core/storage/LeafExpression.hpp"
#include "src/core/traits/traits_bases.hpp"

// Expressions
#include "src/core/op/Convert.hpp"

// Generic expressions
#include "src/core/base/ExpressionBase.hpp"

#endif  // WAVE_GEOMETRY_CORE_HPP
