/**
 * @file
 */

#ifndef WAVE_GEOMETRY_GEOMETRY_HPP
#define WAVE_GEOMETRY_GEOMETRY_HPP

#include <Eigen/Geometry>

#include "core.hpp"

#include "src/util/math/CrossMatrix.hpp"

#include "src/geometry/forward_declarations.hpp"
#include "src/geometry/type_traits.hpp"

// Frames
#include "src/geometry/frames/frame_traits.hpp"
#include "src/geometry/frames/frame_cast.hpp"
#include "src/geometry/frames/Framed.hpp"

#include "src/geometry/op/CommonExpressions.hpp"

// Geometric leaves and bases
#include "src/geometry/base/CompoundBase.hpp"
#include "src/geometry/base/ScalarBase.hpp"
#include "src/geometry/base/AffineBase.hpp"
#include "src/geometry/base/VectorBase.hpp"
#include "src/geometry/base/ProjectiveBase.hpp"
#include "src/geometry/base/TransformBase.hpp"
#include "src/geometry/base/RotationBase.hpp"
#include "src/geometry/leaf/Scalar.hpp"
#include "src/geometry/leaf/MatrixRotation.hpp"
#include "src/geometry/leaf/QuaternionRotation.hpp"
#include "src/geometry/leaf/AngleAxisRotation.hpp"
#include "src/geometry/base/RelativeRotationBase.hpp"
#include "src/geometry/leaf/RelativeRotation.hpp"
#include "src/geometry/base/PointBase.hpp"
#include "src/geometry/leaf/Point.hpp"
#include "src/geometry/base/TranslationBase.hpp"
#include "src/geometry/leaf/Translation.hpp"
#include "src/geometry/base/HomogeneousPointBase.hpp"
#include "src/geometry/leaf/HomogeneousPoint.hpp"
#include "src/geometry/leaf/UnitHomogeneousPoint.hpp"
#include "src/geometry/leaf/Zero.hpp"
#include "src/geometry/leaf/Identity.hpp"
#include "src/geometry/leaf/MatrixRigidTransform.hpp"
#include "src/geometry/leaf/CompactRigidTransform.hpp"
#include "src/geometry/base/RigidTransformBase.hpp"
#include "src/geometry/base/TwistBase.hpp"
#include "src/geometry/leaf/Twist.hpp"

// Expressions
#include "src/geometry/op/Sum.hpp"
#include "src/geometry/op/Subtract.hpp"
#include "src/geometry/op/Rotate.hpp"
#include "src/geometry/op/Transform.hpp"
#include "src/geometry/op/Compose.hpp"
#include "src/geometry/op/ExpMap.hpp"
#include "src/geometry/op/LogMap.hpp"
#include "src/geometry/op/BoxPlus.hpp"
#include "src/geometry/op/BoxMinus.hpp"
#include "src/geometry/op/Minus.hpp"
#include "src/geometry/op/Scale.hpp"
#include "src/geometry/op/Product.hpp"
#include "src/geometry/op/Divide.hpp"
#include "src/geometry/op/Inverse.hpp"

#endif  // WAVE_GEOMETRY_GEOMETRY_HPP
