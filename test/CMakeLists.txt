# geometry
WAVE_GEOMETRY_ADD_TYPED_TEST(rotation_test rotation_test.cpp
  RotationTest TYPES
  wave::RotationMd
  wave::RotationQd
  wave::RotationAd)

WAVE_GEOMETRY_ADD_TEST(rotation_misc_test rotation_misc_test.cpp)

WAVE_GEOMETRY_ADD_TEST(scalar_expression_test scalar_expression_test.cpp
  scalar_expression_op_test.cpp
  scalar_ref_test.cpp)

WAVE_GEOMETRY_ADD_TYPED_TEST(affine_test affine_test.cpp
  AffineTest TYPES
  wave::Pointd)

WAVE_GEOMETRY_ADD_TYPED_TEST(vector_test vector_test.cpp
  VectorTest TYPES
  wave::Translationd
  wave::Translation<Eigen::Vector3f>
  wave::RelativeRotationd
  wave::Twistd)

WAVE_GEOMETRY_ADD_TYPED_TEST(rigid_transform_test rigid_transform_test.cpp
  RigidTransformTest TYPES
  wave::RigidTransformMd
  wave::RigidTransformQd)

WAVE_GEOMETRY_ADD_TYPED_TEST(manifold_test manifold_test.cpp
  ManifoldTest TYPES
  wave::RotationMd
  wave::RotationQd
  wave::RotationAd
  wave::RigidTransformMd
  wave::RigidTransformQd)

WAVE_GEOMETRY_ADD_TYPED_TEST(homogeneous_test homogeneous_test.cpp
  HomogeneousTest TYPES
  wave::HomogeneousPointd
  wave::UnitHomogeneousPointd)

WAVE_GEOMETRY_ADD_TEST(homogeneous_manifold_test homogeneous_manifold_test.cpp)

WAVE_GEOMETRY_ADD_TEST(rvalue_expression_test rvalue_expression_test.cpp)

# benchmarks
WAVE_GEOMETRY_ADD_TEST(imu_preint_test imu_preint_test.cpp)

# core
WAVE_GEOMETRY_ADD_TEST(is_same_test is_same_test.cpp)

# util
WAVE_GEOMETRY_ADD_TEST(index_sequence_test util/index_sequence_test.cpp)
WAVE_GEOMETRY_ADD_TEST(type_list_test util/type_list_test.cpp)
WAVE_GEOMETRY_ADD_TEST(util_cross_matrix util/cross_matrix_test.cpp)
WAVE_GEOMETRY_ADD_TEST(identity_matrix_test util/identity_matrix_test.cpp)
WAVE_GEOMETRY_ADD_TEST(util_math_test util/math_test.cpp)

# dynamic
WAVE_GEOMETRY_ADD_TEST(dynamic_expression_test.cpp dynamic_expression_test.cpp)

# compound expressions
WAVE_GEOMETRY_ADD_TEST(compound_test compound_test.cpp)

# estimation
WAVE_GEOMETRY_ADD_TYPED_TEST(noise_test estimation/noise_test.cpp
  NoiseTest TYPES
  wave::RotationQd
  wave::Translationd)

WAVE_GEOMETRY_ADD_TEST(factor_test estimation/factor_test.cpp)
