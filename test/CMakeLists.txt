IF(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # Disable excessive warning from clang (see https://github.com/dealii/dealii/issues/3705)
  # from templated test fixtures
  ADD_COMPILE_OPTIONS(-Wno-undefined-var-template)
ENDIF()

# geometry
WAVE_ADD_TEST(rotation_test rotation_test.cpp)
WAVE_ADD_TEST(vector_expression_test vector_expression_test.cpp)
WAVE_ADD_TEST(rvalue_expression_test rvalue_expression_test.cpp)
WAVE_ADD_TEST(rigid_transform_test rigid_transform_test.cpp)
WAVE_ADD_TEST(manifold_test manifold_test.cpp)

# benchmarks
WAVE_ADD_TEST(imu_preint_test imu_preint_test.cpp)

# core
WAVE_ADD_TEST(is_same_test is_same_test.cpp)

# util
WAVE_ADD_TEST(index_sequence_test util/index_sequence_test.cpp)
WAVE_ADD_TEST(type_list_test util/type_list_test.cpp)
WAVE_ADD_TEST(util_cross_matrix util/cross_matrix_test.cpp)
WAVE_ADD_TEST(identity_matrix_test util/identity_matrix_test.cpp)
