include(../cmake/AddGTest.cmake)

# wave_add_test: Add a gtest target
function(WAVE_ADD_TEST NAME SOURCE_FILE)
  add_executable(${NAME} ${SOURCE_FILE})
  target_link_libraries(${NAME} wave_geometry GTest::Main)
  add_test(NAME ${NAME} COMMAND ${NAME})
endfunction(WAVE_ADD_TEST)

# Disable excessive warning from clang (see https://github.com/dealii/dealii/issues/3705)
# from templated test fixtures
add_compile_options(-Wno-undefined-var-template)

# geometry
wave_add_test(rotation_test rotation_test.cpp)
wave_add_test(vector_expression_test vector_expression_test.cpp)
wave_add_test(rvalue_expression_test rvalue_expression_test.cpp)
wave_add_test(rigid_transform_test rigid_transform_test.cpp)
wave_add_test(manifold_test manifold_test.cpp)

# benchmarks
wave_add_test(imu_preint imu_preint.cpp)

# core
wave_add_test(is_same is_same.cpp)

# util
wave_add_test(index_sequence_test util/index_sequence_test.cpp)
wave_add_test(type_list_test util/type_list_test.cpp)
wave_add_test(util_cross_matrix util/cross_matrix_test.cpp)
wave_add_test(identity_matrix_test util/identity_matrix_test.cpp)