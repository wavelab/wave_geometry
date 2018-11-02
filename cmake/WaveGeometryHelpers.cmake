# - Functions and macros used in libwave

INCLUDE(CMakeParseArguments)
INCLUDE(GNUInstallDirs)

# wave_geometry_add_test: Add a gtest target
#
# WAVE_GEOMETRY_ADD_TEST(Name [DISABLED] src1 [src2...])
#
# The test will be added to the tests run by "make test", unless DISABLED is
# given. It will be linked against the needed gtest libraries. Any other links
# can be made separately with the target_link_libraries command.
FUNCTION(WAVE_GEOMETRY_ADD_TEST NAME)

    # Define the arguments this function accepts
    SET(options DISABLED)
    SET(one_value_args "")
    SET(multi_value_args "")
    CMAKE_PARSE_ARGUMENTS(WAVE_GEOMETRY_ADD_TEST
        "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    # Build the test executable using the given sources
    ADD_EXECUTABLE(${NAME} ${WAVE_GEOMETRY_ADD_TEST_UNPARSED_ARGUMENTS})

    # Link gtest libraries including one providing main()
    TARGET_LINK_LIBRARIES(${NAME} wave_geometry GTest::Main)

    # Put the test executable in the tests/ directory
    SET_TARGET_PROPERTIES(${NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests)

    # Build this test on "make tests"
    ADD_DEPENDENCIES(tests ${NAME})

    IF(NOT WAVE_GEOMETRY_ADD_TEST_DISABLED)
        # Add the executable as a test, so it runs with "make test"
        ADD_TEST(NAME ${NAME} COMMAND ${NAME})
    ENDIF()

ENDFUNCTION(WAVE_GEOMETRY_ADD_TEST)

# wave_geometry_add_typed_test: Add a gtest target, with a macro for the type under test
#
# WAVE_GEOMETRY_ADD_TYPED_TEST(Name src TestCase TYPES t1 [t2...] [DISABLED])
#
# For each given type name, a test will be added with WAVE_GEOMETRY_ADD_TEST using a
# generated file which uses gtest's INSTANTIATE_TYPED_TEST_CASE_P macro. The TEST_CASE
# argument must match the gtest test case name in the source file.
#
# This helper lets us separate compilation of typed tests into multiple translation units
# without manually making separate source files for each instantiation.
#
# This function is limited to tests with a single source file.
FUNCTION(WAVE_GEOMETRY_ADD_TYPED_TEST NAME SOURCE_FILE TEST_CASE)
    # Define the arguments this function accepts
    SET(options "")  # We pass on the DISABLED option as an unparsed argument
    SET(one_value_args "")
    SET(multi_value_args TYPES)
    CMAKE_PARSE_ARGUMENTS(wave_geometry_add_typed_test
      "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

    IF(NOT wave_geometry_add_typed_test_TYPES)
        MESSAGE(FATAL_ERROR "At least one value must be given for TYPES")
    ENDIF()

    SET(generated_sources "")

    FOREACH(type_name ${wave_geometry_add_typed_test_TYPES})
        STRING(REPLACE "::" "_" replaced_name ${type_name})
        STRING(REGEX REPLACE "[<>]" "_" replaced_name ${replaced_name})
        SET(generated_file "${NAME}_${replaced_name}.cpp")

        SET(TEST_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${SOURCE_FILE}")
        SET(WAVE_GEOMETRY_TYPE_UNDER_TEST ${type_name})
        SET(TEST_CASE_PREFIX "${replaced_name}")
        SET(TEST_CASE_NAME "${TEST_CASE}")
        CONFIGURE_FILE("${PROJECT_SOURCE_DIR}/test/instantiate_typed_test.cpp.in"
            "${generated_file}" @ONLY)
        LIST(APPEND generated_sources "${generated_file}")
    ENDFOREACH()


    WAVE_GEOMETRY_ADD_TEST(${NAME} ${generated_sources}
      ${wave_geometry_add_typed_test_UNPARSED_ARGUMENTS})

ENDFUNCTION(WAVE_GEOMETRY_ADD_TYPED_TEST)


#     wave_geometry_add_benchmark: Add a target which links against google benchmark
#
# WAVE_GEOMETRY_ADD_BENCHMARK(Name src1 [src2...])
#
# The target will be added to the tests run by "make benchmark". It will be
# linked against the needed libraries. Any other links can be made separately
# with the target_link_libraries command.
FUNCTION(WAVE_GEOMETRY_ADD_BENCHMARK NAME)
    # Build the executable using the given sources
    ADD_EXECUTABLE(${NAME} ${ARGN})

    TARGET_LINK_LIBRARIES(${NAME} wave_geometry benchmark::benchmark GTest::GTest)

    # Put the executable in the benchmarks/ directory
    SET_TARGET_PROPERTIES(${NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/benchmarks)

    # Add a ctest that will not be run by default.
    # To run it the command `ctest -C benchmark -L benchmark` must be used.
    # (Setting -C excludes it from the default set, and setting -L excludes the
    # default tests from it). Note the target `benchmark` runs this command.
    ADD_TEST(NAME ${NAME} COMMAND ${NAME} CONFIGURATIONS benchmark)
    SET_TESTS_PROPERTIES(${NAME} PROPERTIES LABELS benchmark)

    # Build this target on "make benchmarks"
    ADD_DEPENDENCIES(benchmarks ${NAME})
ENDFUNCTION(WAVE_GEOMETRY_ADD_BENCHMARK)
