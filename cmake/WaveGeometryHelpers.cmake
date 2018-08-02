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
