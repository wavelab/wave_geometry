#include "manifold_test.hpp"

// Run each test for framed and unframed variants of these types
using LeafTypes = test_types_list<wave::RigidTransformMd, wave::RigidTransformQd>;

INSTANTIATE_TYPED_TEST_CASE_P(Transforms, ManifoldTest, LeafTypes);
