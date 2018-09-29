#include "manifold_test.hpp"

// Run each test for framed and unframed variants of these types
using LeafTypes = test_types_list<wave::RotationMd, wave::RotationQd, wave::RotationAd>;

INSTANTIATE_TYPED_TEST_CASE_P(Rotations, ManifoldTest, LeafTypes);
