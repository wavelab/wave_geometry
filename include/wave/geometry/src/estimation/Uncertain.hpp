/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_UNCERTAIN_HPP
#define WAVE_GEOMETRY_UNCERTAIN_HPP

namespace wave {

/** A leaf expression paired with an uncertainty
 */
template <typename Leaf, template <typename> class NoiseModel>
class Uncertain {
    static_assert(internal::is_leaf_expression<Leaf>{} ||
                    internal::is_compound_leaf_expression<Leaf>{},
                  "Template parameter must be a leaf expression");

 public:
    Leaf value;
    NoiseModel<Leaf> noise;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_UNCERTAIN_HPP
