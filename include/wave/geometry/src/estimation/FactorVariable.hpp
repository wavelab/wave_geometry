/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_FACTORVARIABLE_HPP
#define WAVE_GEOMETRY_FACTORVARIABLE_HPP

namespace wave {

template <typename Leaf>
class FactorVariable : public FactorVariableBase {
    static_assert(internal::is_leaf_expression<Leaf>{},
                  "Template parameter must be a leaf");
    static_assert(std::is_same<double, internal::scalar_t<Leaf>>{},
                  "Leaves used with FactorVariable must have double as scalar type");

 public:
    /** Returns a pointer to the variable's underlying double array. */
    double *data() noexcept override {
        return value_.value().data();
    }

    /** Returns the number of elements in the variable's underlying double array. */
    std::size_t size() const noexcept override {
        return value_.value().size();
    }

    const auto &value() const & {
        return value_;
    }

    auto &value() & {
        return value_;
    }

    auto value() && {
        return value_;
    }

 private:
    Leaf value_;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_FACTORVARIABLE_HPP
