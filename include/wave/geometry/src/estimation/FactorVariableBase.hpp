/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_FACTORVARIABLEBASE_HPP
#define WAVE_GEOMETRY_FACTORVARIABLEBASE_HPP

namespace wave {

class FactorVariableBase {
 public:
    /** Returns a pointer to the variable's underlying double array. */
    virtual double *data() noexcept = 0;

    /** Returns the number of elements in the variable's underlying double array. */
    virtual std::size_t size() const noexcept = 0;

    virtual ~FactorVariableBase() = default;
};

}  // namespace wave

#endif  // WAVE_GEOMETRY_FACTORVARIABLEBASE_HPP
