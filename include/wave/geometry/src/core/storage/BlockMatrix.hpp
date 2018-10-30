/**
 * @file
 */

#ifndef WAVE_GEOMETRY_BLOCKMATRIX_HPP
#define WAVE_GEOMETRY_BLOCKMATRIX_HPP

namespace wave {

/**
 * An Eigen matrix extended with blocks associated with two compound expressions
 */
template <typename L, typename R>
class BlockMatrix : public Eigen::Matrix<internal::common_scalar_t<L, R>,
                                         internal::traits<L>::TangentSize,
                                         internal::traits<R>::TangentSize> {
 public:
    using Base = Eigen::Matrix<internal::common_scalar_t<L, R>,
                               internal::traits<L>::TangentSize,
                               internal::traits<R>::TangentSize>;

 private:
    using LTraits = internal::traits<L>;
    using RTraits = internal::traits<R>;

    using LTangentSizes = internal::tangent_sizes<typename LTraits::TangentBlocks>;
    using RTangentSizes = internal::tangent_sizes<typename RTraits::TangentBlocks>;

    // Convenience value getter templates, used in access functions
    // Not variable templates because of gcc5 bug. Use result's ::value or rely on its
    // conversion operator to size_t.
    template <size_t I>
    using LSizeOf = tmp::integer_sequence_element<I, LTangentSizes>;

    template <size_t I>
    using RSizeOf = tmp::integer_sequence_element<I, RTangentSizes>;

    using LIndices = tmp::accumulate_index_sequence<LTangentSizes>;
    using RIndices = tmp::accumulate_index_sequence<RTangentSizes>;

    template <size_t I>
    using LIndexOf = tmp::integer_sequence_element<I, LIndices>;
    template <size_t I>
    using RIndexOf = tmp::integer_sequence_element<I, RIndices>;

 public:
    // Inherit base constructors and assignment operators
    using Base::Base;
    using Base::operator=;
    BlockMatrix() = default;

    /** Get a mutable block corresponding to the Ith component of L wrt the Jth
     * component of R */
    template <int I, int J>
    auto blockWrt() noexcept {
        static_assert(I < LIndices::size(), "Index out of range");
        static_assert(J < RIndices::size(), "Index out of range");
        return this->template block<LSizeOf<I>{}, RSizeOf<J>{}>(LIndexOf<I>::value,
                                                                RIndexOf<J>::value);
    }

    /** Get a block corresponding to the Ith component of L wrt the Jth component of R */
    template <int I, int J>
    auto blockWrt() const noexcept {
        static_assert(I < LIndices::size(), "Index out of range");
        static_assert(J < RIndices::size(), "Index out of range");
        return this->template block<LSizeOf<I>{}, RSizeOf<J>{}>(LIndexOf<I>::value,
                                                                RIndexOf<J>::value);
    }

    /** Get a mutable block of columns wrt the Jth component of R */
    template <int J>
    auto colsWrt() noexcept {
        static_assert(J < RIndices::size(), "Index out of range");
        return this->template middleCols<RSizeOf<J>{}>(RIndexOf<J>::value);
    }

    /** Get a block of columns wrt the Jth component of R */
    template <int J>
    auto colsWrt() const noexcept {
        static_assert(J < RIndices::size(), "Index out of range");
        return this->template middleCols<RSizeOf<J>{}>(RIndexOf<J>::value);
    }

    /** Get a mutable block of rows wrt the Ith component of L */
    template <int I>
    auto rowsWrt() noexcept {
        static_assert(I < LIndices::size(), "Index out of range");
        return this->template middleRows<LSizeOf<I>{}>(LIndexOf<I>::value);
    }

    /** Get a block of rows wrt the Ith component of L */
    template <int I>
    auto rowsWrt() const noexcept {
        static_assert(I < LIndices::size(), "Index out of range");
        return this->template middleRows<LSizeOf<I>{}>(LIndexOf<I>::value);
    }
};

/**
 * An Eigen DiagonalMatrix extended with blocks associated with a compound expression
 */
template <typename L>
class DiagonalBlockMatrix
    : public Eigen::DiagonalMatrix<internal::scalar_t<L>,
                                   internal::traits<L>::TangentSize> {
 public:
    using Base =
      Eigen::DiagonalMatrix<internal::scalar_t<L>, internal::traits<L>::TangentSize>;

 private:
    using LTraits = internal::traits<L>;

    using LTangentSizes = internal::tangent_sizes<typename LTraits::TangentBlocks>;

    // Convenience value getter templates, used in access functions
    // Not variable templates because of gcc5 bug. Use result's ::value or rely on its
    // conversion operator to size_t.
    template <size_t I>
    using LSizeOf = tmp::integer_sequence_element<I, LTangentSizes>;

    using LIndices = tmp::accumulate_index_sequence<LTangentSizes>;

    template <size_t I>
    using LIndexOf = tmp::integer_sequence_element<I, LIndices>;

 public:
    // Inherit base constructors and assignment operators
    using Base::Base;
    using Base::operator=;
    DiagonalBlockMatrix() = default;

    /** Get a mutable segment corresponding to the Ith component of L wrt the Jth
     * component of R */
    template <int I>
    auto segmentWrt() noexcept {
        static_assert(I < LIndices::size(), "Index out of range");
        return this->template segment<LSizeOf<I>{}>(LIndexOf<I>::value);
    }
};

/**
 * An Eigen vector extended with blocks associated with a compound expression
 */
template <typename L>
using BlockVector = BlockMatrix<L, internal::scalar_t<L>>;

}  // namespace wave

namespace Eigen {
namespace internal {

// These definitions tell Eigen to treat BlockMatrix identically to it's Matrix
// base class. See https://eigen.tuxfamily.org/dox/TopicNewExpressionType.html
template <typename L, typename R>
struct traits<wave::BlockMatrix<L, R>> : traits<typename wave::BlockMatrix<L, R>::Base> {
};

template <typename L, typename R>
struct evaluator<wave::BlockMatrix<L, R>>
    : evaluator<typename wave::BlockMatrix<L, R>::Base> {};

// Treat DiagonalBlockMatrix identically to it's DiagonalMatrix base class.
template <typename L>
struct traits<wave::DiagonalBlockMatrix<L>>
    : traits<typename wave::DiagonalBlockMatrix<L>::Base> {};

template <typename L>
struct evaluator<wave::DiagonalBlockMatrix<L>>
    : evaluator<typename wave::DiagonalBlockMatrix<L>::Base> {};

}  // namespace internal
}  // namespace Eigen

#endif  // WAVE_GEOMETRY_BLOCKMATRIX_HPP
