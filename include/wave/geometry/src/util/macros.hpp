/**
 * @file
 * @author lkoppel
 */

#ifndef WAVE_GEOMETRY_MACROS_HPP
#define WAVE_GEOMETRY_MACROS_HPP

#include <Eigen/Core>

#ifdef NDEBUG
#if EIGEN_COMP_CLANG
// Based on experiments, always_inline makes clang-4.0 faster, but not GCC
#define WAVE_STRONG_INLINE EIGEN_ALWAYS_INLINE
#else
#define WAVE_STRONG_INLINE EIGEN_STRONG_INLINE
#endif
#else
// When debugging don't force inlining
#define WAVE_STRONG_INLINE inline
#endif


// We sometimes need to explicitly declare special member functions (copy constructors and
// operator=) even when they should already be defaulted, to avoid a GCC bug
// (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=63540)
// The default constructor is not included. It is not related to the bug and it is more
// clear to show the default constructor anyway.
#define WAVE_DEFAULT_COPY_AND_MOVE_FUNCTIONS(Derived) \
    Derived(const Derived &) = default;               \
    Derived(Derived &&) = default;                    \
    Derived &operator=(const Derived &) = default;    \
    Derived &operator=(Derived &&) = default;

#endif  // WAVE_GEOMETRY_MACROS_HPP
