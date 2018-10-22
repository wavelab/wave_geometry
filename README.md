
# A manifold geometry library for robotics

[![Build Status](https://travis-ci.com/wavelab/wave_geometry.svg?branch=master)](https://travis-ci.com/wavelab/wave_geometry)
[![Documentation](https://img.shields.io/website-online-offline-blue-red/http/wavelab.github.io%2Fwave_geometry.svg?label=docs)](https://wavelab.github.io/wave_geometry/)

**wave_geometry** is a header-only C++14 library for working with rotations and transformations in robotics and computer vision applications. It differs from similar libraries by offering:

* Fast operations using expression templates
* Fast on-manifold automatic differentiation
* Compile-time coordinate frame semantics checking

Source code is available under the [MIT License](LICENSE).
Documentation is [available online](https://wavelab.github.io/wave_geometry/).


## Features

### Manifold operations <sup>*[Docs][manifold operations]*</sup>

`wave_geometry` includes operations on SO(3), the Lie group of 3D rotations, and SE(3),
 the group of 3D rigid transformations.
 
### Automatic differentiation <sup>*[Docs][Automatic differentiation]*</sup>
Any expression can be differentiated with respect to its variables.

```cpp
wave::RotationMd R = wave::RotationMd::Random();
wave::Translationd p1 = wave::Translationd::Random();
wave::Translationd p2 = R * p1;

// Using C++17
auto [p2, J_p2_wrt_R, J_p2_wrt_p1] = (R * p1).evalWithJacobians(R, p1);

// Or, using C++14
wave::Translationd p2;
Eigen::Matrix3d J_p2_wrt_R, J_p2_wrt_p1;
std::tie(p2, J_p2_wrt_R, J_p2_wrt_p1) = (R * p1).evalWithJacobians(R, p1);
```

`wave_geometry` computes *local Jacobians*, which are independent of the choice of parametrization for `R` (e.g., rotation matrix or quaternion), and are useful for on-manifold optimization.

### Coordinate frame semantics <sup>*[Docs][Coordinate frame semantics]*</sup>
Representing poses without confusion is notoriously difficult, and mistakes such as using a quantity expressed in the wrong coordinate frame have bogged down many a robotics project.
`wave_geometry` provides built-in coordinate frame semantics checking.
 
 
Coordinate frame semantics checking can detect invalid operations *at compile time*.
Consider this example:
```cpp
struct BodyFrame;  
struct CameraFrame;  
struct WorldFrame;  
  
wave::RotationMFd<WorldFrame, BodyFrame> r1;  
wave::RotationMFd<CameraFrame, WorldFrame> r2;  
  
// Let's get the rotation between World and Camera (maybe)
wave::RotationMFd<WorldFrame, CameraFrame> result = r1 * r2;
```
This code has a mistake, but is saved by semantics checking. When we try to compile it, we get the following error message (from clang 5, not showing colours):
```
In file included from /.../example.cpp:1:
In file included from /.../wave/geometry/geometry.hpp:45:
/.../wave/geometry/src/geometry/op/Compose.hpp:22:5: error: static_assert failed "Adjacent frames do not match"
    static_assert(std::is_same<RightFrameOf<Lhs>, LeftFrameOf<Rhs>>(),
    ^             ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/.../example.cpp:31:60: note: in instantiation of template class 'wave::Compose<wave::Framed<wave::MatrixRotation<Eigen::Matrix<double, 3, 3, 0, 3, 3> >, WorldFrame, BodyFrame>, wave::Framed<wave::MatrixRotation<Eigen::Matrix<double, 3, 3, 0, 3, 3> >, CameraFrame, WorldFrame> >' requested here
    wave::RotationMFd<WorldFrame, CameraFrame> result = r1 * r2;
                                                           ^
```

The corrected code below compiles successfully:
```
// Let's get the rotation between World and Camera (fixed)  
wave::RotationMFd<WorldFrame, CameraFrame> result = r1 * inverse(r2);
```

<!-- Documentation links -->
[Manifold operations]: https://wavelab.github.io/wave_geometry/manifold_operations.html
[Automatic differentiation]: https://wavelab.github.io/wave_geometry/autodiff.html
[Coordinate frame semantics]: https://wavelab.github.io/wave_geometry/frame_semantics.html


## Installation

`wave_geometry` is a header-only library, meaning no prior compilation is required to use
it in your project. It requires Eigen 3.3.2 or above, Boost 1.58 or above
(header-only libraries), and a C++14 compiler.

`wave_geometry` can be installed with CMake:
see [installation instructions](https://wavelab.github.io/wave_geometry/install.html).

The library is in the initial development stage and the API may change at any time.
See [CHANGELOG.md](CHANGELOG.md) for recent changes.
