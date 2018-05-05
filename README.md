
# A manifold geometry library for robotics

[![Build Status](https://travis-ci.com/wavelab/wave_geometry.svg?branch=master)](https://travis-ci.com/wavelab/wave_geometry)

**wave_geometry** is a header-only C++ library for working with rotations and transformations in robotics and computer vision applications. It differs from similar libraries by offering:

* Fast operations using expression templates
* Fast on-manifold automatic differentiation
* Compile-time coordinate frame semantics checking

The code is available under the [MIT License](LICENSE).

## Features

### Manifold operations

`wave_geometry` includes operations on SO(3), the Lie group of 3D rotations, and SE(3), the group of 3D rigid transformations.

#### Supported operations

| Operation  |       | Code |
| :---       |   :---:   | :---: |
| Sum | <img src="https://latex.codecogs.com/png.latex?\mathbf{v}&plus;\mathbf{v}" title="\mathbf{v}+\mathbf{v}" /> | `v + v` |
| Difference | <img src="https://latex.codecogs.com/png.latex?\mathbf{v}-\mathbf{v}" title="\mathbf{v}-\mathbf{v}" /> | `v - v` |
| Negative  | <img src="https://latex.codecogs.com/png.latex?-\mathbf&space;v" title="-\mathbf v" /> | `-v` |
| Composition | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\Phi&space;\circ&space;\mathbf&space;\Phi" title="\mathbf \Phi \circ \mathbf \Phi" /> | `R * R` |
| Inverse | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\Phi^{-1}" title="\mathbf \Phi^{-1}" /> | `inverse(R)` |
| Coordinate map | <img src="https://latex.codecogs.com/png.latex?\mathbf&space;\Phi&space;(\mathbf&space;p)" title="\mathbf \Phi (\mathbf p)" /> | `R * p` |
| Exponential map | <img src="https://latex.codecogs.com/png.latex?\exp(\mathbf\varphi)" title="\exp(\mathbf \varphi)" /> | `exp(w)` |
| Logarithmic map | <img src="https://latex.codecogs.com/png.latex?\log(\mathbf&space;\Phi)" title="\log(\mathbf \Phi)" /> | `log(R)` |
| Manifold plus | <img src="https://latex.codecogs.com/png.latex?\mathbf\Phi\boxplus\mathbf\varphi=\exp(\mathbf\varphi)\circ\mathbf\Phi" title="\mathbf\Phi \boxplus \mathbf\varphi = \exp(\mathbf\varphi) \circ \mathbf\Phi" /> | `R + w` |
| Manifold minus | <img src="https://latex.codecogs.com/png.latex?\mathbf\Phi_1\boxminus\mathbf\Phi_2=\log(\mathbf\Phi_1\circ\mathbf\Phi_2^{-1})" title="\mathbf\Phi_1 \boxminus \mathbf\Phi_2 = \log(\mathbf\Phi_1 \circ \mathbf\Phi_2^{-1})" /> | `R - R` |

Above, <img src="https://latex.codecogs.com/png.latex?\mathbf\Phi" alt="Phi" /> or `R` represents a Lie group element, <img src="https://latex.codecogs.com/png.latex?\mathbf\varphi" alt="small phi" />  or `w` represents a Lie algebra element, <img src="https://latex.codecogs.com/png.latex?\mathbf{p}" alt="p" /> or `p` represents a translation, and <img src="https://latex.codecogs.com/png.latex?\mathbf{v}" alt="v" /> or `v` represents any element of R^3, so(3) or se(3).

### Automatic differentiation

Any expression can be differentiated with respect to its variables:

```cpp
wave::RotationMd R = wave::RotationMd::Random();
wave::Translationd p1 = wave::Translationd::Random();
wave::Translationd p2 = R * p1;
Eigen::Matrix3d J_p2_wrt_R = (R * p1).jacobian(R);
```

Above, `J_p2_wrt_R` is the 3x3 Jacobian of `p2 = R * p1` with respect to small changes in `R`. `wave_geometry` computes *local Jacobians*, which are independent of the choice of parametrization for `R` (e.g., rotation matrix or quaternion), and are useful for on-manifold optimization.

It is possible (and more efficient) to combine evaluation and multiple Jacobian calculations:

```cpp
// Using C++17
auto [p2, J_p2_wrt_R, J_p2_wrt_p1] = (R * p1).evalWithJacobians(R, p1);

// Or, using C++11
wave::Translationd p2;
Eigen::Matrix3d J_p2_wrt_R, J_p2_wrt_p1;
std::tie(p2, J_p2_wrt_R, J_p2_wrt_p1) = (R * p1).evalWithJacobians(R, p1);
```

We can also get all Jacobians at once by providing no arguments:

```cpp
auto [p2, J_p2_wrt_R, J_p2_wrt_p1] = (R * p1).evalWithJacobians();
```

Currently, the call `.evalWithJacobians(R, p1)` uses forward-mode automatic differentiation while `.evalWithJacobians()` uses reverse mode; however, future versions of `wave_geometry` may simply choose the fastest mode for the arguments given.

`wave_geometry`'s expression template-based autodiff algorithm produces efficient code which runs nearly as fast as (or in some cases, just as fast as) hand-optimized code for manually-derived derivatives.

## Coordinate frame semantics

Representing poses without confusion is notoriously difficult, and mistakes such as using a quantity expressed in the wrong coordinate frame have bogged down many a robotics project.  `wave_geometry` provides built-in coordinate frame semantics checking. It extends [Furgale's recommended notation](http://paulfurgale.info/news/2014/6/9/representing-robot-pose-the-good-the-bad-and-the-ugly) by putting semantic information into the language type system. 

Some examples:

| In math           | In English                | In wave_geometry          |
| :---                 |  :---                  | :---                    |
| <img src="https://latex.codecogs.com/png.latex?{}_A\mathbf{p}_{BC}" title="{}_A\mathbf{p}_{BC}" /> | The vector from frame B to frame C, expressed in frame A | `Framed<Translationd, A, B, C>` or `TranslationFd<A, B, C>`|
| <img src="https://latex.codecogs.com/png.latex?{}_A\mathbf\omega_{BC}" title="{}_A\mathbf\omega_{BC}" /> | The angular velocity of frame C with respect to frame B, expressed in frame A | `Framed<RelativeRotatiod, A, B, C> ` or `RelativeRotationFd<A, B, C>` |
| <img src="https://latex.codecogs.com/png.latex?\mathbf{R}_{AB}" title="\mathbf{R}_{AB}" /> | The rotation between frame A and frame B, such that <img src="https://latex.codecogs.com/png.latex?{}_A\mathbf{p}_{BC}={}\mathbf{R}_{AB}({}_B\mathbf{p}_{BC})" title="{}_A\mathbf{p}_{BC}={}\mathbf{R}_{AB}({}_B\mathbf{p}_{BC})" /> | `Framed<RotationMd, A, B>` or `RotationMFd<A, B>` |

Here, `A`, `B` and `C` are types which do nothing but represent a coordinate frame; more descriptive names can be used if desired.

Coordinate frame semantics checking can detect invalid operations *at compile time*. Consider this code:
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

The corrected code compiles successfully.
```
// Let's get the rotation between World and Camera (fixed)  
wave::RotationMFd<WorldFrame, CameraFrame> result = r1 * inverse(r2);
```

In addition to making code safer and more readable, using coordinate frame semantics actually makes `wave_geometry`'s autodiff faster by providing extra type information.


### Supported parametrizations
| Group                | Parametrization        | Class template          | Alias for `double` as scalar |
| :---                 |  :---                  | :---                    | :--- |  
| SO(3) rotation       | 3x3 Matrix             | `MatrixRotation<T>`     | `RotationMd` |
|                      | Quaternion             | `QuaternionRotation<T>` | `RotationQd` |
|                      | Angle-axis             | `AngleAxisRotation<T>`  | `RotationAd` |
| SE(3) transformation | 4x4 homogeneous matrix |`MatrixRigidTransform<T>`| `RigidTransformMd` |
|                      | 7-vector (quaternion + translation)              | `CompactRigidTransform` | `RigidTransformQd` |
| R^3 translation      | 3-vector               | `Translation<T>`        | `Translationd` |
| so(3) diff. rotation | 3-vector        | `RelativeRotation<T>`      | `RelativeRotationd` |
| se(3) diff. transformation | 6-vector (angular + linear)  | `Twist<T>`  | `Twistd` |

Note the template parameter `<T>` is not a scalar type, but the full type of the underlying Eigen class (for example, `Eigen::Vector3d` or `Map<Matrix<double, 3, 3, RowMajor>>`). Thus `wave_geometry` classes easily work with arbitrary Eigen Maps, memory layouts, and block expressions.

Quaternions use the conventions of `Eigen::Quaternion`: Hamilton product, storage order `x,y,z,w` (despite different order in the constructor), and homomorphic quaternion-matrix conversion: `C(q1)*C(q2) = C(q1*q2)`.


## How to install

`wave_geometry` is a header-only library, meaning no compilation is required to use it in your project. Put the `include` directory into your compiler's search path, and write

```
#include <wave/geometry/geometry.hpp>
```

CMake install and link commands will be added soon.

### Dependencies

`wave_geometry` requires:
  * an existing installation of [Eigen](http://eigen.tuxfamily.org) 3.3.2 or above
  * an existing installation of Boost (only the header-only Optional library is used)
  * a modern C++11 compiler (tested on GCC 5.4, clang 4.0)

### Development status

`wave_geometry` is in the initial development stage and the API may change at any time.


## More information
The library is described in a conference paper:
```
@inproceedings{koppel2018manifold,
  title={Manifold Geometry with Fast Automatic Derivatives and Coordinate Frame Semantics Checking in {C++}},
  author={Koppel, Leonid and Waslander, Steven L.},
  booktitle={15th Conference on Computer and Robot Vision (CRV)},
  year={2018},
  note = {to be published}
}
```

