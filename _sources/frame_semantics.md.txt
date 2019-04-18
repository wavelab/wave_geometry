# Coordinate frame semantics

Representing poses without confusion is notoriously difficult, and mistakes such as using a quantity expressed in the wrong coordinate frame have bogged down many a robotics project. wave_geometry provides built-in coordinate frame semantics checking. It extends [Furgale's recommended notation](http://paulfurgale.info/news/2014/6/9/representing-robot-pose-the-good-the-bad-and-the-ugly) by putting semantic information into the language type system. 

Some examples:

```eval_rst

.. csv-table::
   :header: In math,  In English, In wave_geometry 
    
   :math:`{}_A\mathbf{p}_{BC}`, "The vector from frame B to frame C, expressed in frame A", "``Framed<Translationd, A, B, C>`` or ``TranslationFd<A, B, C>``"
   :math:`{}_A\mathbf\omega_{BC}`, "The angular velocity of frame C with respect to frame B, expressed in frame A", "``Framed<RelativeRotatiod, A, B, C>`` or ``RelativeRotationFd<A, B, C>``"
   :math:`\mathbf{R}_{AB}`, "The rotation between frame A and frame B, such that :math:`{}_A\mathbf{p}_{BC}={}\mathbf{R}_{AB}({}_B\mathbf{p}_{BC})`", "``Framed<RotationMd, A, B>`` or ``RotationMFd<A, B>``"

```


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

```console
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
```cpp
// Let's get the rotation between World and Camera (fixed)  
wave::RotationMFd<WorldFrame, CameraFrame> result = r1 * inverse(r2);
```

In addition to making code safer and more readable, using coordinate frame semantics actually makes wave_geometry's autodiff faster by providing extra type information.
