# Automatic differentiation

Any expression can be differentiated with respect to its variables:

```cpp
wave::RotationMd R = wave::RotationMd::Random();
wave::Translationd p1 = wave::Translationd::Random();
wave::Translationd p2 = R * p1;
Eigen::Matrix3d J_p2_wrt_R = (R * p1).jacobian(R);
```

Above, `J_p2_wrt_R` is the 3x3 Jacobian of `p2 = R * p1` with respect to small changes in `R`. wave_geometry computes *local Jacobians*, which are independent of the choice of parametrization for `R` (e.g., rotation matrix or quaternion), and are useful for on-manifold optimization.

It is possible (and more efficient) to combine evaluation and multiple Jacobian calculations:

```cpp
auto [p2, J_p2_wrt_R, J_p2_wrt_p1] = (R * p1).evalWithJacobians(R, p1);
```

We can also get all Jacobians at once by providing no arguments:

```cpp
auto [p2, J_p2_wrt_R, J_p2_wrt_p1] = (R * p1).evalWithJacobians();
```

Currently, the call `.evalWithJacobians(R, p1)` uses forward-mode automatic differentiation while `.evalWithJacobians()` uses reverse mode; however, future versions of wave_geometry may simply choose the fastest mode for the arguments given.

wave_geometry's expression template-based autodiff algorithm produces efficient code which runs nearly as fast as (or in some cases, just as fast as) hand-optimized code for manually-derived derivatives.
