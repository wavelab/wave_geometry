# Manifold operations

`wave_geometry` includes operations on `$\SO3$`, the Lie group of 3D rotations,
and `$\SE3$`, the group of 3D rigid transformations.

```eval_rst
.. csv-table:: Supported manifold operations
   :header: Operation, "", Code
   :widths: 30, 20, 30
    
    Composition, :math:`\vgrp \circ \vgrp`, ``R * R``
    Inverse, :math:`\vgrp^{-1}`, ``inverse(R)``
    Coordinate map, :math:`\vgrp (\vvec)`, ``R * p``
    Exponential map, :math:`\exp(\valg)`, ``exp(w)``
    Logarithmic map, :math:`\log(\vgrp)`, ``log(R)``
    Manifold plus, :math:`\vgrp \boxplus \valg = \exp(\valg) \circ \vgrp`, ``R + w``
    Manifold minus, :math:`\vgrp_1 \boxminus \vgrp_2 = \log(\vgrp_1 \circ \vgrp_2^{-1})`, ``R - R``
```

Above, `$\vgrp$` or `R` represents a Lie group element, `$\valg$`  or `w` represents a Lie
algebra element, `$\vvec$` and `p` represents a translation. The following operations are
supported for Euclidean elements, where `$\vvecg$` or `v` represents any element of
`$\R{n}$`, `$\so3$` or `$\se3$`, and `$a$` or `a` represents a scalar:vector


```eval_rst
.. csv-table:: Supported vector operations
   :header: Operation, "", Code
   :widths: 30, 10, 50

    Sum, :math:`\vvecg+\vvecg`, ``v + v``
    Difference, :math:`\vvecg-\vvecg`, ``v - v``
    Negation, :math:`-\vvecg`, ``-v``
    Scalar multiplication, :math:`a\vvecg`, ``a * v``
    Scalar division, :math:`\vvecg/a`, ``v / a``
    Dot product, :math:`\vvecg \cdot \vvecg`, "``dot(v, a)``"
```

These operations are supported for affine points, `$\vvec$`:

```eval_rst
.. csv-table:: Supported point operations
   :header: Operation, "", Code
   :widths: 30, 10, 30

    Translation between points, :math:`\vvec - \vvec = \vvecg`, ``p - p``
    Point translation, :math:`\vvec + \vvecg = \vvec`, ``p + v``
    Point translation, :math:`\vvec - \vvecg = \vvec`, ``p - v``
```
