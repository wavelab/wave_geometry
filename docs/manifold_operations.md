# Manifold operations

`wave_geometry` includes operations on `$SO(3)$`, the Lie group of 3D rotations,
and `$SE(3)$`, the group of 3D rigid transformations.

```eval_rst

.. csv-table:: Supported operations
   :header: Operation, "", Code
   :widths: 15, 10, 30
    
    Sum, :math:`\mathbf{v}+\mathbf{v}`, ``v + v``
    Difference, :math:`\mathbf{v}-\mathbf{v}`, ``v - v``
    Negative, :math:`-\mathbf v`, ``-v``
    Scalar multiplication, :math:`a\mathbf v`, ``a * v``
    Scalar division, :math:`\mathbf v/a`, ``v / a``
    Composition, :math:`\mathbf \Phi \circ \mathbf \Phi`, ``R * R``
    Inverse, :math:`\mathbf \Phi^{-1}`, ``inverse(R)``
    Coordinate map, :math:`\mathbf \Phi (\mathbf p)`, ``R * p``
    Exponential map, :math:`\exp(\mathbf \varphi)`, ``exp(w)``
    Logarithmic map, :math:`\log(\mathbf \Phi)`, ``log(R)``
    Manifold plus, :math:`\mathbf\Phi \boxplus \mathbf\varphi = \exp(\mathbf\varphi) \circ \mathbf\Phi`, ``R + w``
    Manifold minus, :math:`\mathbf\Phi_1 \boxminus \mathbf\Phi_2 = \log(\mathbf\Phi_1 \circ \mathbf\Phi_2^{-1})`, ``R - R``

```

Above, `$\mathbf  \Phi$` or `R` represents a Lie group element,
`$\mathbf \varphi$`  or `w` represents a Lie algebra element,
`$\mathbf p$` or `p` represents a translation,
`$a$` or `a` represents a scalar,
and `$\mathbf v$` or `v` represents any element of `$ \mathbb R^n $`, `$so(3)$` or `$se(3)$`.

