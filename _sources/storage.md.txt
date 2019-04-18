# Storage and `auto`

`wave_geometry` is an expression template library, meaning operations such as `a + b` and
`a.norm()` return placeholder _expression objects_ instead of values.
These objects are sometimes called “invisible proxies” because they copy the interface of
the value they represent.
Our library is built on top of Eigen, a widely used expression template library.
[Eigen's documentation](https://eigen.tuxfamily.org/dox/TopicInsideEigenExample.html)
gives a pretty good introduction to the idea of expression templates and their benefits.

Eigen's [“common pitfalls”](https://eigen.tuxfamily.org/dox/TopicPitfalls.html) page, on
the other hand, gives an idea of what can go wrong with expression templates. It explains
why it can be dangerous to use the `auto` keyword with Eigen expressions.

While our library uses Eigen internally and it is easy to convert values to and from Eigen
objects, `wave_geometry` expressions are not Eigen expressions. We designed
`wave_geometry` to allow using `auto` as a variable initializer.

## The problem with `auto`

Scott Meyers describes the `auto` problem in Item 6 of _Effective Modern C++_:

> As a general rule, “invisible” proxy classes don’t play well with `auto`. Objects of
> such classes are often not designed to live longer than a single statement, so creating
> variables of those types tends to violate fundamental library design assumptions.

That rule is true for Eigen. Usually, Eigen makes a simple choice about how to store
operands: If the operand is a plain leaf (such as `Matrix3d`), it is stored by reference.
If the operand is a lightweight expression (such as a `Product`), it is stored by value.
Trouble occurs when Eigen stores a reference to a temporary object. For example,

```cpp
Eigen::Matrix3d A = /*...*/;
auto expr = A * Eigen::Vector3d{0., 0., 1.};
Eigen::Vector3d result = expr;
```

produces undefined behaviour because `Eigen::Vector3d{...}` is a temporary object which is
destroyed after `expr` is initialized. Since `expr` holds references to both its operands,
it is left with an invalid (“dangling”) reference.



## Why it's OK to use `auto` with wave_geometry

`wave_geometry` expressions _are_ “designed to live longer than a single statement.”
It is safe to write:

```cpp
wave::RotationMd a = /*...*/;
auto expr = a * wave::Translationd{0., 0., 1.};
wave::Translationd result = expr.eval();
Eigen::Matrix3d J_a = expr.jacobian(a);
```

Why? `wave_geometry` stores operands by value if they were received as [rvalues] (for
example, by reference to a temporary). In this example, `expr` has type `Rotate<RotationMd
&, Translationd &&>`, indicating the left operand is stored by reference and the right
operand is stored by value. The expression type is chosen by the invoked `operator*`. The
user typically does not need to worry about the specific type of the expression—only that
it represents (in this example) a rotation.

[rvalues]: https://en.cppreference.com/w/cpp/language/value_category

Operands received by reference are stored by reference (even lightweight expressions).
Note that for `wave_geometry` expressions, `Expr<U>` has the same meaning as `Expr<U &&>`;
we add `&&` for leaves to signal that there is a difference from Eigen.

While it is safe to use `auto` as a variable initializer, it is not always safe to
_return_ expression objects from a function with `auto` return type (introduced in C++14).
If the expression contains references to non-static local variables inside the function,
the references become invalid when the function returns.
