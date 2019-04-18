# Dynamic expressions

Sometimes, we want to do something not possible with pure expression templates: decide the
structure of an expression at runtime, make heterogenous vectors of expressions, or spread
compilation across translation units. Specifically, we want expressions with _dynamic_
polymorphism. We may also want to store objects on the heap and share their ownership.

These tasks are possible with `wave_geometry`'s `dynamic` module.
This module comes in a separate header: 

```cpp
#include <wave/geometry/geometry.hpp>
#include <wave/geometry/dynamic.hpp>
```

## Proxy expressions

For any leaf type `L`, the class `Proxy<L>` wraps a heap-allocated expression that will
evaluate to `L`, but whose concrete type is determined at runtime. Proxy objects hold a
`shared_ptr` to the expression. Consider the example:

```cpp
wave::Proxy<wave::Translationd> t = wave::Translationd::Random();
wave::Proxy<wave::RotationMd> R1 = wave::RotationMd::Random();
wave::Proxy<wave::RotationMd> R2 = wave::RotationMd::Random();

wave::Proxy<wave::Translationd> result = R1 * t;
if (R2_is_needed) {
    result = R2 * result;
}
return result;
```

This example above can equivalently be written as:

```cpp
auto t = makeProxy(wave::Translationd::Random());
auto R1 = makeProxy(wave::RotationMd::Random());
auto R2 = makeProxy(wave::RotationMd::Random());

auto result = makeProxy(R1 * t);
if (R2_is_needed) {
    result = R2 * result;
}
return result;
```

The first three lines construct random-valued leaf objects on the heap and
give their ownership to `Proxy` objects. `result` is a `Proxy<Translationd>`, the same
type as `t`, but it points to a `Rotate` expression of other `Proxy` objects. We can
safely `return result` knowing the dynamic expression tree it points to on the heap will
remain, because its lifetime is controlled by `shared_ptr`s. We can assign to proxies with
shared pointer semantics.

`Proxy<L>` is analogous to GTSAM's `Expression<L>`.


Proxies are fully compatible with "regular" expressions: they can be evaluated using
`eval()` or by assignment to a leaf, they can be used in static expressions, and they can
hold arbitrary expressions. Caveat: it is usually a bad idea put an expression that
contains references to stack objects in a `Proxy`. If those objects are destroyed before
the `Proxy`, it will be left with dangling references.


Under the hood, creating a `Proxy` constructs a `Dynamic` expression. For example, the line

```cpp 
auto result = makeProxy(R1 * t);
```

puts an object of type `Dynamic<Rotate<Proxy<RotationMd>&&,Proxy<Translationd>&&>&&>` on
the heap. The resulting `Proxy<Translationd>` holds a shared pointer to that object
through its abstract base class, `DynamicBase<Translationd>`.
