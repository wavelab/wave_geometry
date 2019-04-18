# Documentation

The wave_geometry docs are published to
[wavelab.github.io/wave_geometry](https://wavelab.github.io/wave_geometry).


## How to build the docs

The docs are built with Sphinx and a few extensions.
Install them as Python packages:

```
pip install sphinx sphinx_rtd_theme recommonmark sphinx-markdown-tables
```

Then, in your wave_geometry build directory:

```
cmake .. -DBUILD_DOCS=ON
make docs
```

The output is placed in `docs/html`.

The documentation published online is built using the `sphinxcontrib-versioning`
package.
