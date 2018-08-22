#!/bin/bash
set -e  # exit on first error

# Where to put the HTML output
: ${DOCS_BUILD_DIR:=build}

# Build docs for all tagged versions
# Always build docs on the current branch, though we only deploy from master
sphinx-versioning build -r $TRAVIS_BRANCH docs $DOCS_BUILD_DIR -- -W

# Add .nojekyll to stop GitHub Pages from excluding underscore-prefixed files
touch $DOCS_BUILD_DIR/.nojekyll

# Add the readme, which is excluded by Sphinx but wanted on gh-pages
cp docs/README.md $DOCS_BUILD_DIR
