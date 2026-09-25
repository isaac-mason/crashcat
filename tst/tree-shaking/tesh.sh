#!/bin/sh
set -e

# entry.js registers only sphere and box. anything belonging to a shape, constraint or narrowphase
# path it never uses must not survive bundling.
npx rolldown -c rolldown.config.mjs

fail=0

# $1 = the string that must NOT appear, $2 = what its presence would mean
refute() {
    if grep -q "$1" dist/output.js; then
        echo "FAIL: found '$1' in the bundle — $2"
        fail=1
    else
        echo "ok: '$1' shaken out"
    fi
}

# $1 = the string that MUST appear, so the checks above cannot pass vacuously
require() {
    if grep -q "$1" dist/output.js; then
        echo "ok: '$1' present, as expected"
    else
        echo "FAIL: '$1' missing from the bundle — the entry or the check is wrong"
        fail=1
    fi
}

# unused shape and constraint definitions
refute "type: ShapeType.TRIANGLE_MESH" "an unregistered shape def was retained"
refute "type: ConstraintType.CONE" "an unregistered constraint def was retained"

# support fills for unregistered convex shapes. convexity is carried by the `convex` descriptor on
# each shape def, so a shape's support fill is only referenced from its own module and drops with
# it. wiring the convex pairs from a central place that names the fills directly would silently
# undo that — it cost 32 kB (+26%) on this exact entry before these checks existed.
refute "setCapsuleSupport" "capsule support was retained by a shape that is not registered"
refute "setCylinderSupport" "cylinder support was retained by a shape that is not registered"
refute "setHullSupport" "convex hull support was retained by a shape that is not registered"
refute "computeScaledShrunkHullPoints" "convex hull machinery was retained"

# the shapes that ARE registered must still be there
require "setBoxSupport"
require "setSphereSupport"

if [ "$fail" -ne 0 ]; then
    echo "tree shaking check failed"
    exit 1
fi
echo "tree shaking check passed"
