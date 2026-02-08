#!/bin/sh

# build with rollup
npx rollup -c rollup.config.mjs

# should not find string "type: ShapeType.TRIANGLE_MESH"
if grep -q "type: ShapeType.TRIANGLE_MESH" dist/output.js; then
    echo "Error: Found string 'type: ShapeType.TRIANGLE_MESH' in output, tree shaking failed"
    exit 1
else
    echo "Success: String 'type: ShapeType.TRIANGLE_MESH' not found in output, tree shaking succeeded"
fi