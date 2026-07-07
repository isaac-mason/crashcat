import { compilecat } from 'compilecat/rolldown';
import MagicString from 'magic-string';
import filesize from 'rollup-plugin-filesize';

function stripDebug() {
    const re = /\bassert\s*\((?:[^()]*|\((?:[^()]*|\([^()]*\))*\))*\)\s*;?/g;
    return {
        name: 'strip-debug',
        transform(code, id) {
            if (!id.endsWith('.ts') && !id.endsWith('.js')) return null;
            if (!code.includes('assert(')) return null;
            // Remove each `assert(...)` via magic-string so we hand back a real
            // source map (original → stripped). Returning `map: null` here would
            // break the chain: the next transform (compilecat) maps from
            // this stripped code, and rollup can't compose back to the original.
            const s = new MagicString(code);
            re.lastIndex = 0;
            for (let m = re.exec(code); m !== null; m = re.exec(code)) {
                s.remove(m.index, m.index + m[0].length);
            }
            return { code: s.toString(), map: s.generateMap({ hires: true }) };
        },
    };
}

export default [
    {
        input: './src/index.ts',
        external: [],
        output: [
            {
                file: 'dist/index.js',
                format: 'es',
                sourcemap: true,
                exports: 'named',
            },
        ],
        plugins: [
            stripDebug(),
            compilecat({
                sourcemap: true,
                
                // Scope: crashcat's own src + the mathcat package (so @optimize
                // functions can inline mathcat vec3/etc.). Nothing else in
                // node_modules is read.
                include: [/\/src\//, /\/node_modules\/mathcat\//],
            }),
            filesize(),
        ],
    },
    {
        input: './three/index.ts',
        external: ['crashcat', 'three'],
        output: [
            {
                file: 'dist/three.js',
                format: 'es',
                sourcemap: true,
                exports: 'named',
            },
        ],
        plugins: [stripDebug(), filesize()],
    },
];
