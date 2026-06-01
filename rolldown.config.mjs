import compilecat from 'compilecat/rolldown';
import filesize from 'rollup-plugin-filesize';

function stripDebug() {
    const re = /\bassert\s*\((?:[^()]*|\((?:[^()]*|\([^()]*\))*\))*\)\s*;?/g;
    return {
        name: 'strip-debug',
        transform(code, id) {
            if (!id.endsWith('.ts') && !id.endsWith('.js')) return null;
            if (!code.includes('assert(')) return null;
            return { code: code.replace(re, ''), map: null };
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
        plugins: [compilecat({ debug: true }), stripDebug(), filesize()],
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
