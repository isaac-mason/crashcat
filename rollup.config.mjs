import path from 'node:path';
import { nodeResolve } from '@rollup/plugin-node-resolve';
import typescript from '@rollup/plugin-typescript';
import compilecat from 'compilecat/rollup';
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
        external: ['mathcat'],
        output: [
            {
                file: 'dist/index.js',
                format: 'es',
                sourcemap: true,
                exports: 'named',
            },
        ],
        plugins: [
            compilecat(),
            stripDebug(),
            nodeResolve(),
            typescript({
                tsconfig: path.resolve(import.meta.dirname, './tsconfig.json'),
                emitDeclarationOnly: true,
            }),
            filesize(),
        ],
    },
    {
        input: './three/index.ts',
        external: ['mathcat', 'crashcat', 'three'],
        output: [
            {
                file: 'dist/three.js',
                format: 'es',
                sourcemap: true,
                exports: 'named',
            },
        ],
        plugins: [
            stripDebug(),
            nodeResolve(),
            typescript({
                tsconfig: path.resolve(import.meta.dirname, './tsconfig.json'),
                emitDeclarationOnly: true,
            }),
            filesize(),
        ],
    },
];
