import path from 'node:path';
import typescript from '@rollup/plugin-typescript';
export default [{
    input: './src/index.ts',
    external: [],
    output: [{ file: 'dist-test/index.js', format: 'es', sourcemap: false, exports: 'named' }],
    plugins: [typescript({ tsconfig: path.resolve(import.meta.dirname, './tsconfig.json'), emitDeclarationOnly: true })],
}];
