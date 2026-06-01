import compilecat from 'compilecat/rolldown';
export default [{
    input: './src/index.ts',
    external: [],
    output: [{ file: 'dist-test/index.js', format: 'es', sourcemap: false, exports: 'named' }],
    plugins: [compilecat({ debug: false })],
}];
