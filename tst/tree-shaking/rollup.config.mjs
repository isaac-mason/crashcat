import { nodeResolve } from '@rollup/plugin-node-resolve';

export default {
	input: './entry.js',
	external: [],
	output: {
		file: 'dist/output.js',
		format: 'es',
	},
	plugins: [
		nodeResolve(),
	],
};
