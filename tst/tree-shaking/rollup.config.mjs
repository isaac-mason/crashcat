import { nodeResolve } from '@rollup/plugin-node-resolve';
import filesize from 'rollup-plugin-filesize';

export default {
	input: './entry.js',
	external: [],
	output: {
		file: 'dist/output.js',
		format: 'es',
	},
	plugins: [
		nodeResolve(),
		filesize(),
	],
};
