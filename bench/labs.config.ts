import { defineConfig } from '@pmndrs/labs'

export default defineConfig({
  benchDir: '.',
  // Macro physics benches need bigger budgets than the 5s default — one
  // sample is a multi-hundred-ms world.step loop, so reaching the 14-sample
  // floor for statistical comparison needs ≥30s of CPU budget per bench.
  // Cube-heap settles in ~30s; bump higher if scenarios grow heavier.
  maxCpuTime: 30,
})
