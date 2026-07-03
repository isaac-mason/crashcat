import { defineConfig } from '@pmndrs/labs'

export default defineConfig({
  benchDir: '.',
  // Macro physics benches need bigger budgets than the 5s default — one
  // sample is a multi-hundred-ms world.step loop, so reaching the 14-sample
  // floor for statistical comparison needs a large CPU budget per bench.
  // hull-heap at ~2.3s/iter needs ~46s for 20 samples; 60s is comfortable.
  maxCpuTime: 60,
})
