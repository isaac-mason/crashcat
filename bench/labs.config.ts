import { defineConfig } from '@pmndrs/labs'

export default defineConfig({
  benchDir: '.',
  // Macro physics benches need bigger budgets than the 5s default — one
  // sample is a multi-hundred-ms world.step loop, so reaching the 14-sample
  // floor for statistical comparison needs a large CPU budget per bench.
  // This is a CAP, not a floor: adaptive sampling stops at convergence, so
  // only benches that stay noisy (pyramid at ~1.5s/iter) run this long.
  maxCpuTime: 120,
})
